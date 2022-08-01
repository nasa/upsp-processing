/** @file
 *  @brief  Patch over Image Targets
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/********************************************************************
 * PatchClusters
********************************************************************/

/*****************************************************************************/
template<typename FP>
PatchClusters<FP>::PatchClusters(const std::vector<std::vector<Target_<FP>>>& clusters,
        cv::Size size_in, unsigned int boundary_thickness_in, 
        unsigned int buffer_thickness_in) : size(size_in), 
        boundary_thickness(boundary_thickness_in), buffer_thickness(buffer_thickness_in), 
        bounds_x(clusters.size()), bounds_y(clusters.size()), 
        internal_x(clusters.size()), internal_y(clusters.size()) {

    for (unsigned int i=0; i < clusters.size(); ++i) {
        // Get the boundary and internal points
        std::vector<cv::Point2i> bounds;
        std::vector<cv::Point2i> internal;
        if (clusters[i].size() > 1) {
            get_cluster_boundary(clusters[i], internal, bounds, 
                    boundary_thickness, buffer_thickness);
        } else {
            get_target_boundary(clusters[i][0], internal, bounds, 
                    boundary_thickness, buffer_thickness);
        }

        // Convert points to x/y vectors, remove any that are not in frame
        for (unsigned int j=0; j < internal.size(); ++j) {
            if (contains(size, internal[j])) {
                internal_x[i].push_back(internal[j].x);
                internal_y[i].push_back(internal[j].y);
            }
        }
        for (unsigned int j=0; j < bounds.size(); ++j) {
            if (contains(size, bounds[j])) {
                bounds_x[i].push_back(bounds[j].x);
                bounds_y[i].push_back(bounds[j].y);
            }
        }

        // check
        assert(internal_x[i].size() == internal_y[i].size());
        assert(bounds_x[i].size() == bounds_y[i].size());

    }

}

/*****************************************************************************/
template<typename FP>
template<typename T>
void PatchClusters<FP>::threshold_bounds(cv::Mat_<T> ref, unsigned int thresh, 
        unsigned int offset) {

    int offset_int = (int) offset;
    int zero = 0;
    int col_idxs = ref.cols-1;
    int row_idxs = ref.rows-1;

    for (unsigned int i=0; i < bounds_x.size(); ++i) {
        auto x_it = bounds_x[i].begin();
        auto y_it = bounds_y[i].begin();
        while (x_it != bounds_x[i].end()) {
            // Create a box around the point
            int y_min = std::max(zero, (int) (*y_it)- offset_int);
            int x_min = std::max(zero, (int) (*x_it)- offset_int);
            int width = std::min(col_idxs, (int) (*x_it)+offset_int) - x_min + 1;
            int height = std::min(row_idxs, (int) (*y_it)+offset_int) - y_min + 1;

            cv::Rect bbox(x_min, y_min, width, height);

            // Determine the minimum value within the bounds
            double min_val=0;
            cv::minMaxLoc(ref(bbox), &min_val);

            if (min_val < thresh) {
                x_it = bounds_x[i].erase(x_it);
                y_it = bounds_y[i].erase(y_it);
            } else {
                ++x_it;
                ++y_it;
            }

        }
    }

}

/*****************************************************************************/
template<typename FP>
cv::Mat PatchClusters<FP>::operator()(cv::Mat inp) const {

    unsigned int degree = 3;

    // Convert data to type float if needed
    cv::Mat out;
    if (inp.depth() != CV_32F) {
        inp.convertTo(out, CV_32F);
    } else {
        out = inp.clone();
    }

    for (unsigned int i=0; i < bounds_x.size(); ++i) {
        // skip if too few points
        if (bounds_x[i].size() < ( (degree+2)*(degree+1)/2)) {
            continue;
        } 

        /*
        // check that bounds look real
        cv::Mat tmp_img = convert2_8U(inp);
        cv::cvtColor(tmp_img, tmp_img, CV_GRAY2RGB);
        for (unsigned int j=0; j < bounds_x[i].size(); ++j) {
            tmp_img.at<Vec3b>(bounds_y[i][j],bounds_x[i][j]) = cv::Vec3b(255,0,0);
        }
        cv::String winname = "Boundary Points";
        cv::namedWindow(winname);
        cv::imshow(winname, tmp_img);
        cv::waitKey(0);
        cv::destroyAllWindows();
        */

        // Pull values at boundary, and create poly2d fit
        std::vector<float> bounds_z(bounds_x[i].size());
        for (unsigned int j=0; j < bounds_x[i].size(); ++j) {
            //std::cout << bounds_x[i][j] << " " << bounds_y[i][j] << std::endl;
            bounds_z[j] = out.at<float>(bounds_y[i][j],bounds_x[i][j]);
            //std::cout << bounds_z[j] << std::endl;
            //std::cout << "pulled z" << std::endl;
            //std::cout << bounds_x[i][j] << " " << bounds_y[i][j] << " " << bounds_z[j] << std::endl;
        }
        std::vector<float> poly;
        //std::cout << "fitting" << std::endl;
        polyfit2D(bounds_x[i],bounds_y[i],bounds_z, poly, degree);
        //std::cout << "poly = " ;
        //for (unsigned int j=0; j < poly.size(); ++j) {
        //  std::cout << poly[j] << ", ";
        //}
        //std::cout << std::endl;

        // Evaluate the polynomial at the internal points
        std::vector<float> internal_z;
        //std::cout << "evaluating" << std::endl;
        polyval2D(internal_x[i],internal_y[i],poly,internal_z);
        //std::cout << "finished eval" << std::endl;

        // Adjust Mat at internal points
        //std::cout << internal_z.size() << std::endl;
        for (unsigned int j=0; j < internal_x[i].size(); ++j) {
            //std::cout << internal_z[j] << std::endl;
            out.at<float>(internal_y[i][j],internal_x[i][j]) = internal_z[j];
        }

    }

    return out;
}

/********************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
template<typename I, typename T>
void polyfit2D(const std::vector<I>& x, const std::vector<I>& y,
        const std::vector<T>& z, std::vector<T>& poly, unsigned int degree) {

    assert( (x.size() == y.size()) && (x.size() == z.size()) );
    
    // determine the number of coefficients
    unsigned int coeffs = (degree + 2) * (degree + 1) / 2;

    assert(x.size() >= coeffs);

    // form the matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(x.size(), coeffs);
    unsigned int count = 0;
    for (int ind=0; ind < x.size(); ++ind) {
        count = 0;
        for (int i=0; i <= degree; ++i) {
            for (int j=0; j <= degree; ++j) {
                if ( (i+j) <= degree) {
                    A(ind,count) = (T) std::pow(y[ind],i) * (T) std::pow(x[ind],j);
                    ++count;
                }
            }
        }
    }

    // Initialize polynomial coefficient output vector
    poly.assign(coeffs,0.0);

    // solve
    const Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> z_map(&z[0], z.size());
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> poly_map(&poly[0], coeffs);
    poly_map = A.colPivHouseholderQr().solve(z_map);
}

/*****************************************************************************/
template<typename I, typename T>
void polyval2D(const std::vector<I>& x, const std::vector<I>& y,
        const std::vector<T>& poly, std::vector<T>& z) {

    assert(x.size() == y.size());

    // determine the degree of the polynomial
    unsigned int degree = std::round(-1.5 + sqrt( 9/4 - 2*(1 - poly.size())));

    // check that polynomial is a valid length
    unsigned int coeffs = (degree + 2) * (degree + 1) / 2;
    assert(coeffs == poly.size());

    // evaluate the polynomial at each point
    z.assign(x.size(), 0);
    unsigned int count = 0;
    for (int ind=0; ind < z.size(); ++ind) {
        count = 0;
        for (int i=0; i <= degree; ++i) {
            for (int j=0; j <= degree; ++j) {
                if ( (i+j) <= degree) {
                    assert(count < poly.size());
                    z[ind] += poly[count] * (T) std::pow(y[ind],i) * (T) std::pow(x[ind],j);
                    ++count;
                }
            }
        }
    }
        
}

/*****************************************************************************/
template<typename T>
void cluster_points(const std::vector<T>& targs, 
        std::vector<std::vector<T>>& clusters, int bound_pts/*=4*/) {

    clusters.clear();

    typedef typename T::data_type FP;
    typedef std::list<unsigned int>::iterator list_iterator;

    boost::counting_iterator<unsigned int> cit(0);
    std::list<unsigned int> pts(cit, cit+targs.size());

    std::queue<unsigned int> que;
    auto it = pts.begin();
    while (it != pts.end()) {
        clusters.push_back(std::vector<T>(1,targs[*it]));
        que.push(0);
        pts.erase(it);
        while (!que.empty()) {
            T ref = clusters[clusters.size()-1][que.front()];
            que.pop();

            auto it2 = pts.begin();
            while (it2 != pts.end()) {
                if ( cv::norm(ref.uv - targs[*it2].uv) <= 
                        ((FP) bound_pts + 0.5*(ref.diameter + targs[*it2].diameter))) {
                    que.push(clusters[clusters.size()-1].size());
                    clusters[clusters.size()-1].push_back(targs[*it2]);
                    it2 = pts.erase(it2);
                } else {
                    ++it2;
                }
            }
        }
        it = pts.begin();
    }

}

/*****************************************************************************/
template<typename T>
void get_target_boundary(const T& targ, cv::Point2i& t_min, cv::Point2i& t_max) {
    t_min.x = std::floor(targ.uv.x - 0.5*targ.diameter);
    t_min.y = std::floor(targ.uv.y - 0.5*targ.diameter);
    t_max.x = std::ceil(targ.uv.x + 0.5*targ.diameter);
    t_max.y = std::ceil(targ.uv.y + 0.5*targ.diameter);
}

/*****************************************************************************/
template<typename T>
void get_target_boundary(const T& targ, std::vector<cv::Point2i>& internal,
        std::vector<cv::Point2i>& bounds, unsigned int ubound_pts /*=2*/, 
        unsigned int ubuffer /*=0*/) {

    // The fact that the input args "bound_pts" and "buffer" are unsigned,
    // caused headaches in the expressions below (when negative values for
    // t_min.x and t_max.x got promoted to unsigned).  We deal with this by
    // slightly renaming the input args, and then creating a local var with
    // the old name, but of type "int".  Basically, we convert the input
    // args to be of type int, but without actually changing the interface
    // definition.
    int bound_pts = (int) ubound_pts;
    int buffer = (int) ubuffer;


    internal.clear();
    bounds.clear();

    // get the target boundary
    cv::Point2i t_min, t_max;
    get_target_boundary(targ, t_min, t_max);

    // fill internal with all points inside the boundary
    for (int x=t_min.x; x <= t_max.x; ++x) {
        for (int y=t_min.y; y <= t_max.y; ++y) {
            internal.push_back(cv::Point2i(x,y));
        }
    }

    // fill boundary points
    for (int x=t_min.x-bound_pts-buffer; x <= t_max.x+bound_pts+buffer; ++x) {
        for (int y=t_min.y-bound_pts-buffer; y <= t_max.y+bound_pts+buffer; ++y) {
            if ( (x < (t_min.x-buffer)) || (x > (t_max.x+buffer)) ||
                    (y < (t_min.y-buffer)) || (y > (t_max.y+buffer)) ) {
                bounds.push_back(cv::Point2i(x,y));
            }
        }
    }
}

/*****************************************************************************/
template<typename T>
void get_cluster_boundary(const std::vector<T>& targs, 
        std::vector<cv::Point2i>& internal, std::vector<cv::Point2i>& bounds, 
        unsigned int bound_pts /*=2*/, unsigned int buffer /*=0*/) {

    internal.clear();
    bounds.clear();

    // Bound the space
    std::vector<cv::Point2i> tg_mins(targs.size());
    std::vector<cv::Point2i> tg_maxs(targs.size());

    cv::Point2i t_max(0,0);
    cv::Point2i t_min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    for (unsigned int i=0; i < targs.size(); ++i) {
        get_target_boundary(targs[i], tg_mins[i], tg_maxs[i]);

        t_min.x = std::min(t_min.x, tg_mins[i].x);
        t_min.y = std::min(t_min.y, tg_mins[i].y);

        t_max.x = std::max(t_max.x, tg_maxs[i].x);
        t_max.y = std::max(t_max.y, tg_maxs[i].y);
    }

    // Add boundary points and create a mesh
    t_min -= cv::Point2i(bound_pts+buffer, bound_pts+buffer);
    t_max += cv::Point2i(bound_pts+buffer, bound_pts+buffer);

    unsigned int d_x = t_max.x - t_min.x + 1;
    unsigned int d_y = t_max.y - t_min.y + 1;
    Eigen::MatrixXi cluster = Eigen::MatrixXi::Constant(d_x,d_y,0);

    //std::cout << "Full mesh size = [" << d_x << ", " << d_y << "]" << std::endl;

    // Define the target points
    for (unsigned int i=0; i < targs.size(); ++i) {
        // translate limits to mesh indices
        tg_mins[i] -= t_min;
        tg_maxs[i] -= t_min;

        assert(tg_mins[i].x >= 0);
        assert(tg_mins[i].y >= 0);
        assert(tg_maxs[i].x < d_x);
        assert(tg_maxs[i].y < d_y);

        // fill mesh
        for (int x = tg_mins[i].x; x <= tg_maxs[i].x; ++x) {
            for (int y = tg_mins[i].y; y <= tg_maxs[i].y; ++y) {
                cluster(x,y) = 2;
            }
        }
    }

    // Mark points between targets as internal
    for (unsigned int x=0; x < d_x; ++x) {
        unsigned int min_y = d_y;
        unsigned int max_y = d_y;
        for (unsigned int y=0; y < d_y; ++y) {
            if (cluster(x,y) == 2) {
                min_y = y;
                break;
            }
        }
        if (min_y == d_y) {
            continue;
        }
        for (unsigned int y=d_y-1; y >= min_y; --y) {
            if (cluster(x,y) == 2) {
                max_y = y;
                break;
            }
        }
        assert(max_y != d_y);
        for (unsigned int y=min_y; y <= max_y; ++y) {
            cluster(x,y) = 2;
        }
    }
 
    for (unsigned int y=0; y < d_y; ++y) {
        unsigned int min_x = d_x;
        unsigned int max_x = d_x;
        for (unsigned int x=0; x < d_x; ++x) {
            if (cluster(x,y) == 2) {
                min_x = x;
                break;
            }
        }
        if (min_x == d_x) {
            continue;
        }
        for (unsigned int x=d_x-1; x >= min_x; --x) {
            if (cluster(x,y) == 2) {
                max_x = x;
                break;
            }
        }
        assert(max_x != d_x);
        for (unsigned int x=min_x; x <= max_x; ++x) {
            cluster(x,y) = 2;
        }
    }

    // Add internal and boundary points to output vectors
    unsigned int min_x, min_y, len_x, len_y;
    unsigned int buf_min_x, buf_min_y, buf_len_x, buf_len_y;
    for (unsigned int x=0; x < d_x; ++x) {
        if (x <= (bound_pts+buffer)) {
            min_x = 0;
        } else {
            min_x = x - bound_pts - buffer;
        }
        len_x = std::min(x+bound_pts+buffer,d_x-1) - min_x + 1;
        if (x <= buffer) {
            buf_min_x = 0;
        } else {
            buf_min_x = x - buffer;
        }
        buf_len_x = std::min(x+buffer, d_x-1) - buf_min_x + 1;
        for (unsigned int y=0; y < d_y; ++y) {
            if (y <= (bound_pts+buffer)) {
                min_y = 0;
            } else {
                min_y = y - bound_pts - buffer;
            }
            len_y = std::min(y+bound_pts+buffer,d_y-1) - min_y + 1;

            if (cluster(x,y) == 2) {
                internal.push_back(cv::Point2i(x+t_min.x,y+t_min.y));
                continue;
            }
            if ( (bound_pts > 0) && (buffer > 0) ) {
                if (y <= buffer) {
                    buf_min_y = 0;
                } else {
                    buf_min_y = y - buffer;
                }
                buf_len_y = std::min(y+buffer, d_y-1) - buf_min_y + 1;

                if (cluster.block(buf_min_x,buf_min_y,buf_len_x,buf_len_y).maxCoeff() != 2) {
                    if (cluster.block(min_x,min_y,len_x,len_y).maxCoeff() == 2) {
                        bounds.push_back(cv::Point2i(x+t_min.x,y+t_min.y));
                        cluster(x,y) = 1;
                    }
                }
                continue;
            }
            if (bound_pts > 0) {
                if (cluster.block(min_x,min_y,len_x,len_y).maxCoeff() == 2) {
                    bounds.push_back(cv::Point2i(x+t_min.x,y+t_min.y));
                    cluster(x,y) = 1;
                }
            }
        }
    }

    //std::cout << cluster << std::endl << std::endl;

}

} /* end namespace upsp */
