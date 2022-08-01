/** @file
 * 
 *  @brief  Image and Point Set Registration
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename FP>
void unmatch_outliers(const std::vector<cv::Point_<FP>>& ref,
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        float cutoff /*=3.0*/) {

    float avg_diff = 0.0;
    unsigned int matched_pts = 0;
    for (unsigned int i=0; i < ref.size(); ++i) {
        if (matches[i] != -1) {
            avg_diff += cv::norm(ref[i] - pts[matches[i]]);
            ++matched_pts;
        }
    }
    avg_diff /= (float) matched_pts;

    float std_diff = 0.0;
    for (unsigned int i=0; i < ref.size(); ++i) {
        if (matches[i] != -1) {
            std_diff += pow( cv::norm(ref[i] - pts[matches[i]]) - avg_diff, 2);
        }
    }
    std_diff = sqrt( std_diff / ((float) (matched_pts - 1.0)) );

    float max_dist = avg_diff + cutoff*std_diff;
    for (unsigned int i=0; i < ref.size(); ++i) {
        if (matches[i] != -1) {
            if ( cv::norm(ref[i] - pts[matches[i]]) > max_dist ) {
                matches[i] = -1;
            }
        }
    }
}   

/*****************************************************************************/
template<typename T>
int find_targets(const cv::Mat& src, std::vector<T>& targs, 
        double min_diameter/*=2.0*/, double max_diameter/*=0.0*/) {

    int sz = 0;

    // Blob Detector requires 8-bit Mat
    cv::Mat filtered;
    if (src.depth() != CV_8U) {
        filtered = convert2_8U(src);
    } else {
        filtered = src.clone();
    }

    // Filter the image
    cv::GaussianBlur(filtered, filtered, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    // Set blob detector parameters
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;//10;//15;//10;
    params.thresholdStep = 3; //3;//8;//6;
    params.maxThreshold = 256;//200;//40;//100;
    params.filterByArea = true;
    params.minArea = PI * std::pow(min_diameter*0.5, 2);
    params.minDistBetweenBlobs = 2;

    // if the max_diameter param is not set, use ratios of the image size
    if (max_diameter <= 0.0) {
        double max_diam = std::max(src.size[0], src.size[1]) * 0.1;
        params.maxArea = PI * std::pow(max_diam*0.5, 2);
    } else {
        params.maxArea = PI * std::pow(max_diameter*0.5, 2);
    }

    params.filterByCircularity = true;
    params.minCircularity = 0.4;
    params.filterByConvexity = true;
    params.minConvexity = 0.90;
    params.filterByInertia = false;

    // Find the blobs
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(filtered, keypoints);

    sz = keypoints.size();

    targs.clear();
    if (sz != 0) {
        targs.resize(sz);
        for (int i=0; i < sz; ++i) {
            targs[i].uv = keypoints[i].pt;
            targs[i].diameter = keypoints[i].size;
        }
    } 

    return sz;
}

/*****************************************************************************/
template<typename T>
int find_targets_roi(const cv::Mat& src, const std::vector<cv::Rect>& roi, 
        std::vector<std::vector<T>>& targs, double min_diameter/*=2.0*/, 
        double max_diameter/*=0.0*/) {

    int sz = 0;

    targs.resize(roi.size());

    if (roi.size() == 0) {
        return 0;
    }

    // Blob Detector requires 8-bit Mat
    cv::Mat filtered;
    if (src.depth() != CV_8U) {
        filtered = convert2_8U(src);
    } else {
        filtered = src.clone();
    }

    // Filter the image
    cv::GaussianBlur(filtered, filtered, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

    // Set blob detector parameters
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;//10;
    params.thresholdStep = 3;//6;
    params.maxThreshold = 256;//100;
    params.filterByArea = true;
    params.minArea = PI * std::pow(min_diameter*0.5, 2);

    // if the max_diameter param is not set, use ratios of the image size
    if (max_diameter <= 0.0) {
        double max_diam = std::max(src.size[0], src.size[1]) * 0.1;
        params.maxArea = PI * std::pow(max_diam*0.5, 2);
    } else {
        params.maxArea = PI * std::pow(max_diameter*0.5, 2);
    }

    params.filterByCircularity = true;
    params.minCircularity = 0.4;
    params.filterByConvexity = true;
    params.minConvexity = 0.90;
    params.filterByInertia = false;

    // Create the detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Find blobs in each roi
    for (int i=0; i < roi.size(); ++i) {
        cv::Mat img = filtered(roi[i]);
        std::vector<cv::KeyPoint> keypoints;
        cv::Point2f offset(roi[i].tl());

        detector->detect(img, keypoints);

        sz += keypoints.size();

        targs[i].clear();
        if (sz != 0) {
            targs[i].resize(sz);
            for (int j=0; j < keypoints.size(); ++j) {
                targs[i][j].uv = keypoints[j].pt + offset; // pt is relative to roi
                targs[i][j].diameter = keypoints[j].size;
            }
        } 
    }
    
    return sz;
}

/*****************************************************************************/
template<typename InputIterator1, typename ForwardIterator2>
void closest_point(const InputIterator1 ref_begin, const InputIterator1 ref_end,
        ForwardIterator2 pts_begin, ForwardIterator2 pts_end, 
        std::vector<int>& matches) {

    matches.clear();
    if (pts_begin == pts_end) {
        matches.assign(std::distance(ref_begin, ref_end), -1);
        return;
    }

    int count1 = 0, count2;
    double dist, min_dist;
    for (auto it_ref = ref_begin; it_ref != ref_end; ++it_ref) {
        matches.push_back(-1);
        min_dist = std::numeric_limits<double>::infinity(); 
        count2 = 0;
        for (auto it_pts = pts_begin; it_pts != pts_end; ++it_pts) {
            dist = cv::norm(*it_pts - *it_ref);
            if (dist < min_dist) {
                min_dist = dist;
                matches[count1] = count2;
            }
            ++count2;
        }
        ++count1;
    }

}

/*****************************************************************************/
template<typename Pt>
void closest_point(const std::vector<Pt>& ref, const std::vector<Pt>& pts, 
        std::vector<int>& matches) {
    closest_point(ref.begin(), ref.end(), pts.begin(), pts.end(), matches);
}

/*****************************************************************************/
template<typename InputIterator1, typename ForwardIterator2>
void closest_point2(const InputIterator1 ref_begin, const InputIterator1 ref_end,
        ForwardIterator2 pts_begin, ForwardIterator2 pts_end, 
        std::vector<int>& matches) {

    typedef typename std::iterator_traits<ForwardIterator2>::value_type Pt;

    matches.clear();
    if (pts_begin == pts_end) {
        return;
    }

    // find the closest point for each reference point
    closest_point(ref_begin, ref_end, pts_begin, pts_end, matches);

    // copy references and create indices
    std::vector<Pt> ref(ref_begin, ref_end);
    boost::counting_iterator<unsigned int> cit(0);
    std::vector<unsigned int> ref_idx(cit, cit+ref.size());

    // remove unmatched references
    auto it = ref.begin();
    auto it_idx = ref_idx.begin();
    while (it != ref.end()) {
        if (matches[*it_idx] == -1) {
            it = ref.erase(it);
            it_idx = ref_idx.erase(it_idx);
        } else {
            ++it;
            ++it_idx;
        }
    }

    // if multiple reference points share a pt, the closest one gets it
    it = ref.begin();
    it_idx = ref_idx.begin();
    int match;
    while (it != ref.end()) {
        match = matches[*it_idx];
        auto it2 = it + 1;
        auto it2_idx = it_idx + 1;
        bool bad_match = false;
        while (it2 != ref.end()) {
            if (matches[*it2_idx] == match) {
                if (cv::norm(*(pts_begin+match) - *it) < 
                        cv::norm(*(pts_begin+match) - *it2)) {
                    matches[*it2_idx] = -1;
                    it2 = ref.erase(it2);
                    it2_idx = ref_idx.erase(it2_idx);
                } else {
                    matches[*it_idx] = -1;
                    bad_match = true;
                    break;
                }
            } else {
                ++it2;
                ++it2_idx;
            }
        }
        if (bad_match) {
            it = ref.erase(it);
            it_idx = ref_idx.erase(it_idx);
        } else {
            ++it;
            ++it_idx;
        }
    }
}

/*****************************************************************************/
template<typename Pt>
void closest_point2(const std::vector<Pt>& ref, const std::vector<Pt>& pts, 
        std::vector<int>& matches) {
    closest_point2(ref.begin(), ref.end(), pts.begin(), pts.end(), matches);
}

/*****************************************************************************/
template<typename FP>
void closest_point3(const std::vector<Target_<FP>>& ref, 
        const std::vector<Target_<FP>>& pts, std::vector<int>& matches, 
        float size_buff/* = 0.1*/) {
    closest_point3(ref.begin(), ref.end(), pts.begin(), pts.end(), matches, 
            size_buff);
}

/*****************************************************************************/
template<typename InputIterator1, typename ForwardIterator2>
void closest_point3(const InputIterator1 ref_begin, 
        const InputIterator1 ref_end, ForwardIterator2 pts_begin, 
        ForwardIterator2 pts_end, std::vector<int>& matches, 
        float size_buff) {

    typedef typename std::iterator_traits<ForwardIterator2>::value_type Pt;

    matches.clear();
    if (pts_begin == pts_end) {
        return;
    }

    // find the closest point for each reference point
    matches.clear();
    if (pts_begin == pts_end) {
        matches.assign(std::distance(ref_begin, ref_end), -1);
        return;
    }

    int count1 = 0, count2;
    double dist, min_dist, sz;
    for (auto it_ref = ref_begin; it_ref != ref_end; ++it_ref) {
        matches.push_back(-1);
        min_dist = std::numeric_limits<double>::infinity(); 
        count2 = 0;
        sz = (*it_ref).diameter;
        for (auto it_pts = pts_begin; it_pts != pts_end; ++it_pts) {
            if ( ((*it_pts).diameter > (sz*(1.0+size_buff))) ||
                 ((*it_pts).diameter < (sz*(1.0-size_buff))) ) {
                ++count2;
                continue;
            }
            dist = cv::norm((*it_pts).uv - (*it_ref).uv);
            if (dist < min_dist) {
                min_dist = dist;
                matches[count1] = count2;
            }
            ++count2;
        }
        ++count1;
    }

    // copy references and create indices
    std::vector<Pt> ref(ref_begin, ref_end);
    boost::counting_iterator<unsigned int> cit(0);
    std::vector<unsigned int> ref_idx(cit, cit+ref.size());

    // remove unmatched references
    auto it = ref.begin();
    auto it_idx = ref_idx.begin();
    while (it != ref.end()) {
        if (matches[*it_idx] == -1) {
            it = ref.erase(it);
            it_idx = ref_idx.erase(it_idx);
        } else {
            ++it;
            ++it_idx;
        }
    }

    // if multiple reference points share a pt, the closest one gets it
    it = ref.begin();
    it_idx = ref_idx.begin();
    int match;
    while (it != ref.end()) {
        match = matches[*it_idx];
        auto it2 = it + 1;
        auto it2_idx = it_idx + 1;
        bool bad_match = false;
        while (it2 != ref.end()) {
            if (matches[*it2_idx] == match) {
                if (cv::norm((*(pts_begin+match)).uv - (*it).uv) < 
                        cv::norm((*(pts_begin+match)).uv - (*it2).uv)) {
                    matches[*it2_idx] = -1;
                    it2 = ref.erase(it2);
                    it2_idx = ref_idx.erase(it2_idx);
                } else {
                    matches[*it_idx] = -1;
                    bad_match = true;
                    break;
                }
            } else {
                ++it2;
                ++it2_idx;
            }
        }
        if (bad_match) {
            it = ref.erase(it);
            it_idx = ref_idx.erase(it_idx);
        } else {
            ++it;
            ++it_idx;
        }
    }
}

/*****************************************************************************/
template<typename FP>
double iterative_closest_point(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters/*=100*/, double resid/*=1.0e-7*/) {

    double relaxation = 0.2;

    matches.clear();
    if (pts.size() == 0) {
        return 0.0;
    }
    matches.assign(pts.size(), -1);
    if (ref.size() == 0) {
        return 0.0;
    }

    // create a copy of the pts to transform
    std::vector<cv::Point_<FP>> trans_pts(pts);
    std::vector<cv::Point_<FP>> p_trans_pts;
    
    double last_err = 0.0, err;
    std::vector<cv::Point_<FP>> tmp_pts(ref.size());
    std::vector<cv::Point_<FP>> tmp_ref(ref.size());
    std::vector<cv::Point_<FP>> buf;
    cv::Mat trans_map;
    for (int i=0; i < iters; ++i) {
        // find the closest point to each ref
        closest_point2(ref, trans_pts, matches);

        // create vectors that match up the used ref pts and pts
        // if not matched (matches = -1), don't include in homography
        tmp_pts.resize(ref.size());
        tmp_ref.resize(ref.size());
        unsigned int n_matches = 0;
        for (int j=0; j < tmp_pts.size(); ++j) {
            if (matches[j] >= 0) {
                tmp_ref[n_matches] = ref[j];
                tmp_pts[n_matches++] = pts[matches[j]];
            }
        }
        tmp_pts.resize(n_matches);
        tmp_ref.resize(n_matches);

        // obtain the transformation between matched pts and ref
        trans_map = cv::findHomography(tmp_pts, tmp_ref, cv::RANSAC);

        // apply the transformation to the points
        p_trans_pts.clear();
        cv::perspectiveTransform(pts, p_trans_pts, trans_map);

        for (unsigned int j=0; j < p_trans_pts.size(); ++j) {
            trans_pts[j] = trans_pts[j]*(1.0-relaxation) + p_trans_pts[j]*relaxation;
        }

        // compute l2-norm of distance between ref and transformed pts
        err = 0.0;
        for (int i=0; i < ref.size(); ++i) {
            err += std::pow(cv::norm(trans_pts[matches[i]] - ref[i]),2);
        }
        err = std::sqrt(err);

        /*
        std::cout << "Iter " << i << " error = " << err << std::endl;
        std::cout << "Iter " << i << " error resid = ";
        std::cout << std::abs(err - last_err) << std::endl;
        for (int j=0; j < ref.size(); ++j) {
            std::cout << ref[j] << " maps to " << matches[j] << ": ";
            std::cout << trans_pts[matches[j]] << std::endl;
        }
        */
    
        // check if meet residual requirement
        if (i > 0) {
            if (std::abs(err - last_err) < resid) {
                break;
            }
        }
        last_err = err;
    }

    if (iters > 0) {
        // complete the matching
        closest_point2(ref, trans_pts, matches);

        // remove outliers
        unmatch_outliers(ref, trans_pts, matches, 3.0);
    }

    return std::sqrt(err);
}

/*****************************************************************************/
template<typename FP>
double iterative_closest_point2(const std::vector<Target_<FP>>& ref, 
        const std::vector<Target_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters/*=100*/, double resid/*=1.0e-7*/,
        double size_buff/*=0.1*/) {

    double relaxation = 0.2;
    
    matches.clear();
    if (pts.size() == 0) {
        return 0.0;
    }
    matches.assign(pts.size(), -1);
    if (ref.size() == 0) {
        return 0.0;
    }

    // Get a vector Points for use in openCV functions
    std::vector<cv::Point_<FP>> p_pts;

    upsp::Target2Point2<FP> t2p2;    
    p_pts.assign(boost::make_transform_iterator(pts.begin(), t2p2),
            boost::make_transform_iterator(pts.end(), t2p2));

    // create a copy of the pts to transform
    std::vector<Target_<FP>> trans_pts(pts);
    std::vector<cv::Point_<FP>> p_trans_pts(p_pts);
    
    double last_err = 0.0, err;
    std::vector<cv::Point_<FP>> tmp_pts(ref.size());
    std::vector<cv::Point_<FP>> tmp_ref(ref.size());
    //std::vector<Target_<FP>> buf;
    cv::Mat trans_map;
    for (int i=0; i < iters; ++i) {
        // find the closest point to each ref
        closest_point3(ref, trans_pts, matches, size_buff);

        // create vectors that match up the used ref pts and pts
        // if not matched (matches = -1), don't include in homography
        tmp_pts.resize(ref.size());
        tmp_ref.resize(ref.size());
        unsigned int n_matches = 0;
        for (int j=0; j < tmp_pts.size(); ++j) {
            if (matches[j] >= 0) {
                tmp_ref[n_matches] = ref[j].uv;
                tmp_pts[n_matches++] = trans_pts[matches[j]].uv;
            }
        }
        tmp_pts.resize(n_matches);
        tmp_ref.resize(n_matches);

        if (n_matches < 4) {
            std::cerr << "Unable to perform icp, insufficient matches";
            std::cerr << " based on point size" << std::endl;
            std::abort();
        }
        std::cout << "n_matches = " << n_matches << std::endl;

        // obtain the transformation between matched pts and ref
        trans_map = cv::findHomography(tmp_pts, tmp_ref, cv::RANSAC);

        // apply the transformation to the points
        p_pts = std::vector<cv::Point_<FP>>(p_trans_pts);
        p_trans_pts.clear();
        cv::perspectiveTransform(p_pts, p_trans_pts, trans_map);

        for (unsigned int j=0; j < p_trans_pts.size(); ++j) {
            trans_pts[j].uv = trans_pts[j].uv*(1.0-relaxation) + p_trans_pts[j]*relaxation;
        }

        // compute l2-norm of distance between ref and transformed pts
        err = 0.0;
        for (int i=0; i < ref.size(); ++i) {
            err += std::pow(cv::norm(trans_pts[matches[i]].uv - ref[i].uv),2);
        }
        err = std::sqrt(err);

        std::cout << "Iter " << i << " error = " << err << std::endl;
        std::cout << "  error resid = ";
        std::cout << std::abs(err - last_err) << std::endl;
        /*
        for (int j=0; j < ref.size(); ++j) {
            std::cout << ref[j] << " maps to " << matches[j] << ": ";
            std::cout << trans_pts[matches[j]] << std::endl;
        }
        */

        // check if meet residual requirement
        if (i > 0) {
            if (std::abs(err - last_err) < resid) {
                break;
            }
        }
        last_err = err;
    }

    if (iters > 0) {
        // complete the matching
        closest_point3(ref, trans_pts, matches, size_buff);

        // remove outliers
        std::vector<cv::Point_<FP>> p_ref;
        p_ref.assign(boost::make_transform_iterator(ref.begin(), t2p2),
                boost::make_transform_iterator(ref.end(), t2p2));
        p_trans_pts.assign(boost::make_transform_iterator(trans_pts.begin(), t2p2),
                boost::make_transform_iterator(trans_pts.end(), t2p2));
        unmatch_outliers(p_ref, p_trans_pts, matches, 3.0);
    }

    return std::sqrt(err);
}

/*****************************************************************************/
template<typename FP>
float cpd_affine(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters/*=100*/, float resid/*=1e-5*/) {

    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    matches.clear();
    if (pts.size() == 0) {
        return 0.0;
    }
    matches.assign(pts.size(), -1);
    if (ref.size() == 0) {
        return 0.0;
    }

    // Set free parameters
    float w = 0.1;

    // Get sizes
    int N = ref.size();
    int M = pts.size();
    int D = 2;

    // Convert points to matrices
    Matrix x(N,D);
    for (unsigned int i=0; i < N; ++i) {
        x(i,0) = (double) ref[i].x;
        x(i,1) = (double) ref[i].y;
    }
    
    Matrix y(M,D);
    for (unsigned int i=0; i < M; ++i) {
        y(i,0) = (double) pts[i].x;
        y(i,1) = (double) pts[i].y;
    }

    // Normalize inputs
    Eigen::Vector2d x_mean = x.colwise().mean();
    Eigen::Vector2d y_mean = y.colwise().mean();
    x -= Vector::Constant(N,1.0) * x.colwise().mean();  
    y -= Vector::Constant(M,1.0) * y.colwise().mean();  

    Eigen::Vector2d x_std = ( (x.array()*x.array()).colwise().sum() / (N-1.0) ).sqrt();
    Eigen::Vector2d y_std = ( (y.array()*y.array()).colwise().sum() / (M-1.0) ).sqrt();

    x.block(0,0,N,1) /= x_std[0];
    x.block(0,1,N,1) /= x_std[1];
    y.block(0,0,M,1) /= y_std[0];
    y.block(0,1,M,1) /= y_std[1];

    //std::cout << "x_mean = " << x_mean << std::endl;
    //std::cout << "x_std = " << x_std << std::endl;

    //std::cout << "x = " << x << std::endl;
    //std::cout << "y = " << y << std::endl;

    // Initialize terms
    double sigma2 = 0.0;
    for (unsigned int n=0; n < N; ++n) {
        Matrix x_hat = x.block(n,0,1,2);
        for (unsigned int m=0; m < M; ++m) {
            Matrix y_hat = y.block(m,0,1,2);
            y_hat = y_hat - x_hat;
            sigma2 += y_hat.squaredNorm();
        }
    }
    sigma2 /= (double) (D*N*M);

    // Iterate
    Matrix B(D,D);
    Vector t(D);
    Matrix T = y;
    Matrix old_T = y;
    double T_resid = resid + 1.0;
    for (unsigned int i=0; i < iters; ++i) {
        if (T_resid < resid) {
            break;
        }

        // E-step : Compute P
        Matrix P(M,N);
        for (unsigned int n=0; n < N; ++n) {
            Matrix x_block = x.block(n,0,1,2);
            Vector g(M);
            for (unsigned int m=0; m < M; ++m) {
                Matrix T_block = T.block(m,0,1,2);
                T_block = T_block - x_block;
                g(m) = exp( -1.0 / 2.0 * (T_block.squaredNorm()/sigma2) );
            }
            P.block(0,n,M,1) = g / ( g.sum() + pow(2.0*PI*sigma2,D/2.0) * w / (1.0-w) * M / N );
        }

        // M-step : Solve for W, sigma2, update T
        Vector vec_dP1 = P * Vector::Constant(N,1.0);
        Vector vec_dtP1 = P.transpose() * Vector::Constant(M,1.0);
        auto dP1 = vec_dP1.asDiagonal();

        double N_p = P.sum();
        Vector mu_x = 1.0/N_p * x.transpose() * vec_dtP1;
        Vector mu_y = 1.0/N_p * y.transpose() * vec_dP1;

        Matrix x_hat = x - Vector::Constant(N,1.0)*mu_x.transpose();
        Matrix y_hat = y - Vector::Constant(M,1.0)*mu_y.transpose();

        Matrix RMat = x_hat.transpose() * P.transpose() * y_hat; 
        Matrix LMat = y_hat.transpose() * dP1 * y_hat;

        B = RMat * LMat.inverse();
        t = mu_x - B*mu_y;

        //std::cout << "B = " << B << std::endl;
        //std::cout << "t = " << t << std::endl;

        T = y*B.transpose() + Vector::Constant(M,1.0)*t.transpose();

        sigma2 = (1.0 / (N_p*D)) * ( (x_hat.transpose() * vec_dtP1.asDiagonal() 
                * x_hat).trace() - (x_hat.transpose() * P.transpose() * y_hat * 
                B.transpose()).trace() );

        //std::cout << "sigma2 = " << sigma2 << std::endl;

        if (sigma2 <= 0.0) {
            break;
        }

        // Compute residual
        T_resid = (T - old_T).lpNorm<2>();
        old_T = T;
        //std::cout << "T residual = " << T_resid << std::endl;

/*
        // Match points with closest point
        std::vector<cv::Point_<FP>> new_pts(M);
        for (unsigned int i=0; i < M; ++i) {
            new_pts[i] = cv::Point_<FP>(T(i,0)*x_std[0] + x_mean[0],T(i,1)*x_std[1]+x_mean[1]);
            //new_pts[i] = cv::Point_<FP>(T(i,0),T(i,1));
        }
        closest_point2(ref, new_pts, matches);
    
        cv::Mat img(400, 896, CV_8U, 1);
        img = add_points(ref, img, cv::Scalar(0,255,0), false);
        img = add_points(new_pts, img, cv::Scalar(0,0,255), false);
        cv::String winname = "comp_points";
        cv::namedWindow(winname);
        cv::imshow(winname, img);
        cv::waitKey(0);
        cv::destroyAllWindows();    
*/
    }

    // Reverse normalization
    T.block(0,0,M,1) = T.block(0,0,M,1) * x_std[0] + x_mean[0]*Vector::Constant(M,1);
    T.block(0,1,M,1) = T.block(0,1,M,1) * x_std[1] + x_mean[1]*Vector::Constant(M,1);

    // Match points with closest points and reject outliers
    std::vector<cv::Point_<FP>> new_pts(M);
    for (unsigned int i=0; i < M; ++i) {
        new_pts[i] = cv::Point_<FP>((FP)T(i,0),(FP)T(i,1));
    }
    closest_point2(ref, new_pts, matches);

    unmatch_outliers(ref, new_pts, matches, 5.0);
    
/*
    cv::Mat img(400, 896,CV_8U);
    img = add_points(ref, img, cv::Scalar(0,255,0), false);
    img = add_points(new_pts, img, cv::Scalar(0,0,255), false);
    cv::String winname = "comp_points";
    cv::namedWindow(winname);
    cv::imshow(winname, img);
    cv::waitKey(0);
    cv::destroyAllWindows();    
*/
    // Compute difference from reference
    float diff_norm = 0;
    for (unsigned int i=0; i < N; ++i) {
        if (matches[i] != -1) {
            diff_norm += pow(ref[i].x - new_pts[matches[i]].x,2) + 
                    pow(ref[i].y - new_pts[matches[i]].y,2);
        }
    }

    return sqrt(diff_norm);
}

/*****************************************************************************/
template<typename FP>
float coherent_point_drift(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters/*=100*/, float resid/*=1e-6*/) {

    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    matches.clear();
    if (pts.size() == 0) {
        return 0.0;
    }
    matches.assign(pts.size(), -1);
    if (ref.size() == 0) {
        return 0.0;
    }

    // Set free parameters
    double w = 0.03;
    double beta = 3.0;
    double lambda = 0.8;
    //double alpha = 0.97;

    // Get sizes
    int N = ref.size();
    int M = pts.size();
    int D = 2;

    // Convert points to matrices
    Matrix x(N,2);
    for (unsigned int i=0; i < N; ++i) {
        x(i,0) = (double) ref[i].x;
        x(i,1) = (double) ref[i].y;
    }
    
    Matrix y(M,2);
    for (unsigned int i=0; i < M; ++i) {
        y(i,0) = (double) pts[i].x;
        y(i,1) = (double) pts[i].y;
    }

    // Normalize inputs
    Eigen::Vector2d x_mean = x.colwise().mean();
    x -= Vector::Constant(N,1.0) * x.colwise().mean();  
    y -= Vector::Constant(M,1.0) * y.colwise().mean();  

    Eigen::Vector2d x_std = ( (x.array()*x.array()).colwise().sum() / (N-1.0) ).sqrt();
    Eigen::Vector2d y_std = ( (y.array()*y.array()).colwise().sum() / (M-1.0) ).sqrt();

    x.block(0,0,N,1) /= x_std[0];
    x.block(0,1,N,1) /= x_std[1];
    y.block(0,0,M,1) /= y_std[0];
    y.block(0,1,M,1) /= y_std[1];

    //std::cout << "x = " << x << std::endl;
    //std::cout << "y = " << y << std::endl;

    // Initialize terms
    Matrix G(M,M);
    for (unsigned int i=0; i < M; ++i) {
        Matrix y1 = y.block(i,0,1,2);
        for (unsigned int j=0; j < M; ++j) {
            Matrix y2 = y.block(j,0,1,2);
            y2 = y2 - y1;
            G(i,j) = exp( -1.0 / (2*pow(beta,2)) * y2.squaredNorm() );
        }
    }

    double sigma2 = 0.0;
    for (unsigned int n=0; n < N; ++n) {
        Matrix x_hat = x.block(n,0,1,2);
        for (unsigned int m=0; m < M; ++m) {
            Matrix y_hat = y.block(m,0,1,2);
            y_hat = y_hat - x_hat;
            sigma2 += y_hat.squaredNorm();
        }
    }
    sigma2 /= (double) (D*N*M);

    // Iterate
    Matrix W(M,D);
    Matrix T = y;
    Matrix old_T = y;
    double T_resid = resid + 1.0;
    for (unsigned int i=0; i < iters; ++i) {
        if (T_resid < resid) {
            break;
        }

        // E-step : Compute P
        Matrix P(M,N);
        for (unsigned int n=0; n < N; ++n) {
            Matrix x_hat = x.block(n,0,1,2);
            Vector g(M);
            for (unsigned int m=0; m < M; ++m) {
                Matrix T_hat = T.block(m,0,1,2);
                T_hat = T_hat - x_hat;
                g(m) = exp( -1.0 / 2.0 * (T_hat.squaredNorm()/sigma2) );
            }
            P.block(0,n,M,1) = g / ( g.sum() + pow(2.0*PI*sigma2,D/2.0) * w / (1.0-w) * M / N );
        }

        // M-step : Solve for W, sigma2, update T
        Vector vec_dP1 = P * Vector::Constant(N,1.0);
        Vector vec_dPT1 = P.transpose() * Vector::Constant(M,1.0);
        auto dP1 = vec_dP1.asDiagonal();

        double N_p = P.sum();

        Matrix B = P*x - dP1*y;
        Matrix A = dP1*G + lambda*sigma2*Matrix::Identity(M,M);
        W = A.colPivHouseholderQr().solve(B);

        T = y + G*W;

        sigma2 = (1.0 / (N_p*D)) * ( (x.transpose() * vec_dPT1.asDiagonal() * x).trace() -
                2.0 * ((P*x).eval().transpose() * T).trace() + (T.transpose()*dP1*T).trace() );

        //sigma2 *= alpha*alpha;

        if (sigma2 <= 0.0) {
            break;
        }

        // Compute residual
        T_resid = (T - old_T).lpNorm<2>();
        old_T = T;
        std::cout << "Iter " << i << " | resid = " << T_resid << std::endl;

    }

    // Reverse normalization
    T.block(0,0,M,1) = T.block(0,0,M,1) * x_std[0] + x_mean[0]*Vector::Constant(M,1);
    T.block(0,1,M,1) = T.block(0,1,M,1) * x_std[1] + x_mean[1]*Vector::Constant(M,1);

    // Match points with closest points and reject outliers
    std::vector<cv::Point_<FP>> new_pts(M);
    for (unsigned int i=0; i < M; ++i) {
        new_pts[i] = cv::Point_<FP>((FP)T(i,0),(FP)T(i,1));
    }
    closest_point2(ref, new_pts, matches);

    unmatch_outliers(ref, new_pts, matches, 2.0);

    // Compute difference from reference
    float diff_norm = 0;
    for (unsigned int i=0; i < N; ++i) {
        if (matches[i] != -1) {
            diff_norm += pow(ref[i].x - new_pts[matches[i]].x,2) + 
                    pow(ref[i].y - new_pts[matches[i]].y,2);
        }
    }

    return sqrt(diff_norm);
}

/*****************************************************************************/
template<typename T>
int identify_targets(const cv::Mat& img, 
        const std::vector<cv::Point_<typename T::data_type>>& ref, 
        std::vector<T>& found, std::vector<int>& unmatched) {

    typedef typename T::data_type FP;

    unmatched.clear();

    // find kulites in the frame
    find_targets(img, found);
    std::vector<cv::Point_<FP>> found_pts(found.size());
    Kulite2Point2<FP> k2p;
    std::copy(boost::make_transform_iterator(found.begin(), k2p),
            boost::make_transform_iterator(found.end(), k2p), found_pts.begin());

    // match found kulites to reference kulites
    std::vector<int> matches;
    if ((found_pts.size() >= 4) && (ref.size() >= 4)) {
        closest_point(ref, found_pts, matches);
        //iterative_closest_point(ref, found_pts, matches, 10);
    } else {
        closest_point(ref, found_pts, matches);
        //closest_point2(ref, found_pts, matches);
    }
    assert(matches.size() == ref.size());

    // if any ref match to the same point, only let the closest ref keep that point
    // the other will be unfound
    double dist_i, dist_j;
    for (int i=0; i < ref.size(); ++i) {
        if (matches[i] < 0) {
            continue;
        }
        
        dist_i = cv::norm(ref[i] - found_pts[matches[i]]);
        for (int j=i+1; j < ref.size(); ++j) {
            if (matches[j] < 0) {
                continue;
            }

            if (matches[i] == matches[j]) {
                dist_j = cv::norm(ref[j] - found_pts[matches[j]]);
                if (dist_i < dist_j) {
                    matches[j] = -1;
                } else {
                    matches[i] = -1;
                    break;
                }
            }
        }
    }

    // assign numbers to found kulites, scrap unmatched, and record unfound
    for (int i=0; i < matches.size(); ++i) {
        if (matches[i] < 0) {
            unmatched.push_back(i);
        } else {
            found[matches[i]].num = i;
        }
    }

    std::sort(matches.begin(), matches.end());
    auto f_iter = found.begin();
    int count = 0;
    while (f_iter != found.end()) {
        if (std::binary_search(matches.begin(), matches.end(), count)) {
            ++f_iter;
        } else {
            f_iter = found.erase(f_iter);
        }
        ++count;
    }

    return found.size();
} 

/*****************************************************************************/
template<typename FP>
void create_roi(cv::Size sz, const std::vector<cv::Point_<FP>>& ref, 
        unsigned int pradius, std::vector<cv::Rect>& roi) {
    roi.resize(ref.size());
    cv::Point_<FP> offset((FP)pradius,(FP)pradius);
    cv::Point_<FP> pt1, pt2;
    for (int i=0; i < roi.size(); ++i) {
        // make sure the roi is in the frame
        pt1 = ref[i] - offset;
        pt1.x = std::max(pt1.x,(FP) 0.0);
        pt1.y = std::max(pt1.y,(FP) 0.0);
        pt2 = ref[i] + offset;
        pt2.x = std::min(pt2.x, (FP) (sz.width-1.0));
        pt2.y = std::min(pt2.y, (FP) (sz.height-1.0));

        roi[i] = cv::Rect(pt1, pt2);
    }
}

/*****************************************************************************/
template<typename T>
int identify_targets_local(const cv::Mat& img, 
        const std::vector<cv::Point_<typename T::data_type>>& ref, 
        std::vector<T>& found, std::vector<int>& unmatched, unsigned int prange) {

    unmatched.clear();
    found.clear();

    // Define regions around each reference to search
    std::vector<cv::Rect> roi(ref.size());
    create_roi(img.size(), ref, prange, roi);

    // find kulites in the frame
    std::vector<std::vector<T>> roi_found;
    find_targets_roi(img, roi, roi_found);

    // Choose the found target closest to the reference target
    int idx = -1;
    for (int i=0; i < roi_found.size(); ++i) {
        if (roi_found[i].size() > 0) {
            found.push_back(roi_found[i][0]);
            ++idx;
            found[idx].num = i;
        } else {
            unmatched.push_back(i);
        }   

        for (int j=1; j < roi_found[i].size(); ++j) {
            if (cv::norm(ref[i] - roi_found[i][j].uv) < cv::norm(ref[i] - found[idx].uv)) {
                found[idx] = roi_found[i][j];
                found[idx].num = i;
            }
        }
    }

    return found.size();
}

/*****************************************************************************/
/** Remove found targets that don't have a reference target
 * within radius pixels of it.
 */
template<typename T>
void filter_targets(const std::vector<T>& ref_targs, std::vector<T>& targs,
  float radius)
{
  unsigned targs_out = 0;
  for (unsigned t = 0; t < targs.size(); ++t) {
    for (unsigned rt = 0; rt < ref_targs.size(); ++rt) {
      if (cv::norm(targs[t].uv - ref_targs[rt].uv) < radius) {
        if (t != targs_out)
	  targs[targs_out] = targs[t];
	++targs_out;
	break;
      }
    }
  }
  targs.resize(targs_out);
}

/*****************************************************************************/
/** Remove targets that don't have a reference target within radius pixels
 * of it.  Then remove reference targets that don't have any targets within
 * the radius.
 */
template<typename T>
void bifilter_targets(std::vector<T>& ref_targs, std::vector<T>& targs,
  float radius)
{
  std::vector<bool> ref_has_targ(ref_targs.size(), false);

  unsigned targs_out = 0;
  for (unsigned t = 0; t < targs.size(); ++t) {
    for (unsigned rt = 0; rt < ref_targs.size(); ++rt) {
      if (cv::norm(targs[t].uv - ref_targs[rt].uv) < radius) {
	ref_has_targ[rt] = true;
        if (t != targs_out)
	  targs[targs_out] = targs[t];
	++targs_out;
	break;
      }
    }
  }
  targs.resize(targs_out);

  unsigned ref_targs_out = 0;
  for (unsigned rt = 0; rt < ref_targs.size(); ++rt) {
    if (ref_has_targ[rt]) {
      if (rt != ref_targs_out)
        ref_targs[ref_targs_out] = ref_targs[rt];
      ++ref_targs_out;
    }
  }
  ref_targs.resize(ref_targs_out);
}

} /* end namespace upsp */

