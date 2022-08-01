/** @file
 *  @brief  Basic Image Statistics and Writing
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename T>
void intensity_histc(const cv::Mat_<T>& img, std::vector<int>& edges, 
        std::vector<int>& counts, unsigned int depth /*= 12*/, 
        int bins /*= -1*/) {

    edges.clear();
    counts.clear();

    // Get depth of images
    unsigned int im_depth = 0;
    if (img.depth() == CV_8U) {
        im_depth = 8;
    } else if (img.depth() == CV_16U) {
        im_depth = 16;
    }
    if (depth > im_depth) {
        depth = im_depth;
    }

    // Initialize array
    unsigned int max_value = TWO_POW(depth);
    if (bins == -1) {
        bins = max_value;
    }
    T bin_sz = std::ceil(max_value / bins);
    counts.assign(bins, 0);

    // Fill array
    for (auto it=img.begin(); it != img.end(); ++it) {
        if (*it < max_value) {
            ++counts[std::floor(*it / bin_sz)];
        }
    }

    // Get edges
    edges.resize(bins+1);
    for (unsigned int i=0; i < edges.size(); ++i) {
        edges[i] = i*bin_sz;
    }

}

/*****************************************************************************/
template<typename T>
void intensity_histogram(std::string filename, const cv::Mat_<T>& img, 
        unsigned int depth /*=12*/, int bins /*= -1*/) {

    std::ofstream fs_out(filename);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write histogram data"));
    }

    // Get depth of images
    unsigned int im_depth = 0;
    if (img.depth() == CV_8U) {
        im_depth = 8;
    } else if (img.depth() == CV_16U) {
        im_depth = 16;
    }
    if (depth > im_depth) {
        depth = im_depth;
    }

    // Initialize array
    unsigned int max_value = TWO_POW(depth);
    T bin_sz = std::ceil(max_value / bins);
    std::vector<T> intensity(bins, 0);

    // Fill array
    for (auto it=img.begin(); it != img.end(); ++it) {
        if (*it < max_value) {
            ++intensity[std::floor(*it / bin_sz)];
        }
    }

    // Output results
    fs_out << "min_edge,intensity" << std::endl;
    for (int i=0; i < intensity.size(); ++i) {
        fs_out << (i*bin_sz) << "," << intensity[i] << std::endl;
    }

    fs_out.close();
}

/*****************************************************************************/
template<typename T>
void intensity_histogram(std::string filename, 
        const std::vector<cv::Mat_<T>>& imgs, unsigned int depth /*=12*/) {

    std::ofstream fs_out(filename);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write histogram data"));
    }

    // Get depth of images
    unsigned int im_depth = 0;
    if (imgs[0].depth() == CV_8U) {
        im_depth = 8;
    } else if (imgs[0].depth() == CV_16U) {
        im_depth = 16;
    }
    if (depth > im_depth) {
        depth = im_depth;
    }

    // Initialize array
    std::vector<T> intensity(TWO_POW(depth), 0);

    // Fill array
    for (int i=0; i < imgs.size(); ++i) {
        for (auto it=imgs[i].begin(); it != imgs[i].end(); ++it) {
            if (*it < intensity.size()) {
                ++intensity[*it];
            }
        }
    }

    // Output results
    fs_out << "Intensity Count" << std::endl;
    for (int i=0; i < intensity.size(); ++i) {
        fs_out << i << " " << intensity[i] << std::endl;
    }

    fs_out.close();
}

/*****************************************************************************/
template<typename InputIterator>
cv::Mat average_frames(InputIterator begin, InputIterator end) {

    if (begin == end) {
        return cv::Mat();
    }

    // Initialize output to zeros
    cv::Mat_<double> out = cv::Mat::zeros((*begin).rows, (*begin).cols, CV_64F);

    // Compute sum
    unsigned int count=0;
    for (auto it=begin; it != end; ++it) {
        out += *it;
        ++count;
    }

    return out / (double) count;
}

/*****************************************************************************/
template<typename T>
cv::Mat add_targets(const std::vector<T>& targs, const cv::Mat& src, 
        cv::Scalar color /*=cv::Scalar(0,255,0)*/, bool include_labels /*=true*/, 
        bool include_size /*=false*/) {

    typedef typename T::data_type FP;

    // Convert to 8-bit if needed
    cv::Mat out;
    if (src.depth() != CV_8U) {
        out = convert2_8U(src);
    } else {
        out = src.clone();
    }

    if (targs.size() == 0) {
        return out;
    }

    // Change image form grayscale to color
    if (out.channels() == 1) {
        cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
    }

    // set location of numbers
    cv::Point_<FP> shift(6,-6);

    // check if the targets are numbered
    bool numbered = false;
    for (int i=0; i < targs.size(); ++i) {
        if (targs[i].num != 0) {
            numbered = true;
            break;
        }
    }

    // Add targets to image
    int targ_num;
    for (int i=0; i < targs.size(); ++i) {
        if (numbered) {
            targ_num = targs[i].num;
        } else {
            targ_num = i+1;
        }

        if (include_size) {
            cv::circle(out, targs[i].uv, targs[i].diameter/2.0, color, -1, 8, 0);
        } else {
            cv::circle(out, targs[i].uv, 1, color, -1, 8, 0);
        }

        if (include_labels) {
            cv::putText(out, std::to_string(targ_num), targs[i].uv + shift, cv::FONT_HERSHEY_PLAIN,
                    1.0, color, 1);
        }
    }

    return out;
}

/*****************************************************************************/
template<typename T>
cv::Mat add_target_normals(const std::vector<T>& targs, const cv::Mat& src, 
        cv::Scalar color /*=cv::Scalar(0,255,0)*/, bool include_size /*=false*/,
        std::vector<cv::Point_<float>>& norms, float tip) {

    typedef typename T::data_type FP;

    // Convert to 8-bit if needed
    cv::Mat out;
    if (src.depth() != CV_8U) {
        out = convert2_8U(src);
    } else {
        out = src.clone();
    }

    if (targs.size() == 0) {
        return out;
    }

    // Change image form grayscale to color
    if (out.channels() == 1) {
        cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
    }

    // set location of numbers
    cv::Point_<FP> shift(6,-6);

    // check if the targets are numbered
    bool numbered = false;
    for (int i=0; i < targs.size(); ++i) {
        if (targs[i].num != 0) {
            numbered = true;
            break;
        }
    }

    // Add target normals to image
    int targ_num;
    for (int i=0; i < targs.size(); ++i) {
        if (numbered) {
            targ_num = targs[i].num;
        } else {
            targ_num = i+1;
        }

       try
       {
          cv::arrowedLine(out, targs[i].uv, norms[i], color, 1, 8, 0, tip);
          //cv::line(out, targs[i].uv, norms[i], color, 1, 8, 0);
        }
        catch (const std::exception& e1)
        {
            std::cout << e1.what() << std::endl;
        }
        if (include_size) {
          // change color to blue
            color=cv::Scalar(255,0,0);
            cv::circle(out, targs[i].uv, targs[i].diameter/2.0, color, 1, 8, 0);
        } else {
            cv::circle(out, targs[i].uv, 1, color, 1, 8, 0);
        }
        // back to red
        color=cv::Scalar(0,0,255);
    }

    return out;
}

/*****************************************************************************/
template<typename T>
cv::Mat add_targets(const T& targs, const cv::Mat& src, 
        cv::Scalar color /*=cv::Scalar(0,255,0)*/, bool include_labels /*=true*/, 
        bool include_size /*=false*/) {

    return add_targets(std::vector<T>(1,targs), src, color, include_labels, include_size);
}

/*****************************************************************************/
template<typename FP>
cv::Mat add_points(const std::vector<cv::Point_<FP>>& pts, const cv::Mat& src,
        cv::Scalar color /*=cv::Scalar(0,255,0)*/, bool include_labels /*=false*/) {

    std::vector<Target_<FP>> targs(pts.size());
    for (unsigned int i=0; i < pts.size(); ++i) {
        targs[i].uv = pts[i];
        targs[i].num = i+1;
    }
    return add_targets(targs, src, color, include_labels, false);
}

/*****************************************************************************/
template<typename T>
cv::Mat add_kulites(const std::vector<Kulite_<T>>& kuls, const cv::Mat& src) {

    // Convert to 8-bit if needed
    cv::Mat out;
    if (src.depth() != CV_8U) {
        out = convert2_8U(src);
    } else {
        out = src.clone();
    }

    if (kuls.size() == 0) {
        return out;
    }

    // Change image form grayscale to color
    if (out.channels() == 1) {
        cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
    }

    // set location of numbers
    cv::Point_<T> shift(kuls[0].size*2.0, -1.0*kuls[0].size*2.0);
    int vsize = 0;

    // Add kulites to image
    cv::Point2i virt;
    for (int i=0; i < kuls.size(); ++i) {
        vsize = kuls[i].size/2 - 1;
        if (vsize <=0 ) {
            vsize = 1;
        }

        cv::circle(out, kuls[i].uv, 1, cv::Scalar(0,255,0), -1, 8, 0);
        cv::putText(out, std::to_string(kuls[i].num), kuls[i].uv + shift, cv::FONT_HERSHEY_PLAIN,
                1.0, cv::Scalar(0,0,255), 1);

        virt = kuls[i].top_left;
        virt.x += kuls[i].size;
        virt.y += kuls[i].size;

        cv::rectangle(out, kuls[i].top_left, virt, cv::Scalar(255,0,0));
    }

    return out;

}

/*****************************************************************************/
template<typename T>
double fit_targets(const std::vector<T>& targs, cv::Size size, 
        std::vector<cv::Point2d>& pts) {

    // Extract 2D points (flip y since uv coordinate system is diff than xy)
    pts.resize(targs.size());
    for (int i=0; i < targs.size(); ++i) {
        pts[i] = cv::Point2d(targs[i].xyz.x, -targs[i].xyz.y);
    }

    // Check if diameters are provided, if so get the maximum diameter
    double max_diam = 0.0;
    for (int i=0; i < targs.size(); ++i) {
        if (targs[i].diameter > 0) {
            max_diam = std::max(max_diam, (double) targs[i].diameter);
        }
    }

    // Find the maximum extent in x and y of the targets
    BoundingBox2D bb = get_extent2D(pts.begin(), pts.end(), false);

    // If targets will not fit nicely in size, then scale target locations to fit
    double height = (bb.bounds[1].y - bb.bounds[0].y) + max_diam * 0.5;
    double width = (bb.bounds[1].x - bb.bounds[0].x) + max_diam * 0.5;
    double sf = std::min(size.height / (1.15*height), size.width / (1.15*width) );

    // Shift points so x,y >= 0 and centered with buffer
    for (int i=0; i < pts.size(); ++i) {
        pts[i].x = (pts[i].x - bb.bounds[0].x) * sf + 0.5*(size.width - width*sf);
        pts[i].y = (pts[i].y - bb.bounds[0].y) * sf + 0.5*(size.height - height*sf);
    }

    return sf;
}

/*****************************************************************************/
template<typename T>
cv::Mat plot_targets(const std::vector<T>& targs, cv::Size size, 
        bool add_label /*=false*/) {

    // Initialize white backgournd of requested size
    cv::Mat out(size.height, size.width, CV_8U, cv::Scalar::all(255));

    if (targs.size() == 0) {
        return out;
    }

    std::vector<cv::Point2d> pts;
    double sf = fit_targets(targs, size, pts);

    // Check if diameters are provided, if so get the maximum diameter
    bool has_diam = false;
    double max_diam = 0.0;
    for (int i=0; i < targs.size(); ++i) {
        if (targs[i].diameter > 0) {
            has_diam = true;
            max_diam = std::max(max_diam, (double) targs[i].diameter);
        }
    }

    // Check if nums are provided
    bool has_num = false;
    for (int i=0; i < targs.size(); ++i) {
        if (targs[i].num != 0) {
            has_num = true;
            break;
        }
    }

    // Create the shift for labels
    cv::Point2d shift(6,-6);
    if (has_diam) {
        if (max_diam > 10) {
            shift.x = 0.5 * max_diam + 2.0;
            shift.y = -0.5 * max_diam - 2.0;
        }
    }

    // Plot the targets on the Mat
    int default_radius = std::ceil(std::min(size.height, size.width) * 0.01);
    int radius, label;
    for (int i=0; i < targs.size(); ++i) {
        if (has_diam) {
            radius = std::ceil(targs[i].diameter * 0.5 * sf);
        } else {
            radius = default_radius;
        }
        
        cv::circle(out, pts[i], radius, cv::Scalar(0), -1);
        if (add_label) {
            if (has_num) {
                label = targs[i].num;
            } else {
                label = i + 1;
            }
            cv::putText(out, std::to_string(label), pts[i] + shift, cv::FONT_HERSHEY_PLAIN,
                    1.0, cv::Scalar(0), 1);
        }
    }

    return out; 

}

} /* end namespace upsp */

