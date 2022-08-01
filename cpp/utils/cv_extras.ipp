/** @file
 *
 *  @brief OpenCV Extras
 */

namespace upsp {

/*****************************************************************************/
template<typename T>
cv::Point3_<T> ceil(cv::Point3_<T>& pt) {
    cv::Point3_<T> out(pt);
    out.x = std::ceil(out.x);
    out.y = std::ceil(out.y);
    out.z = std::ceil(out.z);
    return out;
}

/*****************************************************************************/
template<typename T>
cv::Point3_<T> floor(cv::Point3_<T>& pt) {
    cv::Point3_<T> out(pt);
    out.x = std::floor(out.x);
    out.y = std::floor(out.y);
    out.z = std::floor(out.z);
    return out;
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> get_perpendicular(cv::Point3_<FP> vec) {

    cv::Point3_<FP> out(0.0,0.0,0.0);

    // normalize vec
    FP norm = cv::norm(vec);
    if (norm == 0.0) {
        return out;
    }
    vec = vec / norm;

    // pick 2 vector components, get third
    unsigned int max_comp = max_ind(std::abs(vec.x),std::abs(vec.y),std::abs(vec.z));
    switch (max_comp) {
        case 0 :
            out.y = 1.0;
            out.z = 0.0;
            out.x = -(out.y*vec.y + out.z*vec.z)/vec.x;
            break;
        case 1 :
            out.x = 1.0;
            out.z = 0.0;
            out.y = -(out.x*vec.x + out.z*vec.z)/vec.y;
            break;
        case 2 :    
            out.x = 1.0;
            out.y = 0.0;
            out.z = -(out.x*vec.x + out.y*vec.y)/vec.z;
            break;
        default :
            return out;
            break;
    }

    return out / cv::norm(out);
}

/*****************************************************************************/
template<typename T>
double angle_between(const cv::Point3_<T>& v1, const cv::Point3_<T>& v2) {
    double ang = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z)/ cv::norm(v1) / 
            cv::norm(v2);
    return acos(ang);
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> rotate(const cv::Point3_<FP>& axis_dir, 
        const cv::Point3_<FP>& axis_pt, FP angle, const cv::Point3_<FP>& pt) {
    
    // form the rotation matrix, theta = norm(r)
    cv::Mat_<FP> r(3,1);
    r(0) = axis_dir.x;
    r(1) = axis_dir.y;
    r(2) = axis_dir.z;
    r = r / cv::norm(r) * angle;
    cv::Mat_<FP> mat;
    cv::Rodrigues(r, mat);

    // translate point by axis_pt
    cv::Mat_<FP> pt_i(3,1);
    pt_i(0) = pt.x - axis_pt.x;
    pt_i(1) = pt.y - axis_pt.y;
    pt_i(2) = pt.z - axis_pt.z;
    
    // perform rotation
    cv::Mat_<FP> pt_o(3,1);
    pt_o = mat * pt_i;

    // translate back by axis_pt
    return {pt_o(0)+axis_pt.x, pt_o(1)+axis_pt.y, pt_o(2)+axis_pt.z};
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> cross(const cv::Point3_<FP>& pt1, const cv::Point3_<FP>& pt2) {

    cv::Point3_<FP> out;
    out.x =  (pt1.y * pt2.z) - (pt1.y * pt2.x);
    out.y = -(pt1.x * pt2.z) + (pt1.z * pt2.x);
    out.z =  (pt1.x * pt2.y) - (pt1.y * pt2.x);

    return out;
}

/*****************************************************************************/
template<typename T>
void parse_point3(std::string str, cv::Point3_<T>& pt) {

    // strip whitespace
    auto f_isspace = [](unsigned char const c) { return std::isspace(c);};
    str.erase(std::remove_if(str.begin(), str.end(), f_isspace), str.end());

    // split on comma
    std::vector<std::string> terms;
    split_string(str, ',', terms);

    // convert to int or floating point
    std::stringstream ss;
    ss.str(terms[0]);
    ss >> pt.x;
    ss.str(terms[1]);
    ss >> pt.y;
    ss.str(terms[2]);
    ss >> pt.z;
}

/*****************************************************************************/
template<typename T>
void parse_point2(std::string str, cv::Point_<T>& pt) {

    // strip whitespace
    auto f_isspace = [](unsigned char const c) { return std::isspace(c);};
    str.erase(std::remove_if(str.begin(), str.end(), f_isspace), str.end());

    // split on comma
    std::vector<std::string> terms;
    split_string(str, ',', terms);

    // convert to int or floating point
    std::stringstream ss;
    ss.str(terms[0]);
    ss >> pt.x;
    ss.str(terms[1]);
    ss >> pt.y;
}

} /* end namespace upsp */

