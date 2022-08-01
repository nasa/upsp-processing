/** @file
 *  @brief  Camera Calibration
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */
#include <opencv2/calib3d.hpp>

namespace upsp {

/*****************************************************************************/

// Update camera extrinsics to compose previous extrinsics with another
// transform. The transform is pre-applied before the old extrinsic transform.
template<typename Transform>
void CameraCal::apply_extrinsic_transform(Transform& t) {

    const auto GetAffineFromExtrinsics = [this]() {
        cv::Vec3d t(
            tvec_.at<double>(0, 0),
            tvec_.at<double>(1, 0),
            tvec_.at<double>(2, 0)
        );
        cv::Vec3d r(
            rvec_.at<double>(0, 0),
            rvec_.at<double>(1, 0),
            rvec_.at<double>(2, 0)
        );
        return cv::Affine3d(r, t);
    };

    const auto SetExtrinsicsFromAffine = [this](const cv::Affine3d& aff) {
        cv::Vec3d t = aff.translation();
        cv::Vec3d r = aff.rvec();
        tvec_.at<double>(0, 0) = t(0);
        tvec_.at<double>(1, 0) = t(1);
        tvec_.at<double>(2, 0) = t(2);
        rvec_.at<double>(0, 0) = r(0);
        rvec_.at<double>(1, 0) = r(1);
        rvec_.at<double>(2, 0) = r(2);
    };

    // Get affine matrix from transform by applying it to basis vectors
    cv::Point3d px0(0, 0, 0);
    cv::Point3d px1(1, 0, 0);
    cv::Point3d px2(0, 1, 0);
    cv::Point3d px3(0, 0, 1);
    cv::Point3d py0 = t(px0);
    cv::Point3d py1 = t(px1);
    cv::Point3d py2 = t(px2);
    cv::Point3d py3 = t(px3);
    cv::Point3d ey1 = py1 - py0;
    cv::Point3d ey2 = py2 - py0;
    cv::Point3d ey3 = py3 - py0;
    cv::Matx<double, 4, 4> MA1(
        ey1.x, ey2.x, ey3.x, py0.x,
        ey1.y, ey2.y, ey3.y, py0.y,
        ey1.z, ey2.z, ey3.z, py0.z,
        0, 0, 0, 1
    );
    cv::Affine3d A1(MA1);

    const auto A2 = GetAffineFromExtrinsics();

    cv::Affine3d A;
    A = A1.concatenate(A2);
    SetExtrinsicsFromAffine(A);
}

/*****************************************************************************/
template<typename T>
void CameraCal::calibrate_camera(std::vector<T> &targs, cv::Size image_size, 
        int cal_option/*=0*/) {

    typedef typename T::data_type FP;

    sz_ = image_size;

    std::vector<std::vector<cv::Vec<FP,3>>> objectPoints(1,std::vector<cv::Vec<FP,3>>(targs.size()));
    std::vector<std::vector<cv::Vec<FP,2>>> imagePoints(1 ,std::vector<cv::Vec<FP,2>>(targs.size()));

    for (int i=0; i < targs.size(); i++) {
        objectPoints[0][i] = targs[i].xyz;
        imagePoints[0][i] = targs[i].uv;
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    totalAvgErr_ = cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix_, 
        distCoeffs_, rvecs, tvecs, cal_option);

    rvec_ = rvecs[0];
    tvec_ = tvecs[0];

    // Mark that the camera has been calibrated
    calibrated_ = true;

}

/*****************************************************************************/
template<typename T>
void CameraCal::calibrate_intrinsic(const std::vector<T> &targs, cv::Size image_size) {

    typedef typename T::data_type FP;

    sz_ = image_size;

    std::vector<std::vector<cv::Vec<FP,3>>> objectPoints(1,std::vector<cv::Vec<FP,3>>(targs.size()));
    std::vector<std::vector<cv::Vec<FP,2>>> imagePoints(1 ,std::vector<cv::Vec<FP,2>>(targs.size()));

    for (int i=0; i < targs.size(); i++) {
        objectPoints[0][i] = targs[i].xyz;
        imagePoints[0][i] = targs[i].uv;
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    totalAvgErr_ = cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix_, 
        distCoeffs_, rvecs, tvecs); // CV_CALIB_ZERO_TANGENT_DIST);

    std::cout << distCoeffs_ << std::endl;

    calibrated_int_ = true;

}

/*****************************************************************************/
template<typename T>
void CameraCal::calibrate_extrinsic(const std::vector<T> &targs) {

    typedef typename T::data_type FP;

    assert(calibrated_int_ || calibrated_);

    std::vector<cv::Point3_<FP>> objectPoints(targs.size());
    std::vector<cv::Point_<FP>> imagePoints(targs.size());

    // NEXT REV: try doubling up the number of points using the size of the targets

    for (int i=0; i < targs.size(); i++) {
        objectPoints[i] = targs[i].xyz;
        imagePoints[i] = targs[i].uv;
    }

    cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec_, tvec_); 

    calibrated_ = true;
}

/*****************************************************************************/
template<typename T>
void CameraCal::update_calibration(const std::vector<T> &targs, int cal_option/*=0*/) {

    typedef typename T::data_type FP;

    assert(calibrated_int_ || calibrated_);

    std::vector<std::vector<cv::Vec<FP,3>>> objectPoints(1,std::vector<cv::Vec<FP,3>>(targs.size()));
    std::vector<std::vector<cv::Vec<FP,2>>> imagePoints(1 ,std::vector<cv::Vec<FP,2>>(targs.size()));

    for (int i=0; i < targs.size(); i++) {
        objectPoints[0][i] = targs[i].xyz;
        imagePoints[0][i] = targs[i].uv;
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    std::cout << "camera size = " << sz_ << std::endl;
    std::cout << "starting calibration with " << targs.size() << " targets" << std::endl; 
    cv::calibrateCamera(objectPoints, imagePoints, sz_, cameraMatrix_, 
        distCoeffs_, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS | cal_option);
    std::cout << "finished updated calibration" << std::endl;

    rvec_ = rvecs[0];
    tvec_ = tvecs[0];

}

/*****************************************************************************/
template<typename T>
typename T::data_type CameraCal::compute_reproj_errors(const std::vector<T> &targs, std::vector<typename T::data_type> &reproj_err) {

    assert(calibrated_);

    // Project points using calibration
    std::vector<T> proj_targs(targs);
    map_points_to_image(proj_targs);

    // Compute errors
    reproj_err.resize(targs.size());
    typename T::data_type total_err = 0.0;
    for (int i=0; i < targs.size(); ++i) {
        reproj_err[i] = norm(targs[i].uv - proj_targs[i].uv);
        total_err += reproj_err[i]*reproj_err[i];
    }
    total_err = sqrt(total_err);

    return total_err;
}

/*****************************************************************************/
template<typename T>
void CameraCal::map_points_to_image(std::vector<T> &targs) const {

    typedef typename T::data_type FP;

    // Check that type is valid (not really necessary, since I only list 2 types at the bottom) 
    static_assert(std::is_base_of<Target_<FP>, T>::value, "T not derived from Target class");

    std::vector<cv::Point3_<FP>> target;
    std::vector<cv::Point_<FP>> img_target;

    for (int i=0; i < targs.size(); ++i) {
        target.push_back(targs[i].xyz);
    }

    map_points_to_image(target, img_target);

    for (int i=0; i < targs.size(); ++i) {
        targs[i].uv = img_target[i];
    }
}

/*****************************************************************************/
template<typename FP>
void CameraCal::map_points_to_image(const std::vector<cv::Point3_<FP>> &target, std::vector<cv::Point_<FP>> &img_target) const {
    cv::projectPoints(target, rvec_, tvec_, cameraMatrix_, distCoeffs_, img_target);
}

/*****************************************************************************/
template<typename FP>
cv::Point_<FP> CameraCal::map_point_to_image(const cv::Point3_<FP>& pt) const {
    std::vector<cv::Point3_<FP>> input(1,pt);
    std::vector<cv::Point_<FP>> output;
    map_points_to_image(input, output);
    return output[0];
}

/*****************************************************************************/
template<typename FP>
upsp::Ray<FP> CameraCal::pixel_ray(cv::Point_<FP> pt) const {
    assert((pt.x >= 0.0) && (pt.x < sz_.width));
    assert((pt.y >= 0.0) && (pt.y < sz_.height));
    assert(pix_sz_ >= 0.0);

    // get undistorted pixel location
    std::vector<cv::Point_<FP>> input(1,pt);
    std::vector<cv::Point_<FP>> output;
    undistortPoints(input, output, cameraMatrix_, distCoeffs_, cv::noArray(), 
            cameraMatrix_);

    // The z coordinate is equal to the focal length
    FP fx = (FP) cameraMatrix_.at<double>(0,0);
    FP fy = (FP) cameraMatrix_.at<double>(1,1);
    FP f = pix_sz_ * 0.5 * (fx + fy) * MM_TO_INCHES;

    // Define pixel in camera coord. 
    cv::Mat_<FP> dir(3,1);
    FP z = f;
    dir.at(0,0) = (output[0].x - (FP) cameraMatrix_.at<double>(0,2)) * z / fx;
    dir.at(1,0) = (output[0].y - (FP) cameraMatrix_.at<double>(1,2)) * z / fy;
    dir.at(2,0) = z;
    cv::Mat_<FP> origin = dir.clone(); // deep copy

    // Rotate and translate location into model coord.
    cv::Mat_<FP> rot;
    cv::Rodrigues(rvec_, rot);
    dir = rot.inv() * dir;
    dir /= cv::norm(dir);
    origin -= tvec_;
    origin = rot.inv() * origin;

    // Convert to points
    cv::Point3_<FP> p_origin;
    p_origin.x = origin.at(0,0);
    p_origin.y = origin.at(1,0);
    p_origin.z = origin.at(2,0);
    cv::Point3_<FP> p_dir;
    p_dir.x = dir.at(0,0);
    p_dir.y = dir.at(1,0);
    p_dir.z = dir.at(2,0);
    
    return {p_origin, p_dir};
}

} /* end namespace upsp */
