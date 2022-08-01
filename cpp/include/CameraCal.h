/** @file
 *  @brief  Camera Calibration
 *  @date   Sept 22, 2016
 *  @author jmpowel2
 */

#ifndef UFML_CAMERACAL_H_
#define UFML_CAMERACAL_H_

#include <iostream>
#include <vector>

#include "data_structs.h"

#include "utils/general_utils.h"

namespace upsp {

// Helper routine to populate a CameraCal object with
// parameters from a camera calibration JSON file.
//
// Example JSON file:
//
// {
//   "cameraMatrix": [
//     [666.1, 0.0, 539.1],
//     [0.0, 666.1, 261.0],
//     [0.0, 0.0, 1.0]
//   ],
//   "distCoeffs": [-0.043, 0.014, -0.000, 0.000, 0.0],
//   "rmat": [
//     [0.992, -0.001, 0.129],
//     [0.001, -0.999, -0.018],
//     [0.129, 0.018, -0.991]
//   ],
//   "tvec": [-77.1, 0.7, 63.3]
// }
//
void read_json_camera_calibration(upsp::CameraCal&, const std::string&);

class CameraCal {
public:

    /** Create empty camera calibration object */
    CameraCal();

    //virtual ~CameraCal() {}

    /** Loads in or computes the camera calibration properties and loads them into the object
     *
     * @param[in] camera_props_file camera properties input data file
     */
    void calibrate_camera(cv::String camera_props_file);

    /** Set the camera calibration parameters
     *
     * @param[in] cameraMatrix  intrinsic properties
     * @param[in] distCoeffs    distortion coefficients
     * @param[in] rvec          rotation angles
     * @param[in] tvec          translation vector (model units)
     * @param[in] sz            frame size 
     * @param[in] pix_sz        pixel size in mm
     */
    void calibrate_camera(cv::Mat cameraMatrix, cv::Mat distCoeffs, 
            cv::Mat rvec, cv::Mat tvec, cv::Size sz, double pix_sz = -1.0);

    /** Calibrates the camera using target xyz and uv information
     *  
     * @param[in] targs        vector of Target containing xyz and uv data
     * @param[in]  image_size   size of the camera frames
     * @param[in] cal_option    cv::calibrateCamera options
     */
    template<typename T>
    void calibrate_camera(std::vector<T> &targs, cv::Size image_size, int cal_option=0);

    /** Calibrate only the intrinsic properties of the camera
     * 
     * @param[in] targs        vector of Target containing xyz and uv data
     * @param[in] image_size   size of the camera frames
     */
    template<typename T>
    void calibrate_intrinsic(const std::vector<T> &targs, cv::Size image_size);

    /** Calibrate only the extrinsic properties of the camera
     * 
     * @param[in] targs        vector of Target containing xyz and uv data
     */
    template<typename T>
    void calibrate_extrinsic(const std::vector<T> &targs);

    /** Update the full camera calibration with new point matches
     *
     * @param[in] targs     vector of Target containing xyz and uv data
     * @param[in] cal_option    cv::calibrateCamera options
     *
     * @pre calibrated_ == true
     * @pre targs.size() > 4
     */
    template<typename T>
    void update_calibration(const std::vector<T> &targs, int cal_option=0);

    /** Compute the error in the camera calibration as the error between the found
     * target locations and the projected 3D coordinates
     *
     * @param[in] targs         vector of Target with found image plane locations and 3D coordinates
     * @param[out] reproj_err   vector of differences between found and projected
     * @return                  norm of the errors over all Targets
     */
    template<typename T>    
    typename T::data_type compute_reproj_errors(const std::vector<T> &targs, 
            std::vector<typename T::data_type> &reproj_err);

    /** Writes the objects camera properties to the given file.  
     * Requires that they have already been computed or loaded.
     *
     * @param[in] file_out file to write camera properties
     */
    void write_camera_properties(cv::String file_out) const;

    /** Return true if the full camera calibration is available */
    bool is_calibrated() const;

    /** Write the full camera calibration data to the screen
     */
    void display_all_properties() const;

    /** Maps the 3d targets to the 2d image plane
     *
     * @param[in,out] targs vector of T (Target or Kulite) with xyz data to project 
     *                       onto image plane (get back uv)
     */
    template<typename T>
    void map_points_to_image(std::vector<T> &targs) const;

    /** Maps the 3d targets to the 2d image plane
     * 
     * @param[in]     target      vector of 3d targets to project onto the image plane
     * @param[in,out] img_target  vector of 2d points in the image plane
     */
    template<typename FP>
    void map_points_to_image(const std::vector<cv::Point3_<FP>> &target, 
            std::vector<cv::Point_<FP>> &img_target) const;

    /** Maps a 3D point onto the 2D image plane
     * 
     * @param[in]   input   3D coordinate
     * @return              2D coordinate on the image plane
     */
    template<typename FP>
    cv::Point_<FP> map_point_to_image(const cv::Point3_<FP>& pt) const;

    /** Gets the coordinates of the camera center in 3D model space
     * 
     * @return  position of the camera center
     */
    cv::Point3d get_cam_center() const;

    /** Gets the ray from a pixel into 3D model space
     *
     * @param[in] pt    pixel 2D coordinate
     * @return          Ray into 3D space from pixel (origin in inches)
     *
     * @pre @a pt must be in the frame
     * @pre pix_sz_ must be defined and in mm
     */
    template<typename FP>
    upsp::Ray<FP> pixel_ray(cv::Point_<FP> pt) const;

    /** Plot the distortion across the frame
     *
     * @param[in] output    plot of distortion
     */
    void plot_distortion(cv::Mat& output) const;

    /** Return the size of the camera image */
    cv::Size size() const;

    /** Return the calibrated focal length in mm
     *
     * @pre pix_sz_ (pixel size in mm) is set
     * @pre focal length x == focal length y
     */
    float get_focal_length() const;

    template<typename Transform>
    void apply_extrinsic_transform(Transform& T);

protected:

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Mat rvec_;
    cv::Mat tvec_; // in inches
    double pix_sz_; // in mm
    double totalAvgErr_;
    cv::Size sz_;
    bool calibrated_;
    bool calibrated_int_;

private:

    /** Reads a camera properties file and loads the values into the object.
     *
     * @param[in] camera_prop_file  camera properties data file
     */
    void load_camera_properties(cv::String camera_prop_file);

};

} /* end namespace upsp */

#include "../lib/CameraCal.ipp"

#endif /* UFML_CAMERACAL_H_ */
