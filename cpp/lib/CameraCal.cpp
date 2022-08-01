/*
 * CameraCal.cpp
 *
 *  Created on: Sept 22, 2016
 *      Author: jmpowel2
 */

#include <boost/tokenizer.hpp>
#include <vector>
#include <fstream>
#include <sstream>

#include "json.hpp"
#include "CameraCal.h"
#include "utils/general_utils.h"

namespace upsp {

void read_json_camera_calibration(upsp::CameraCal& cal, const std::string& cfg_file) {
  nlohmann::json j;
  {
    std::ifstream ifs(cfg_file);
    ifs >> j;
  }

  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  // todo-mshawlec handle multiple different sizes for distortion coefficients
  cv::Mat distortion_coefficients = cv::Mat::zeros(1, 4, CV_64F);
  cv::Mat rmat = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  // todo-mshawlec populate pixel size
  // double pix_sz;

  auto read_mat = [](cv::Mat& m, nlohmann::json& o, int nr, int nc) {
    for (int ii = 0; ii < nr; ii++) {
      for (int jj = 0; jj < nc; jj++) {
        if (nc == 1) m.at<double>(ii, jj) = o[ii];
        else if (nr == 1) m.at<double>(ii, jj) = o[jj];
        else m.at<double>(ii, jj) = o[ii][jj];
      }
    }
  };

  read_mat(camera_matrix, j["cameraMatrix"], 3, 3);
  read_mat(distortion_coefficients, j["distCoeffs"], 1, 4);
  read_mat(rmat, j["rmat"], 3, 3);
  read_mat(tvec, j["tvec"], 3, 1);
  cv::Size sz(j["imageSize"][0], j["imageSize"][1]);

  cv::Rodrigues(rmat, rvec);
  cal.calibrate_camera(camera_matrix, distortion_coefficients, rvec, tvec, sz);
}

/*****************************************************************************/
CameraCal::CameraCal(): calibrated_(false), calibrated_int_(false), pix_sz_(-1.0) {}

/*****************************************************************************/
void CameraCal::calibrate_camera(cv::String camera_props_file) {

    load_camera_properties(camera_props_file);
    calibrated_ = true; // camera is calibrated
    calibrated_int_ = true;
}

/*****************************************************************************/
void CameraCal::calibrate_camera(cv::Mat cameraMatrix, cv::Mat distCoeffs, 
        cv::Mat rvec, cv::Mat tvec, cv::Size sz, double pix_sz) {

    if ( (cameraMatrix.rows != 3) || (cameraMatrix.cols != 3) ) {
        throw(std::invalid_argument("cameraMatrix should be 3x3 matrix"));
    }
    cameraMatrix_ = cameraMatrix.clone();

    if ( ( distCoeffs.rows != 1) || ( (distCoeffs.cols != 4) && (distCoeffs.cols != 5) && 
            (distCoeffs.cols != 8) ) ) {
        throw(std::invalid_argument("distCoeffs should be a row vector with 4,5,or 8 coefficients"));
    }
    distCoeffs_ = distCoeffs.clone();
        

    if ( (rvec.rows != 3) || (rvec.cols != 1) ) {
        throw(std::invalid_argument("rvec should be 3x1 matrix"));
    }
    rvec_ = rvec.clone();

    if ( (tvec.rows != 3) || (tvec.cols != 1) ) {
        throw(std::invalid_argument("tvec should be 3x1 matrix"));
    }
    tvec_ = tvec.clone();

    sz_ = sz;
    pix_sz_ = pix_sz;

    calibrated_ = true;
    calibrated_int_ = true;
}

/*****************************************************************************/
void CameraCal::write_camera_properties(cv::String file_out) const {

    // Check that camera properties are loaded
    assert(is_calibrated());

    std::ofstream fs_out(file_out);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write camera properties"));
    }
    fs_out.precision(8);

    // Write the Camera Matrix
    fs_out << "# cameraMatrix" << std::endl;
    for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++) {
            fs_out << cameraMatrix_.at<double>(i,j);
            if (j != 2) {
                fs_out << ", ";
            } else {
                fs_out << std::endl;
            }
        }
    }

    // Write the Distortion Coefficients
    fs_out << "# distCoeffs" << std::endl;
    for (int j=0; j < distCoeffs_.size[1] ; j++) {
        fs_out << distCoeffs_.at<double>(0,j) ;
        if (j!= (distCoeffs_.size[1]-1)) {
            fs_out << ", ";
        } else {
            fs_out << std::endl;
        }
    }

    // Write out the Rotation Vector
    fs_out << "# rvec" << std::endl;
    for (int j=0; j < rvec_.size[0]; j++) {
        fs_out << rvec_.at<double>(j,0);
        if (j != 2) {
            fs_out << ", ";
        } else {
            fs_out << std::endl;
        }
    }

    // Write out the Translation Vector
    fs_out << "# tvec" << std::endl;
    for (int j=0; j < tvec_.size[0]; j++) {
        fs_out << tvec_.at<double>(j,0);
        if (j != 2) {
            fs_out << ", ";
        } else {
            fs_out << std::endl;
        }
    }

    // Write out the size of the image
    fs_out << "# image_size" << std::endl;
    fs_out << sz_.width << ", " << sz_.height << std::endl;

    // Write out the pixel size
    fs_out << "# pixel_size" << std::endl;
    fs_out << pix_sz_ << std::endl;

    fs_out.close();
}
    

/*****************************************************************************/
bool CameraCal::is_calibrated() const {return calibrated_;}

/*****************************************************************************/
void CameraCal::display_all_properties() const {

    assert(calibrated_);

    std::cout << std::endl;
    std::cout << "Camera Matrix:" << std::endl;
    std::cout << cameraMatrix_ << std::endl << std::endl;
    
    std::cout << "Distortion Coefficients:" << std::endl;
    std::cout << distCoeffs_ << std::endl << std::endl;

    std::cout << "Rotation Vector:" << std::endl;
    std::cout << rvec_ << std::endl << std::endl;

    std::cout << "Translation Vector:" << std::endl;
    std::cout << tvec_ << std::endl << std::endl;

}

/*****************************************************************************/
cv::Point3d CameraCal::get_cam_center() const {
    cv::Mat_<double> center;
    cv::Mat_<double> rot, rotT;

    cv::Rodrigues(rvec_, rot);

    cv::transpose(rot, rotT);
    center = -rotT * tvec_;

    return {center(0,0), center(1,0), center(2,0)};

}

/*****************************************************************************/
void CameraCal::plot_distortion(cv::Mat& output) const {

    assert(calibrated_);

    // Generate mapping between undistorted and distorted points
    cv::Mat newCameraMatrix, map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), newCameraMatrix,
            size(), CV_32FC1, map1, map2);

    // Generate output with values of distance between undistorted and distorted points
    // in pixels
    output = cv::Mat::zeros(size(), CV_32F);

    float *p_out, *p_map1, *p_map2;
    cv::Point2f p1, p2;
    for (int i=0; i < output.rows; ++i) {
        p_out = output.ptr<float>(i);
        p_map1 = map1.ptr<float>(i);
        p_map2 = map2.ptr<float>(i);
        for (int j=0; j < output.cols; ++j, ++p_out, ++p_map1, ++p_map2) {
            p1 = cv::Point2f(i,j);
            p2 = cv::Point2f(*p_map2, *p_map1);
            *p_out = cv::norm(p1 - p2);
        }
    }

}


/*****************************************************************************/
cv::Size CameraCal::size() const {return sz_;}

/*****************************************************************************/
float CameraCal::get_focal_length() const {
    if (!calibrated_int_) {
        std::cerr << "Cannot return focal length before calibrating";
        std::cerr << " the camera intrinsics" << std::endl;
        std::abort();
    }
    switch(cameraMatrix_.type()) {
        case CV_32FC1: return cameraMatrix_.at<float>(0,0) * pix_sz_;
        case CV_64FC1: return cameraMatrix_.at<double>(0,0) * pix_sz_;
    }
    assert(false);
    return -1;
}

/*****************************************************************************/
void CameraCal::load_camera_properties(cv::String camera_prop_file) {

    std::ifstream ifs_in(camera_prop_file);
    if (!ifs_in) {
        throw(std::invalid_argument("Cannot open camera properties file"));
    }

    // Initialize Camera Position, Orientation, and Properties
    cameraMatrix_ = cv::Mat::zeros(3,3,CV_64F);
    distCoeffs_.release();  // Can be 4, 5, or 8 elements
    rvec_ = cv::Mat::zeros(3,1,CV_64F);
    tvec_ = cv::Mat::zeros(3,1,CV_64F);

    std::string line;
    std::vector<std::string> vec;
    typedef boost::tokenizer<boost::escaped_list_separator<char>> tokenizer;
    while (getline(ifs_in,line)) {

        if (line[0] == '#') {
            if (line.find("cameraMatrix") != std::string::npos) {
                for (int i=0; i < 3; i++) {
                    if (getline(ifs_in,line)) {
                        tokenizer tok(line);
                        vec.assign(tok.begin(), tok.end());
                        if (vec.size() != 3) {
                            throw(std::invalid_argument("Camera Matrix is invalid"));
                        }
                        for (int j=0; j < 3; j++) {
                            cameraMatrix_.at<double>(i,j) = std::stod(vec[j]);
                        }
                    } else {
                        throw(std::invalid_argument("Camera properties file has incorrect format"));
                    }
                }
            } else if (line.find("distCoeffs") != std::string::npos) {
                if (getline(ifs_in,line)) {
                    tokenizer tok(line);
                    vec.assign(tok.begin(), tok.end());
                    if ((vec.size() != 4) && (vec.size() != 5) && (vec.size() != 8)) {
                        throw(std::invalid_argument("Distortion Coefficients list is invalid"));
                    }
                    distCoeffs_ = cv::Mat::zeros(1,vec.size(),CV_64F);
                    for (int j=0; j < vec.size(); j++) {
                        distCoeffs_.at<double>(0,j) = std::stod(vec[j]);
                    }
                } else {
                    throw(std::invalid_argument("Camera properties file has incorrect format"));
                }
            } else if (line.find("rvec") != std::string::npos) {
                if (getline(ifs_in,line)) {
                    tokenizer tok(line);
                    vec.assign(tok.begin(), tok.end());
                    if (vec.size() != 3) {
                        throw(std::invalid_argument("Rotation Vector is invalid"));
                    }
                    for (int i=0; i < 3; i++) {
                        rvec_.at<double>(i,0) = std::stod(vec[i]);
                    }
                } else {
                    throw(std::invalid_argument("Camera properties file has incorrect format"));
                }
            } else if (line.find("tvec") != std::string::npos) {
                if (getline(ifs_in,line)) {
                    tokenizer tok(line);
                    vec.assign(tok.begin(), tok.end());
                    if (vec.size() != 3) {
                        throw(std::invalid_argument("Translation Vector is invalid"));
                    }
                    for (int i=0; i < 3; i++) {
                        tvec_.at<double>(i,0) = std::stod(vec[i]);
                    }
                } else {
                    throw(std::invalid_argument("Camera properties file has incorrect format"));
                }
            } else if (line.find("image_size") != std::string::npos) {
                if (getline(ifs_in, line)) {
                    tokenizer tok(line);
                    vec.assign(tok.begin(), tok.end());
                    if (vec.size() != 2) {
                        throw(std::invalid_argument("Image Size is invalid"));
                    }
                    sz_ = cv::Size(std::stod(vec[0]), std::stod(vec[1]));
                }
            } else if (line.find("pixel_size") != std::string::npos) {
                ifs_in >> pix_sz_;
            }
        }
    }

    ifs_in.close();

}

} /* end namespace upsp */
