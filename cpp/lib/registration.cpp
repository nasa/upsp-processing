/** @file
 * 
 *  @brief  Image and Point Set Registration
 *  @date   August 18, 2017
 *  @author jmpowel2
 */

#define THRUST_DEVICE_SYSTEM THRUST_DEVICE_SYSTEM_CPP
#include "registration.h"

namespace upsp {

/********************************************************************
 * RegisterImage
********************************************************************/

/*****************************************************************************/
RegisterImage::RegisterImage(const cv::Mat& ref_img) : 
        ref_img_(ref_img), max_iters(50), epsilon(0.001) {}

/*****************************************************************************/
cv::Mat RegisterImage::operator()(cv::Mat inp) {
    return register_pixel(ref_img_, inp, wm_, max_iters, epsilon);
}


/********************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
cv::Mat register_pixel(const cv::Mat& ref_img, const cv::Mat& inp_img, 
        cv::Mat& warp_matrix, int max_iters/*=50*/, double epsilon/*=0.001*/) {

    // grayscale and ref CV_32F
    assert(ref_img.channels() == 1);
    assert(inp_img.channels() == 1);
    assert(ref_img.depth() == CV_32F);

    // convert input image to CV_32F, if needed
    cv::Mat inp;
    if (inp_img.depth() != CV_32F) {
        inp_img.convertTo(inp, CV_32F);
    } else {
        inp = inp_img;
    }

    // define a motion model
    const int warp_mode = cv::MOTION_AFFINE;

    // initialize the warp matrix to identity
    if (warp_mode == cv::MOTION_HOMOGRAPHY) {
        warp_matrix = cv::Mat::eye(3, 3, CV_32F);
    } else {
        warp_matrix = cv::Mat::eye(2, 3, CV_32F);
    }

    // specify termination criteria for algorithm
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 
            max_iters, epsilon);

    // run ECC algorithm
    cv::findTransformECC(ref_img, inp, warp_matrix, warp_mode, criteria);

    // get the warped image
    cv::Mat warp_img;
    if (warp_mode != cv::MOTION_HOMOGRAPHY) {
        cv::warpAffine(
            inp_img, warp_img, warp_matrix, ref_img.size(),
            cv::INTER_NEAREST | cv::WARP_INVERSE_MAP
        );
    } else {
        cv::warpPerspective(
            inp_img, warp_img, warp_matrix, ref_img.size(),
            cv::INTER_NEAREST | cv::WARP_INVERSE_MAP
        );
    }

    return warp_img;
}

} /* end namespace upsp */

