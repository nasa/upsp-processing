/** @file
 *  @brief  Simple Data Structure Definitions (no openCV)
 *  @date   May 11, 2018
 *  @author jmpowel2
 */

#ifndef UFML_NON_CV_UPSP_H_
#define UFML_NON_CV_UPSP_H_

#include <fstream>
#include <iostream>

#include "utils/general_utils.h"

namespace upsp {

/** Load the paint calibration and compute gain */
struct PaintCalibration {

    /** Load the paint calibration file */
    PaintCalibration(std::string filename);

    /** Compute the gain 
     *
     * @param[in] T     wall temperature (degF)
     * @param[in] Pss   steady state PSP pressure (psf)
     */
    float get_gain(float T, float Pss);

    /***************************************************************/

    float a;
    float b;
    float c;
    float d;
    float e;
    float f;

};

/** Add the paint calibration formula to the stream */
std::ostream& operator<<(std::ostream& os, const PaintCalibration& pcal);

/** Manage the Wind Tunnel Test Conditions */
struct TunnelConditions : equality_comparable<TunnelConditions> {

    /** Create object with invalid values */
    TunnelConditions();

    /** Compare the test conditions for equality */
    bool operator==(const TunnelConditions& tc);


    /***************************************************************/

    float alpha;    // deg
    float beta;     // deg
    float phi;      // deg
    float mach;
    float rey;      // million/ft
    float ptot;     // psf
    float qbar;     // psf
    float ttot;     // degF
    float ps;       // psf
    float tcavg;    // degF

    std::string test_id;
    int run;
    int seq;
};

/** Read the wind tunnel test conditions from file */
upsp::TunnelConditions read_tunnel_conditions(const std::string& filename);

/** Manage the camera settings */
struct CameraSettings {

    /** Create object with invalid values */
    CameraSettings();

    /** Compare the camera settings for equality */
    bool operator==(const CameraSettings& cs);

    /***************************************************************/

    int framerate;  // Hz
    float fstop;    
    float exposure; // microseconds
    std::vector<float> focal_lengths;
    std::vector<int> cam_nums;
};

} /* end namespace upsp */

#endif /* UFML_NON_CV_UPSP_H_ */
