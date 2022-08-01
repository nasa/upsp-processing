#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "sample_data.h"

/*****************************************************************************/
upsp::TunnelConditions get_sample_tc() {

    upsp::TunnelConditions tc;
    tc.alpha = 0.0;
    tc.beta  = 0.0;
    tc.phi   = 0.0;
    tc.mach  = 0.85;
    tc.rey   = 3.0;
    tc.ptot  = 1500.0;
    tc.qbar  = 450.0;
    tc.ttot  = 65.0;
    tc.ps    = 900.0;
    
    tc.test_id = "sample file";
    tc.run = 1;
    tc.seq = 1;

    return tc;
}

/*****************************************************************************/
upsp::CameraSettings get_sample_cs() {

    upsp::CameraSettings cs;

    cs.framerate = 100;
    cs.fstop = 4;
    cs.exposure = 10000;
    cs.focal_lengths.assign(1, 21.99);
    cs.cam_nums.assign(1, 1);

    return cs;
}

