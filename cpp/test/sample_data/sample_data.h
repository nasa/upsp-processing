#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "upsp.h"

/** Create sample tunnel conditions */
upsp::TunnelConditions get_sample_tc();

/** Create sample camera settings */
upsp::CameraSettings get_sample_cs();

/** Create untransposed data */
template<typename Model>
void create_data(const Model& model, bool nodal, unsigned int n_frames,
        std::vector<float>& sol);

#include "sample_data.ipp"
