#ifndef MRAWREADER_H_
#define MRAWREADER_H_

#include <iostream>
#include <string>
#include <opencv2/core.hpp>

#include "PSPVideo.h"

namespace upsp {

/** Photron MRAW video reader */
class MrawReader: public VideoReader {
public:
    MrawReader(const std::string& mraw_file);
    virtual ~MrawReader();
    void load_properties(VideoProperties &props) override;
    cv::Mat read_frame(unsigned int n) override;

private:
    std::ifstream ifs_;
    std::string cih_file_;
    VideoProperties *properties_;

    bool check_first_line_(const std::string& line);
};

}

#endif
