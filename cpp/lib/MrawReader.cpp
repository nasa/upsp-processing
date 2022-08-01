#include "MrawReader.h"

#include <vector>
#include <exception>
#include <stdexcept>
#include <cstdint>
#include <algorithm>
#include <cctype>
#include <functional>
#include <sstream>
#include <iostream>
#include <regex>
#include <map>
#include <opencv2/core.hpp>

// trim from start
static inline std::string& ltrim(std::string& s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

// trim from end
static inline std::string& rtrim(std::string& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::not1(std::ptr_fun<int, int>(std::isspace)))
              .base(),
          s.end());
  return s;
}

// trim from both ends
static inline std::string& trim(std::string& s) { return ltrim(rtrim(s)); }

// Tokenize based on delimiter supplied as a regex
static std::vector<std::string> split(const std::string& input,
                                      const std::string& regex) {
  // passing -1 as the submatch index parameter performs splitting
  std::regex re(regex);
  std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
  return {first, last};
}

namespace upsp {

MrawReader::MrawReader(const std::string& mraw_file)
    : ifs_(mraw_file, std::ios::binary) {
    if (!ifs_) {
        throw std::invalid_argument("Video File is invalid");
    }

    cih_file_ = mraw_file.substr(0, mraw_file.rfind('.')) + ".cih";
}

MrawReader::~MrawReader() {
    if (ifs_.is_open()) {
        ifs_.close();
    }
}

void MrawReader::load_properties(VideoProperties &props) {
    std::ifstream input;
    std::vector<std::string> lines;
    std::map<std::string, std::string> tokens;
    const std::string TOKEN_DELIMITER("\\s:\\s");

    input.open(cih_file_);

    std::string line;
    while (std::getline(input, line)) {
        // MRAW header (cih) files might have carriage returns
        // ... robust trim before parsing
        line = trim(line);
        if (lines.empty() && !check_first_line_(line)) {
            // TODO decide what to do here
        }
        lines.push_back(line);
        const auto toks = split(line, TOKEN_DELIMITER);
        if (toks.size() == 2) {
            // todo-mshawlec: sanity checks
            // - if tokens.size != 2
            // - if duplicate tokens
            tokens[toks[0]] = toks[1];
        }
    }

    input.close();

    properties_ = &props;

    // todo-mshawlec sanity checks
    // - if field isn't present in header file
    properties_->width = std::stoul(tokens["Image Width"]);
    properties_->height = std::stoul(tokens["Image Height"]);
    properties_->bit_depth = std::stoul(tokens["Color Bit"]);
    properties_->frame_rate = std::stoul(tokens["Record Rate(fps)"]);
    properties_->num_frames = std::stoul(tokens["Total Frame"]);
}

bool MrawReader::check_first_line_(const std::string& line) {
    const std::string EXPECTED_FIRST_LINE = "#Camera Information Header";
    if (line == EXPECTED_FIRST_LINE) {
        return true;
    }
    else {
        std::cerr << "Expected first line '" + EXPECTED_FIRST_LINE + "'; got '"
            + line + "'" << std::endl;
    }
    return true;
}

cv::Mat MrawReader::read_frame(unsigned int n) {
    unsigned int npix = properties_->width * properties_->height;
    unsigned int nbytes = (npix * properties_->bit_depth) / 8;

    uint8_t *packed = new uint8_t[nbytes];
    cv::Mat unpacked = cv::Mat::zeros(properties_->height, properties_->width, CV_16U);

    // seek to frame if needed
    uint64_t offset = (n - 1) * ((uint64_t)nbytes);
    if (ifs_.tellg() != offset) {
        try {
            ifs_.seekg(offset);
        }
        catch(std::exception const& e) {
            std::cerr << "There was an error seeking to offset " << offset;
            std::cerr << ": " << e.what() << std::endl;
        }
    }
    ifs_.exceptions(std::ifstream::eofbit | std::ifstream::failbit |
            std::ifstream::badbit);

    // read from the file
    try {
        ifs_.read((char*)packed, nbytes);
    }
    catch(std::exception const& e) {
        std::cerr << "There was an error reading Mraw: " << e.what() << std::endl;
    }

    unpack_12bit(packed, unpacked, nbytes);

    delete[] packed;
    return unpacked;
}

}
