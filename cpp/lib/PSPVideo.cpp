/** @file
 *  @brief  Video Abstract Base Class
 *  @date   August 22, 2016
 *  @author jmpowel2
 */

#include "PSPVideo.h"

#include <memory>
#include <cassert>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace upsp {

/*****************************************************************************/
PSPVideo::PSPVideo(std::unique_ptr<VideoReader> reader)
    : reader_(std::move(reader)),
      properties_{0, 0, 0, 0, 0, 0, 0},
      current_frame_(0) {
    reader_->load_properties(properties_);
}

/*****************************************************************************/
PSPVideo::VideoIterator PSPVideo::begin() {
    return VideoIterator(this, 1);
}

/*****************************************************************************/
PSPVideo::VideoIterator PSPVideo::end() {
    return VideoIterator(this, get_number_frames()+1);
}

/*****************************************************************************/
unsigned int PSPVideo::get_number_frames() const {
    return properties_.num_frames;
}

/*****************************************************************************/
unsigned int PSPVideo::get_current_frame() const {
    return current_frame_;
}

/*****************************************************************************/
unsigned int PSPVideo::get_frame_rate() const {
    return properties_.frame_rate;
}

/*****************************************************************************/
cv::Size PSPVideo::get_frame_size() const {
    return cv::Size(properties_.width, properties_.height);
}

/*****************************************************************************/
unsigned int PSPVideo::get_width() const {
    return properties_.width;
}

/*****************************************************************************/
unsigned int PSPVideo::get_height() const {
    return properties_.height;
}

/*****************************************************************************/
unsigned int PSPVideo::get_bit_depth() const {
    return properties_.bit_depth;
}

/*****************************************************************************/
float PSPVideo::get_exposure() const {
    return properties_.exposure;
}

/*****************************************************************************/
float PSPVideo::get_aperture() const {
    return properties_.aperture;
}

/*****************************************************************************/
cv::Mat PSPVideo::get_next() {
    return get_frame(current_frame_ + 1);
}

/*****************************************************************************/
cv::Mat PSPVideo::get_frame(unsigned int frame_num) {
    if ((frame_num > 0) && (frame_num <= get_number_frames())) {
        current_frame_ = frame_num;
        return reader_->read_frame(current_frame_);
    }
    else {
        throw std::invalid_argument("frame number is outside range of frames");
    }
    return cv::Mat();
}

/*****************************************************************************/
void PSPVideo::get_frames(unsigned int frame1, unsigned int frame2,
        std::vector<cv::Mat>& output) {
    if ((frame2 < frame1) || (frame1 <= 0) || (frame2 > get_number_frames())) {
        throw std::invalid_argument("requested frames are outside of range");
    }

    int nframes = frame2 - frame1 + 1;
    output.resize(nframes);
    for (int i = 0; i < nframes; i++) {
        output[i] = reader_->read_frame(frame1+i);
    }
    current_frame_ = frame2;
}

void unpack_10bit(uint8_t *packed, cv::Mat &unpacked, size_t nbytes) {
    cv::MatIterator_<uint16_t> dst = unpacked.begin<uint16_t>();
    uint16_t p, q, r, s, t;
    for (size_t i = 0; i < nbytes ; i += 5, dst += 4) {
        // promote 8-bit directly to 16-bit
        p = packed[i];
        q = packed[i+1];
        r = packed[i+2];
        s = packed[i+3];
        t = packed[i+4];

        dst[0] = (p << 2) | (q >> 6);
        dst[1] = ((q & 0x3F) << 4) | (r >> 4);
        dst[2] = ((r & 0x0F) << 6) | (s >> 2);
        dst[3] = ((s & 0x03) << 8) | (t);

        CV_Assert(dst[0] < 1024);
        CV_Assert(dst[1] < 1024);
        CV_Assert(dst[2] < 1024);
        CV_Assert(dst[3] < 1024);
    }
}

void unpack_12bit(uint8_t *packed, cv::Mat &unpacked, size_t nbytes) {
    cv::MatIterator_<uint16_t> dst = unpacked.begin<uint16_t>();
    uint16_t p, q, r;
    for (size_t i = 0; i < nbytes; i += 3, dst += 2) {
        // promote 8-bit directly to 16-bit
        p = packed[i];
        q = packed[i+1];
        r = packed[i+2];

        dst[0] = (p << 4) | (q >> 4);
        dst[1] = ((q & 0xF) << 8) | r;

        CV_Assert(dst[0] < 4096);
        CV_Assert(dst[1] < 4096);
    }
}


/********************************************************************
 * VideoIterator
********************************************************************/

/*****************************************************************************/
PSPVideo::VideoIterator::VideoIterator() : video_(nullptr), curr_(0)  {}

/*****************************************************************************/
cv::Mat PSPVideo::VideoIterator::operator*() const {
    return this->video_->get_frame(curr_);
}

/*****************************************************************************/
cv::Mat PSPVideo::VideoIterator::operator[](int n) const {
    return this->video_->get_frame(n);
}

/*****************************************************************************/
PSPVideo::VideoIterator& PSPVideo::VideoIterator::operator++() {
    ++curr_;
    return *this;
}

/*****************************************************************************/
PSPVideo::VideoIterator& PSPVideo::VideoIterator::operator--() {
    --curr_;
    return *this;
}

/*****************************************************************************/
PSPVideo::VideoIterator& PSPVideo::VideoIterator::operator+=(int n) {
    curr_ += n;
    return *this;
}

/*****************************************************************************/
PSPVideo::VideoIterator PSPVideo::VideoIterator::operator+(int n) const {
    return VideoIterator(video_, curr_+n);
}

/*****************************************************************************/
PSPVideo::VideoIterator PSPVideo::VideoIterator::operator-(int n) const {
    return VideoIterator(video_, curr_-n);
}

/*****************************************************************************/
int PSPVideo::VideoIterator::operator-(VideoIterator a) {
    return curr_ - a.curr_;
}

/*****************************************************************************/
bool PSPVideo::VideoIterator::operator==(const VideoIterator& vi) const {
    if (is_end_iterator()) {
        if ( (video_ != nullptr) && (vi.video_ != nullptr) ) {
            return video_ == vi.video_;
        } else {
            return vi.is_end_iterator();
        }
    } else {
        if (vi.is_end_iterator()) {
            return false;
        } else {
            return ( (video_ == vi.video_) && (curr_ == vi.curr_) );
        }
    }
}

/*****************************************************************************/
bool PSPVideo::VideoIterator::operator<(const VideoIterator& vi) const {
    if (video_ == vi.video_) {
        if (video_ == nullptr) {
            return false;  // both equivalent end iterators
        } else {
            return curr_ < vi.curr_;
        }
    } else {
        if (video_ == nullptr) {
            return false;
        } else if (vi.video_ == nullptr) {
            return !is_end_iterator();
        } else {
            return video_ < vi.video_;
        }
    }
}

/*****************************************************************************/
PSPVideo::VideoIterator::VideoIterator(PSPVideo* video,
        unsigned int frame) : video_(video), curr_(frame) {
    assert(video != nullptr);
    assert(frame >= 1);
}

/*****************************************************************************/
bool PSPVideo::VideoIterator::is_end_iterator() const {
    return ( (video_ == nullptr) || (curr_ >= (video_->get_number_frames()+1)) );
}


} /* end namespace upsp */

