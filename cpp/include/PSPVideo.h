/** @file
 *  @brief  Video Abstract Base Class
 *  @date   August 22, 2016
 *  @author jmpowel2
 */

#ifndef UFML_PSPVIDEO_H_
#define UFML_PSPVIDEO_H_

#include <memory>
#include <iterator>
#include <vector>
#include <opencv2/core.hpp>

#include "utils/general_utils.h"

namespace upsp {

/** Struct to hold video metadata */
struct VideoProperties {
    unsigned int num_frames;
    unsigned int width;
    unsigned int height;
    unsigned int bit_depth;
    unsigned int frame_rate;
    float exposure;
    float aperture;
};

/** Abstract interface for all readers */
class VideoReader {
public:
    virtual ~VideoReader() {};

    /** Populate as many properties as possible */
    virtual void load_properties(VideoProperties &props) = 0;

    /** Read a single frame by 1-based index */
    virtual cv::Mat read_frame(unsigned int n) = 0;
};

class PSPVideo {
public:
    PSPVideo(std::unique_ptr<VideoReader> reader);
    virtual ~PSPVideo() {};

    /** Get the specified frame
     *
     * @param[in] frame_num frame number (1-based)
     */
    cv::Mat get_frame(unsigned int frame_num);

    /** Get the next frame
     */
    cv::Mat get_next();

    /** Get all frames in range
     *
     * @param[in] frame1    first frame to retrieve (1-based)
     * @param[in] frame2    last frame to retrieve (1-based)
     * @param[out] output   all frames in range [@a frame1, @a frame2]
     */
    void get_frames(unsigned int frame1, unsigned int frame2,
            std::vector<cv::Mat>& output);

    /** Get the number of frames in the video */
    unsigned int get_number_frames() const;

    /** Get the current frame number (1-based) */
    unsigned int get_current_frame() const;

    /** Get the video frame rate in Hz */
    unsigned int get_frame_rate() const;

    /** Get the size of the video frames in pixels */
    cv::Size get_frame_size() const;

    /** Get the frame width in pixels */
    unsigned int get_width() const;

    /** Get the frame height in pixels */
    unsigned int get_height() const;

    /** Get the bit depth of each pixel */
    unsigned int get_bit_depth() const;

    /* Get the video exposure in microseconds */
    float get_exposure() const;

    /** Get the camera aperture setting */
    float get_aperture() const;


    class VideoIterator;

    /** Get iterator to the first frame */
    VideoIterator begin();

    /** Get iterator to one past the last frame */
    VideoIterator end();

    /** Iterator class for moving through video */
    class VideoIterator : private totally_ordered<VideoIterator> {
    public:
        using value_type        = cv::Mat;
        using pointer           = cv::Mat*;
        using reference         = cv::Mat&;
        using difference_type   = int;
        using iterator_category = std::random_access_iterator_tag;

        /** Create invalid iterator */
        VideoIterator();

        /** Get the current frame */
        cv::Mat operator*() const;

        /** Index into the frames 1-based
         * @pre n must be in range [1,video_->get_number_frames()]
         */
        cv::Mat operator[](int n) const;

        /** Increment the iterator by 1 */
        VideoIterator& operator++();

        /** Decrement iterator by 1
         * @pre --curr_ >= 1
         */
        VideoIterator& operator--();

        /** Shift iterator by n
         * @pre (curr_ + n) >= 1
         */
        VideoIterator& operator+=(int n);

        /** Create iterator shifted forward by n
         */
        VideoIterator operator+(int n) const;

        /** Create iterator shifted backward by n
         */
        VideoIterator operator-(int n) const;

        /** Determine distance between iterators
         * @pre there exists a value n s.t. this+n == b
         */
        int operator-(VideoIterator a);

        /** Check if this iterator is equivalent to @a vi */
        bool operator==(const VideoIterator& vi) const;

        /** Determine if this is less than @a vi in a global order
         * if when comparing va, vb one has nullptr video:
         *      va < vb if vb.video_ == nullptr and !va.is_end_iterator()
         */
        bool operator<(const VideoIterator& vi) const;

    private:
        friend class PSPVideo;

        /** Create valid iterator */
        VideoIterator(PSPVideo* video, unsigned int frame);

        /** Return true if this is an end iterator */
        bool is_end_iterator() const;

        PSPVideo* video_;
        unsigned int curr_;
    };

private:
    std::unique_ptr<VideoReader> reader_;
    VideoProperties properties_;
    unsigned int current_frame_;  // counting starts at 1
};

/**
 * Unpack nbytes of 10-bit data from packed into unpacked. Pixels are assumed
 * to be stored MSBit first and MSByte first, and adjacent pixels are assumed
 * to belong to the same row in the image (except at row boundaries).
 *
 *     byte 0: [ A------- ]
 *     byte 1: [ -aB----- ]
 *     byte 2: [ ---bC--- ]
 *     byte 3: [ -----cD- ]
 *     byte 4: [ -------d ]
 *     ...
 * */
void unpack_10bit(uint8_t *packed, cv::Mat &unpacked, size_t nbytes);

/**
 * Unpack nbytes of 12-bit data from packed into unpacked. Pixels are assumed
 * to be stored MSBit first and MSByte first, and adjacent pixels are assumed
 * to belong to the same row in the image (except at row boundaries).
 *
 *     byte 0: [ A------- ]
 *     byte 1: [ ---aB--- ]
 *     byte 2: [ -------b ]
 *     ...
 */
void unpack_12bit(uint8_t *packed, cv::Mat &unpacked, size_t nbytes);

} /* end namespace upsp */

#endif /* UFML_PSPVIDEO_H_ */
