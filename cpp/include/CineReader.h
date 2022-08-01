/** @file
 *  @brief  Read Cine Video
 *  @date   August 22, 2016
 *  @author jmpowel2
 */

#ifndef UFML_CINEREADER_H_
#define UFML_CINEREADER_H_

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>

#include "PSPVideo.h"
#include "vr_cine.h"

#define TWO_POW(p) (1 << (p))

namespace upsp {

/** Cine File Reader */
class CineReader : public VideoReader {
public:

    /** Create empty CineReader reader object */
    CineReader(const std::string& filepath);

    /** Close CineReader */
    virtual ~CineReader();

    /** Print a summary of the cine file metadata to stdout */
    void summarize();

    /** Get the number of bits per pixel as packed in the file. This is usually
     * the same as the bit depth, unless frames are uncompressed upon reading */
    unsigned int get_bits_per_pixel() const;

    /** Write header details to stream */
    virtual void write_header(std::ofstream& ofs);

    /** Print all header details to stream */
    void dump_header(std::ostream& out) const;

    void load_properties(VideoProperties &props) override;

    cv::Mat read_frame(unsigned int n) override;

protected:
    std::string filepath_;
    std::ifstream ifs_;
    unsigned int bits_per_pixel_;
    VideoProperties *properties_;

    /** Read and validate offsets from file header */
    void read_offsets();

    /** Read a 10-bit or 12-bit packed frame */
    cv::Mat read_packed();

    /** Read an 8-bit frame */
    cv::Mat read_linear();

    /** Read @a size bytes from the given file descriptor.
     *  Exits on error or short read.
     *
     * @param[out] buf      pointer to buffer
     * @param[in] N         number of bytes to read
     */
    void read(void* buf, size_t N);

    // VR structure
    CINEFILEHEADER cine_header_;
    BITMAPINFOHEADER bitmap_header_;
    SETUP setup_;
    std::vector<uint64_t> image_offsets_;
    // after every public function, curr_offset_ is the same as the associated
    // file's stream position
    uint64_t curr_offset_;
    // after every public function, wanted_offset_ is the offset of first byte of
    // image data for the next frame to be read (skipping past any per-frame header)
    uint64_t wanted_offset_;

};

/** Print out CineReader information */
struct CinePrint {
    template<typename T>
    void operator()(const T& t) const {
        std::string field_name = boost::fusion::extension::struct_member_name<
                tagCINEFILEHEADER,0>::call();
        std::cout << field_name << std::endl;
        std::cout << std::boolalpha << typeid(t).name() << ' ';
        std::cout << boost::lexical_cast<std::string>(t) << ' ' << t << std::endl;
    }
};

} /* end namespace upsp */

#endif /* UFML_CINEREADER_H_ */
