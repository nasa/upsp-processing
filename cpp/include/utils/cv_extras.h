/** @file
 *
 *  @brief OpenCV Extras
 */

#ifndef UTILS_CV_EXTRAS_H_
#define UTILS_CV_EXTRAS_H_

#include <sstream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../PSPVideo.h"

namespace upsp {

/** Iterator along a string that returns the size of the string up
 * to a given point
 */
struct TextSizeIterator : private totally_ordered<TextSizeIterator> {
public:
    using value_type        = unsigned int;
    using pointer           = unsigned int*;
    using reference         = unsigned int&;
    using difference_type   = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;

    /** Default constructor creates an iterator to end */
    TextSizeIterator();

    /* Create an iterator */
    TextSizeIterator(std::string text, int font_face, double font_scale,
            int thickness, unsigned int position);

    /** Evaluate any valid non-end iterator */
    unsigned int operator*() const; 

    /** Get the position in the string */
    int get_position() const;

    /** Move to the next position in the string */
    TextSizeIterator& operator++();

    /** Check that the iterators are equivalent */
    bool operator==(const TextSizeIterator& tsi) const;

    /** Check that this iterator is less than @a tsi */
    bool operator<(const TextSizeIterator& tsi) const;

private:

    std::string text;
    int font_face;
    double font_scale;
    int thickness;
    int position;

};

/** Convert cv::Mat::type() to string format */
std::string convert_type_string(int type);

/** Convert cv::Mat::depth() to string format */
std::string convert_depth_string(int cv_depth);

/** Convert Mat file to unsigned 8-biti (CV_8U)
 *  rescales to fill range 0-255
 */
cv::Mat convert2_8U(const cv::Mat& inp);

/** Get the AC component of a subset of a frame */
void get_ac_submat(const cv::Mat& input, cv::Mat& output, const cv::Rect roi);

/** Get the DC component of a subset of a series of frames */
void get_dc_submat(upsp::PSPVideo* cam, cv::Mat& output, const cv::Rect roi, 
        int num_frames, int start_frame=1);

/** Divide a string into multiple strings of limited printed text
 * size
 *
 * @param[in] input         string to split
 * @param[in] max_len       the maximum length of any string component in pixels
 * @param[in] font_face     openCV font face 
 * @param[in] font_scale    openCV font scale
 * @param[in] thickness     openCV font thickness
 * @param[in,out] splt      vector of the string components
 */
void split_text(std::string input, unsigned int max_len, int font_face, 
        double font_scale, int thickness, std::vector<std::string>& splt);

/** Round Point up to nearest integer in each direction
 * 
 * @param[in] pt input point
 * @return  rounded up @a pt
 */
template<typename T>
cv::Point3_<T> ceil(cv::Point3_<T>& pt);

/** Round Point down to nearest integer in each direction
 * 
 * @param[in] pt input point
 * @return  rounded down @a pt
 */
template<typename T>
cv::Point3_<T> floor(cv::Point3_<T>& pt);

/** Returns a unit vector perpendicular to the input vector
 * 
 * @param[in] vec   vector 
 * @return      unit vector perpendicular to @a vec
 *
 * @post if cv::norm(@a vec) == 0.0, return cv::Point3_<FP>(0,0,0)
 */
template<typename FP>
cv::Point3_<FP> get_perpendicular(cv::Point3_<FP> vec);

/** Determines the angle between two vectors
 *
 * @param[in] v1    first 3D vector
 * @param[in] v2    second 3D vector
 * @return          angle between two vectors in radians
 */
template<typename T>
double angle_between(const cv::Point3_<T>& v1, const cv::Point3_<T>& v2);

/** Rotate point given an axis of rotation and an angle
 *
 * @param[in] axis_dir  direction of the axis
 * @param[in] axis_pt   anchor point for the axis
 * @param[in] angle     angle to rotate (in radians)
 * @param[in] pt        point to rotate
 */
template<typename FP>
cv::Point3_<FP> rotate(const cv::Point3_<FP>& axis_dir, const cv::Point3_<FP>& axis_pt,
                       FP angle, const cv::Point3_<FP>& pt);

/** Compute the cross product */
template<typename FP>
cv::Point3_<FP> cross(const cv::Point3_<FP>& pt1, const cv::Point3_<FP>& pt2);

/** Parse comma separated 3D point */
template<typename T>
void parse_point3(std::string str, cv::Point3_<T>& pt);

/** Parse comma separated 2D point */
template<typename T>
void parse_point2(std::string str, cv::Point_<T>& pt);

/** Replace hot pixels with the median of their neighbors.  Hot pixels
 * are ones with values of thresh or higher, and where the replacement
 * with the median changes the value by at least min_change.  If more 
 * than max_hot pixels are above the threshold, nothing is done.
 */
void fix_hot_pixels(cv::Mat& inp, bool logit = false, int thresh = 4064,
  int min_change = 512, int max_hot = 5);

/** Generate a colormap image with accompanying colorbar, displaying
 * the number of grid nodes per pixel.
 *
 * @param[in] nodecounts 8UC1 image; px values = # nodes mapped to each px
 * @param[in] dst Output image with colormap + colorbar
 */
void nodes_per_pixel_colormap(cv::Mat& nodecounts, cv::Mat& dst);

} /* end namespace upsp */

#include "../utils/cv_extras.ipp"

#endif /* UTILS_CV_EXTRAS_H_ */
