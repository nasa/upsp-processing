/** @file
 *  @brief  Basic Image Statistics and Writing
 *  @date   August 23, 2017
 *  @author jmpowel2
 */

#ifndef UFML_IMAGE_PROCESSING_H_
#define UFML_IMAGE_PROCESSING_H_

#include <algorithm>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>

#include "data_structs.h"

#include "PSPVideo.h"
#include "image_processing.h"
#include "models.h"
#include "utils/cv_extras.h"
#include "utils/general_utils.h"

namespace upsp {

/** Operator for removing the DC component of an image */
struct SubtractDC : std::unary_function<cv::Mat, cv::Mat> {

    /** Default constructor, creates invalid operator */
    SubtractDC();

    /** Set the region of interest */
    SubtractDC(cv::Rect roi);

    /** Subtract the set DC component from @a inp */
    cv::Mat operator()(cv::Mat inp) const;

    /***************************************************************/    

    cv::Rect roi_;
};

/** Removes a timestamp located in the bottom left corner */
struct TimeStamp : std::unary_function<cv::Mat, cv::Mat> {

    /** Find a time stamp in a sample image
     *  box location must be stationary
     * 
     * @param[in] sample    image with a timestamp in the bottom left
     *                      corner inside a rectangle
     */ 
    TimeStamp(const cv::Mat& sample);

    /** Set the area in the timestamp rectangle to 0 */
    cv::Mat operator()(cv::Mat inp) const;

    /** Set the area in the time stamp with the average value in
     *  the frame
     */
    cv::Mat fill_avg(cv::Mat inp) const;

    /***************************************************************/

    cv::Rect ts_;
};

/** Put a string in upper right corner on a frame 
 */
struct Stamp : std::unary_function<cv::Mat, cv::Mat> {

    /** Define the stamp that will be placed on frames
     * @param[in] stamp_in      string containing all terms separated by ';' characters
     * @param[in] max_length    the maximum length (width) in pixels for the block of text
     *                          to fill (will be automatically adjusted up if the longest
     *                          term is too small to fit in this space
     * @param[in] img_size      size of the images that will be provided
     */
    Stamp(std::string stamp_in, unsigned int max_length, cv::Size img_size); 

    /** Add stamp to the frame
     */
    cv::Mat operator()(const cv::Mat& inp) const;

    /***************************************************************/

    std::vector<std::string> terms;
    std::vector<cv::Point2i> start_pts;

    cv::Size img_size;
    std::string stamp;
    unsigned int max_length;
    int font_face;
    double font_scale;
    cv::Scalar color;
    int thickness;

}; 

/** Subtract the DC component from a frame 
 *
 * @param[in,out] inp   image to update
 * @param[in] roi       region over which to average pixels for "DC" component, if empty
 *                      average over entire frame 
 */
void subtract_dc(cv::Mat& inp, cv::Rect roi = cv::Rect());

/** Find all horizontally and vertically aligned rectangles in a frame 
 * (sensitive to many parameters, likely to require tweaking for other applications)
 *
 * @param[in] inp       image frame
 * @param[in,out] rects vector of found rectangles
 *
 */
void find_rectangles(const cv::Mat& inp, std::vector<cv::Rect>& rects);

/** Put an image on a colormap and add a colorbar
 *
 * @param[in] input     reference image
 * @param[out] output   colorized image
 * @param[in] map       color map to use
 */
void colorize_image(const cv::Mat& input, cv::Mat& output, 
        int colormap = cv::COLORMAP_JET);

/** Get discrete values for a colormap
 *
 * @param[in] count     number of values
 * @param[out] colors   RGB color values
 * @param[in] colormap  openCV colormap
 */
void get_colors(unsigned int count, std::vector<cv::Scalar>& colors, 
        int colormap=cv::COLORMAP_JET);

/** Write out histogram data in csv format
 *
 * @param[in] filename  file for writing
 * @param[in] img       image from which to collect data
 * @param[in] depth     maximum depth of images
 * @param[in] bins      number of bins, if -1, use 2^depth
 *
 * @pre @a img must be either CV_8U or CV_16U
 */
template<typename T>
void intensity_histogram(std::string filename, const cv::Mat_<T>& img, 
        unsigned int depth =12, int bins = -1);
/** Write out histogram data in space-separated columns
 *
 * @param[in] filename  file for writing
 * @param[in] imgs      vector of images from which to collect data
 * @param[in] depth     maximum depth of images
 *
 * @pre all @a imgs must be of the same type (either CV_8U or CV_16U)
 * @pre no value in any @a imgs may have value >= 2^depth
 */
template<typename T>
void intensity_histogram(std::string filename, const std::vector<cv::Mat_<T>>& imgs, 
        unsigned int depth = 12);

/** Generate histogram counts
 *
 * @param[in] img       image from which to collect data
 * @param[out] edges    bin edges
 * @param[out] counts   bin counts
 * @param[in] depth     maximum depth of images
 * @param[in] bins      number of bins, if -1, use 2^depth
 *
 * @pre @a img must be either CV_8U or CV_16U
 * @post @a edges.size() -1 == @a counts.size() == @a bins
 */
template<typename T>
void intensity_histc(const cv::Mat_<T>& img, std::vector<int>& edges, 
        std::vector<int>& counts, unsigned int depth = 12, int bins = -1);

/** Average all frames pixel-by-pixel in range
 *
 * @param[in] begin     beginning of range
 * @param[in] end       one past the end of the range
 * @return              if @a begin != @a end, a Mat of depth CV_64F with the average 
 *                      of all frames in the range, otherwise cv::Mat()
 *
 * @pre evaluating the iterator should return cv::Mat or cv::Mat& or const cv::Mat&
 */
template<typename InputIterator>
cv::Mat average_frames(InputIterator begin, InputIterator end);

/** Add targets to an image
 *
 * @param[in] targs             Targets to overlay onto image
 * @param[in] src               Input Mat frame over which to lay targets
 * @param[in] color             target color
 * @param[in] include_labels    flag for including target ids 
 * @param[in] include_size      use the target diameter to set size of mark
 * @return output               Output Mat for src with targets
 */
template<typename T>
cv::Mat add_targets(const std::vector<T>& targs, const cv::Mat& src, 
        cv::Scalar color = cv::Scalar(0,255,0), bool include_labels = true, 
        bool include_size = false);

/** Add a single target to an image 
 *
 * @param[in] targs             Target to overlay onto image
 * @param[in] src               Input Mat frame over which to lay target
 * @param[in] color             target color
 * @param[in] include_labels    flag for including target id
 * @param[in] include_size      use the target diameter to set size of mark
 * @return output               Output Mat for src with target
 */
template<typename T>
cv::Mat add_targets(const T& targs, const cv::Mat& src, 
        cv::Scalar color = cv::Scalar(0,255,0), bool include_labels = true, 
        bool include_size = false);

/** Add points to an image
 *
 * @param[in] pts               Points to overlay onto image
 * @param[in] src               Input Mat frame over which to lay points
 * @param[in] color             point color
 * @param[in] include_labels    flag for numbering points by index+1 in image
 * @return output               Output Mat for src with points
 */
template<typename FP>
cv::Mat add_points(const std::vector<cv::Point_<FP>>& pts, const cv::Mat& src,
        cv::Scalar color = cv::Scalar(0,255,0), bool include_labels = false);

/** Add kulites to an image
 *
 * @param[in] kuls      Kulites to overlay onto image
 * @param[in] src       Input Mat frame over which to lay kulites
 * @return output       Output Mat for src with kulites
 */
template<typename T>
cv::Mat add_kulites(const std::vector<Kulite_<T>>& kuls, const cv::Mat& src);

/** Nicely fit 3d planar data onto a 2d frame
 *
 * @param[in] targs     Targets with xyz data (z ignored)
 * @param[in] size      Size of the frame to fit the data onto
 * @param[out] pts      2d points fitting nicely into @a size
 * @return              scale factor used to scale data
 */
template<typename T>
double fit_targets(const std::vector<T>& targs, cv::Size size, 
        std::vector<cv::Point2d>& pts);

/** Create a Mat to display 3d planar (z ignored) target locations
 *
 * @param[in] targs     Targets to display (need uv information)
 * @param[in] size      Size of the Mat
 * @param[in] add_label if true, add @a targs num to each plotted target
 * @return              image with targets
 *
 * @post if for all i, 0 <= i < targs.size(),  @a targs[i].diameter > 0, then
 *      all @a targs will be displayed with their given diameter
 * @post if for all i, 0 <= i < targs.size(), @a targs[i].num == 0, and @a add_label
 *      then, labels will be 1 + @a targs index
 */
template<typename T>
cv::Mat plot_targets(const std::vector<T>& targs, cv::Size size, bool add_label = false);

/** Create a Mat to display target normalss
 *
 * @param[in] targs     Targets to display (need uv information)
 * @param[in] src       Input Mat frame over which to lay targets
 * @param[in] color     Target normal color
 * @param[in] include_size      use the target diameter to set size of mark
 * @param[in] norms     Target normals
 * @param[in] tip       opencv arrowedLine tip variable
 * @return              image with target normals
 *
 * @post if for all i, 0 <= i < targs.size(),  @a targs[i].diameter > 0, then
 *      all @a targs will be displayed with their given diameter
 */
template<typename T>
cv::Mat add_target_normals(const std::vector<T>& targs, const cv::Mat& src,
        cv::Scalar color /*=cv::Scalar(0,255,0)*/, bool include_size /*=false*/,
        std::vector<cv::Point_<float>>& norms, float tip);

} /* end namespace upsp */

#include "../lib/image_processing.ipp"

#endif /* UFML_IMAGE_PROCESSING_H_ */
