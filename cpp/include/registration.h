/** @file
 * 
 *  @brief  Image and Point Set Registration
 *  @date   August 18, 2017
 *  @author jmpowel2
 */

#ifndef UFML_REGISTRATION_H_
#define UFML_REGISTRATION_H_

#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <opencv2/opencv.hpp>

#include "data_structs.h" 
#include "image_processing.h"
#include "utils/general_utils.h"
#include "utils/cv_extras.h"

namespace upsp {

/** @brief Type of Mapping
 */
enum TransformType { AFFINE, HOMOGRAPHY };

/** Register an image to a reference image */
struct RegisterImage : std::unary_function<cv::Mat, cv::Mat> {

    /** Initialize the reference image */
    RegisterImage(const cv::Mat& ref_img); 

    /** Perform pixel based image registration */
    cv::Mat operator()(cv::Mat inp);

    /***************************************************************/

    cv::Mat wm_;
    const cv::Mat& ref_img_;
    int max_iters;
    double epsilon;
};

/** Use pixel-based registration to align an image to a reference image, assumes
 * an affine transformation
 *
 * @param[in] ref_img   reference image
 * @param[in] inp_img   image to be aligned to reference
 * @param[out] warp_matrix  mapping between @a ref_img and @a inp_img
 * @param[in] max_iters maximum number of iterations for ECC algorithm
 * @param[in] epsilon   threshold of the increment in correlation coefficient between
 *                      two iterations, criteria for ECC algorithm to stop
 * @param[in] interpolation_flags pixel interpolation method for image warping
 *                      two iterations, criteria for ECC algorithm to stop
 * @return              @a inp_img warped to align with @a ref_img
 *
 * @pre @a ref_img and @a inp_img are grayscale images
 * @pre @a ref_img is CV_32F
 */
cv::Mat register_pixel(const cv::Mat& ref_img, const cv::Mat& inp_img, 
        cv::Mat& warp_matrix, int max_iters=50, double epsilon=0.001,
        int interpolation_flags=cv::INTER_LINEAR);

/** Remove match pairs where the distance between the points is significantly large
 *
 * @param[in] ref           reference points
 * @param[in] pts           points
 * @param[in,out] matches   matching between @a ref and @a pts
 * @param[in] cutoff        standard deviations above mean for rejecting pairs
 */
template<typename FP>
void unmatch_outliers(const std::vector<cv::Point_<FP>>& ref,
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        float cutoff=3);

/** Find all circular targets in frame
 * Uses SimpleBlobDetector with filtering
 *
 * @param[in]       src             image frame
 * @param[in,out]   targs           vector of found targets (will be cleared before use)
 * @param[in]       min_diameter    minimum allowable target diameter, in pixels
 * @param[in]       max_diameter    maximum allowable target diameter, in pixels,
 *                                  if <= 0.0, will use 10% of maximum dimension
 * @return                          number of targets found
 */
template<typename T>
int find_targets(const cv::Mat& src, std::vector<T>& targs, 
        double min_diameter=2.0, double max_diameter=0.0);

/** Find all circular targets in regions of interest
 * Uses SimpleBlobDetector with filtering
 *
 * @param[in]       src             image frame
 * @param[in,out]   targs           vector of found targets for each roi 
 *                                  (will be cleared before use)
 * @param[in]       roi             regions to search for targets
 * @param[in]       min_diameter    minimum allowable target diameter, in pixels
 * @param[in]       max_diameter    maximum allowable target diameter, in pixels,
 *                                  if <= 0.0, will use 10% of maximum dimension
 * @return                          number of targets found
 *
 * @post targs.size() == roi.size()
 * @post sz == \f$\sum_{i=1}^{roi.size()} targs[i].size()\f$
 */
template<typename T>
int find_targets_roi(const cv::Mat& src, const std::vector<cv::Rect>& roi, 
        std::vector<std::vector<T>>& targs, double min_diameter=2.0, 
        double max_diameter=0.0);

/** For each reference point find the point that is closest
 *
 * @tparam InputIterator1   InputIterator with value_type openCV Point2 or Point3
 * @tparam ForwardIterator2 ForwardIterator with value_type openCV Point2 or Point3
 * @param[in] ref_begin     beginning of reference points
 * @param[in] ref_end       one past the end of reference points
 * @param[in] pts_begin     points to match to @a ref (begin iterator)
 * @param[in] pts_end       points to match to @a ref (end iterator)
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 *
 * @post for all 0 <= i < distance(ref_begin, ref_end), 
 *       matches[i] < distance(pts_begin,pts_end)
 * @post @a matches.size() == distance(ref_begin, ref_end)
 *
 * Complexity O(ref_size * pts_size) where  ref_size = distance(ref_begin, ref_end) and
 *                                          pts_size = distance(pts_begin, pts_end)
 */
template<typename InputIterator1, typename ForwardIterator2>
void closest_point(const InputIterator1 ref_begin, const InputIterator1 ref_end,
        ForwardIterator2 pts_begin, ForwardIterator2 pts_end, std::vector<int>& matches);


/** For each reference point find the point that is closest
 *
 * @tparam Pt               openCV Point2(d/f) or Point3(d/f)
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 *
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 * @post @a matches.size() == ref.size()
 *
 * Complexity O(ref.size() * pts.size())
 */
template<typename Pt>
void closest_point(const std::vector<Pt>& ref, const std::vector<Pt>& pts, 
        std::vector<int>& matches);

/** For each reference point find the point that is closest without repeats
 *
 * @tparam InputIterator1   InputIterator with value_type openCV Point2 or Point3
 * @tparam ForwardIterator2 ForwardIterator with value_type openCV Point2 or Point3
 * @param[in] ref_begin     beginning of reference points
 * @param[in] ref_end       one past the end of reference points
 * @param[in] pts_begin     points to match to @a ref (begin iterator)
 * @param[in] pts_end       points to match to @a ref (end iterator)
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 *
 * @post for all 0 <= i < distance(ref_begin, ref_end), 
 *       matches[i] < distance(pts_begin,pts_end)
 * @post @a matches.size() == distance(ref_begin, ref_end)
 * @post for all 0 <= i,j < distance(ref_begin, ref_end) and i != j and matches[i] >= 0, 
 *      matches[i] != matches[j]
 *
 * Complexity O(ref_size * pts_size) where  ref_size = distance(ref_begin, ref_end) and
 *                                          pts_size = distance(pts_begin, pts_end)
 */
template<typename InputIterator1, typename ForwardIterator2>
void closest_point2(const InputIterator1 ref_begin, const InputIterator1 ref_end,
        ForwardIterator2 pts_begin, ForwardIterator2 pts_end, std::vector<int>& matches);


/** For each reference point find the point that is closest without repeats
 *
 * @tparam Pt               openCV Point2(d/f) or Point3(d/f)
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 *
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *      matches[i] != matches[j]
 *
 * Complexity O(ref.size() * pts.size())
 */
template<typename Pt>
void closest_point2(const std::vector<Pt>& ref, const std::vector<Pt>& pts, 
        std::vector<int>& matches);

/** For each reference point find the point that is closest without repeats
 *  account for size of the points
 *
 * @tparam InputIterator1   InputIterator with value_type Target_
 * @tparam ForwardIterator2 ForwardIterator with value_type Target_
 * @param[in] ref_begin     beginning of reference points
 * @param[in] ref_end       one past the end of reference points
 * @param[in] pts_begin     points to match to @a ref (begin iterator)
 * @param[in] pts_end       points to match to @a ref (end iterator)
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 * @param[in] size_buff     fraction of reference Target size that points must
 *                          fall within for a valid match
 *
 * @post for all 0 <= i < distance(ref_begin, ref_end), 
 *       matches[i] < distance(pts_begin,pts_end)
 * @post @a matches.size() == distance(ref_begin, ref_end)
 * @post for all 0 <= i,j < distance(ref_begin, ref_end) and i != j and matches[i] >= 0, 
 *      matches[i] != matches[j]
 *
 * Complexity O(ref_size * pts_size) where  ref_size = distance(ref_begin, ref_end) and
 *                                          pts_size = distance(pts_begin, pts_end)
 */
template<typename InputIterator1, typename ForwardIterator2>
void closest_point3(const InputIterator1 ref_begin, const InputIterator1 ref_end,
        ForwardIterator2 pts_begin, ForwardIterator2 pts_end, std::vector<int>& matches,
        float size_buff);

/** For each reference point find the point that is closest without repeats
 *  account for size of the points
 *
 * @tparam FP               floating point (float or double)
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between pts and ref, 
 *                          ref[i] corresponds to pts[matches[i]] 
 *                          where i is an index in range [ref_begin,ref_end)
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i]
 * @param[in] size_buff     fraction of reference Target size that points must
 *                          fall within for a valid match
 *
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *      matches[i] != matches[j]
 *
 * Complexity O(ref.size() * pts.size())
 */
template<typename FP>
void closest_point3(const std::vector<Target_<FP>>& ref, 
        const std::vector<Target_<FP>>& pts, 
        std::vector<int>& matches, float size_buff = 0.1);

/** Match a vector of points to a vector of reference points using iterative closest
 * point, each ref will have at most 1 match from pts 
 * ICP is currently (9/14/17) available in openCV contrib, once it moves to the production
 * openCV it will be used here
 *
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between @a pts and @a ref, 
 *                          for 0 <= i < ref.size() , ref[i] corresponds to pts[matches[i]]
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i] 
 * @param[in] iters         maximum number of iterations 
 * @param[in] resid         when l2-norm of euclidean distance between matched points
 *                          is < @a resid, the algorithm will terminate
 * @return                  l2-norm of euclidean distances between matched points
 *
 * @pre @a iters > 0
 * @pre @a resid > 0.0
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *       matches[i] != matches[j]
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 */
template<typename FP>
double iterative_closest_point(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters=100, double resid=1.0e-7);

/** Match a vector of points to a vector of reference points using iterative closest
 * point, each ref will have at most 1 match from pts 
 * take size of the points into account
 * ICP is currently (9/14/17) available in openCV contrib, once it moves to the production
 * openCV it will be used here
 *
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between @a pts and @a ref, 
 *                          for 0 <= i < ref.size() , ref[i] corresponds to pts[matches[i]]
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i] 
 * @param[in] iters         maximum number of iterations 
 * @param[in] resid         when l2-norm of euclidean distance between matched points
 *                          is < @a resid, the algorithm will terminate
 * @param[in] size_buff     fraction of reference Target size that points must
 *                          fall within for a valid match
 * @return                  l2-norm of euclidean distances between matched points
 *
 * @pre @a iters > 0
 * @pre @a resid > 0.0
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *       matches[i] != matches[j]
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 */
template<typename FP>
double iterative_closest_point2(const std::vector<Target_<FP>>& ref, 
        const std::vector<Target_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters=100, double resid=1.0e-7, double size_buff=0.1);

/** Match a vector of points to a vector of reference points using coherent point drift
 * for affine transformations each ref will have at most 1 match from pts 
 *
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between @a pts and @a ref, 
 *                          for 0 <= i < ref.size() , ref[i] corresponds to pts[matches[i]]
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i] 
 * @param[in] iters         maximum number of iterations 
 * @param[in] resid         when norm of difference between transformed points in successive
 *                          iterations is < @a resid, the algorithm will terminate
 * @return                  l2-norm of euclidean distances between matched points
 *
 * @pre @a iters > 0
 * @pre @a resid > 0.0
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *       matches[i] != matches[j]
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 */
template<typename FP>
float cpd_affine(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters=100, float resid=1e-5);

/** Match a vector of points to a vector of reference points using coherent point drift
 * each ref will have at most 1 match from pts 
 *
 * @param[in] ref           reference points
 * @param[in] pts           points to match to @a ref
 * @param[in,out] matches   matching between @a pts and @a ref, 
 *                          for 0 <= i < ref.size() , ref[i] corresponds to pts[matches[i]]
 *                          unless matches[i] < 0 , in which case 
 *                          no match is found for ref[i] 
 * @param[in] iters         maximum number of iterations 
 * @param[in] resid         when norm of difference between transformed points in successive
 *                          iterations is < @a resid, the algorithm will terminate
 * @return                  l2-norm of euclidean distances between matched points
 *
 * @pre @a iters > 0
 * @pre @a resid > 0.0
 * @post for all 0 <= i,j < ref.size() and i != j and matches[i] >= 0, 
 *       matches[i] != matches[j]
 * @post @a matches.size() == ref.size()
 * @post for all 0 <= i < ref.size(), matches[i] < pts.size()
 */
template<typename FP>
float coherent_point_drift(const std::vector<cv::Point_<FP>>& ref, 
        const std::vector<cv::Point_<FP>>& pts, std::vector<int>& matches, 
        unsigned int iters=100, float resid=1e-6);

/** Find and match targets to a reference
 *
 * @param[in] img           image to search for targets
 * @param[in] ref           reference target locations
 * @param[out] found        found targets 
 * @param[out] unmatched    indices of all ref that were not matched to a found target
 * @return                  number of found targets
 *
 * @post found[i] matches with ref[found[i].num]
 */
template<typename T>
int identify_targets(const cv::Mat& img, 
        const std::vector<cv::Point_<typename T::data_type>>& ref, 
        std::vector<T>& found, std::vector<int>& unmatched);

/** Create regions around image points of certain radius
 *
 * @param[in] sz        image size
 * @param[in] ref       image points of interest
 * @param[in] pradius   radius in pixels
 * @param[out] roi      rectangles encompassing each point
 */
template<typename FP>
void create_roi(cv::Size sz, const std::vector<cv::Point_<FP>>& ref, 
        unsigned int pradius, std::vector<cv::Rect>& roi);

/** Find closest target within a range of reference locations
 *
 * @param[in] img           image to search for targets
 * @param[in] ref           reference target locations
 * @param[out] found        found targets 
 * @param[out] unmatched    indices of all ref that were not matched to a found target
 * @param[in] prange        search radius in pixels
 * @return                  number of found targets
 *
 * @pre for all i, 0 <= i < ref.size(), ref[i] is in frame
 * @post found[i] matches with ref[found[i].num]
 */
template<typename T>
int identify_targets_local(const cv::Mat& img, 
        const std::vector<cv::Point_<typename T::data_type>>& ref, 
        std::vector<T>& found, std::vector<int>& unmatched, unsigned int prange);

} /* end namespace upsp */

#include "../lib/registration.ipp"

#endif /* UFML_REGISTRATION_H_ */
