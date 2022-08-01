/** @file
 *  @brief  Patch over Image Targets
 *  @date   August 22, 2017
 *  @author jmpowel2
 */

#ifndef UFML_PATCHES_H_
#define UFML_PATCHES_H_

#include <boost/iterator/counting_iterator.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <list>
#include <queue>
#include <vector>

#include "data_structs.h"
#include "projection.h"

#include "utils/cv_extras.h"

namespace upsp {

/** Patch over targets, using initial locations */
struct PatchTargets : std::unary_function<cv::Mat, cv::Mat> {

    /** Initialize the list of targets */
    PatchTargets(const std::vector<Target>& targs);

    /** Patch over all targets and return patched image */
    cv::Mat operator()(cv::Mat inp) const;

    /***************************************************************/

    const std::vector<Target>& targs_;
};

/** Patch over clusters of targets */
template<typename FP>
struct PatchClusters : std::unary_function<cv::Mat, cv::Mat> {

    /** Create 2D polynomials that can be used to patch each cluster
     *
     * @param[in] clusters  vector of clusters containing targets, each 
     *                      cluster is patched
     *                      as a unit
     * @param[in] size_in   size of the image frames
     * @param[in] boundary_thickness    thickness of the cluster boundary in pixels
     * @param[in] buffer_thickness      number of pixels between edge of 
     *                                  cluster and boundary
     *
     * @pre for all 0 <= i < clusters.size(), clusters[i].size() >= 1
     */
    PatchClusters(const std::vector<std::vector<Target_<FP>>>& clusters, cv::Size size_in, 
            unsigned int boundary_thickness_in, unsigned int buffer_thickness_in);

    /** Remove nodes below threshold from list of boundary points
     *
     * @param[in] ref       reference frame
     * @param[in] thresh    threshold
     * @param[in] offset    distance between threshold and nearest valid 
     *                      boundary point (pixels)
     */
    template<typename T>
    void threshold_bounds(cv::Mat_<T> ref, unsigned int thresh, unsigned int offset);

    /** Patch over all clusters in the frame
     *
     * @param[in] inp   input frame
     * @return          patched frame
     */
    cv::Mat operator()(cv::Mat inp) const;

    /***************************************************************/

    cv::Size size;
    unsigned int boundary_thickness;
    unsigned int buffer_thickness;

    std::vector<std::vector<unsigned int>> bounds_x;
    std::vector<std::vector<unsigned int>> bounds_y;
    std::vector<std::vector<unsigned int>> internal_x;
    std::vector<std::vector<unsigned int>> internal_y;
};

/** Fit a 2D polynomial to data
 *
 * @param[in] x         x position
 * @param[in] y         y position
 * @param[in] z         value at each (x,y) position
 * @param[in,out] poly  output polynomial coefficients
 * @param[in] degree    degree of polynomial to fit
 *
 * @pre @a x.size() == @a y.size() == @a z.size()
 * @pre @a x.size() >= (degree + 2) * (degree + 1) / 2
 */
template<typename I, typename T>
void polyfit2D(const std::vector<I>& x, const std::vector<I>& y, 
        const std::vector<T>& z, std::vector<T>& poly, unsigned int degree);

/** Evaluate a 2D polynomial at (x,y) coordinates
 *
 * @param[in] x     x position
 * @param[in] y     y position
 * @param[in] poly  polynomial coefficients
 * @param[in,out] z result of applysing the polynomial to given (x,y) positions
 *
 * @pre @a x.size() == @a y.size()
 * @pre @a poly is a valid vector of polynomial coefficients
 */ 
template<typename I, typename T>
void polyval2D(const std::vector<I>& x, const std::vector<I>&y, 
        const std::vector<T>& poly, std::vector<T>& z);

/** Fill in each gap in data (roughly circular) by polynomial interpolation
 *
 * @param[in,out] img   image for which to fill gaps
 * @param[in] targs     targets where there are data gaps
 * @param[in] bound_pts number of points around the gaps to consider during 
 *                      interpolation
 *
 * @pre @a targs must include both the origin of the gaps and the diameter
 * @pre all @a targs must be completely within @a img after adding bound_pts
 * @post @a img will be type CV_64F
 */
void fill_data_gaps(cv::Mat& img, const std::vector<Target>& targs, int bound_pts=2);

/** Sort targets into clusters if they are within a certain distance
 *   uses a brute force approach
 *
 * @param[in] targs     targets to cluster
 * @param[out] clusters vector of clusters of targets
 * @param[in] bound_pts minimum boundary points between target boundaries
 */
template<typename T>
void cluster_points(const std::vector<T>& targs, std::vector<std::vector<T>>& clusters, int bound_pts=4);

/** Define a bounding box around a target
 *
 * @param[in] target    target with center and diameter
 * @param[out] t_min    min x/y boundary of square
 * @parma[out] t_max    max x/y boundary of square
 */
template<typename T>
void get_target_boundary(const T& targ, cv::Point2i& t_min, cv::Point2i& t_max);

/** Define boundary and internal points for a target
 *
 * @param[in] target    target with center and diameter
 * @param[out] internal all points internal to the target
 * @param[out] bounds   all points that make up the boundary of the target
 * @param[in] bound_pts thickness of the boundary
 * @param[in] buffer    pixels between the edge of targets and the boundary
 */
template<typename T>
void get_target_boundary(const T& targ, std::vector<cv::Point2i>& internal,
        std::vector<cv::Point2i>& bounds, unsigned int bound_pts=2, unsigned int buffer=0);

/** Define the bounds of a cluster and return the interior and boundary indices
 *
 * @param[in] targs     targets in a cluster
 * @param[out] internal all points in the image that are inside the cluster
 * @param[out] bounds   all points in the image that make up the boundary of the cluster
 * @param[in] bound_pts thickness of the boundary
 * @param[in] buffer    pixels between the edge of targets and the boundary
 *
 * @pre @a targs must include both the center and diameter of all targets
 */
template<typename T>
void get_cluster_boundary(const std::vector<T>& targs, std::vector<cv::Point2i>& internal, std::vector<cv::Point2i>& bounds, unsigned int bound_pts=2, unsigned int buffer=0);

} /* end namespace upsp */

#include "../lib/patches.ipp"

#endif /* UFML_PATCHES_H_ */
