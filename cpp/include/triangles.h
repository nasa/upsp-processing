/** @file
 *  @brief  Handles Triangles, triangulation, convex hulls
 *  @date   May 8, 2019
 *  @author jmpowel2
 */

#ifndef UFML_TRIANGLES_H_
#define UFML_TRIANGLES_H_

#include <cmath>
#include <functional>
#include <opencv2/opencv.hpp>
#include <vector>

#include "utils/cv_extras.h"
#include "utils/general_utils.h"
#include "data_structs.h"
#include "models.h"

namespace upsp {

/** Triangulate a convex polygon
 *
 * @param[in] poly     convex polygon
 * @return               vector of triangles
 */
template<typename FP>
std::vector<Triangle<FP>> triangulate(const Polygon<FP>& poly);

/** Convert convex polyhedron to a set of polygons
 *  relies on polyhedron tolerance to find vertices on planes
 *
 * @param[in] poly      convex polyhedron
 * @return              vector of polygons
 */
template<typename FP, size_t S>
std::vector<Polygon<FP>> convert_polygons(const Polyhedron<S,FP>& poly);

/** Create a minimal bounding box around a triangle
 *
 * @param[in] tri       triangle
 */
template<typename FP>
BoundingBox_<cv::Point3_<FP>> minimal_bound(const Triangle<FP>& tri);

} /* end namespace upsp */

#include "../lib/triangles.ipp"

#endif /* UFML_TRIANGLES_H_ */
