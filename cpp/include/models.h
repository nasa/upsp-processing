/** @file
 *  @brief  Operations on Grid Models
 *  @date   July 25, 2017
 *  @author jmpowel2
 */

#ifndef UFML_MODELS_H_
#define UFML_MODELS_H_

#include <cmath>
#include <Eigen/Dense>
#include <functional>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <vector>

#include "data_structs.h"
#include "utils/general_utils.h"
#include "utils/cv_extras.h"
#include "projection.h"
#include "triangles.h"

namespace upsp {

/** Structure always returns true, unary operator */
template<typename N>
struct AlwaysTrue {
    AlwaysTrue() {}
    /** Return true */
    bool operator()(N) { return true;}
};

/** Create a minimal BoundingBox needed to enclose all points in range
 *
 * @tparam  InputIterator   must have value_type of cv::Point3_<>
 * @param[in] first         InputIterator to first node of interest
 * @param[in] last          InputIterator 1 past the last node of interest
 * @param[in] buffer        distance to add in each direction
 * @return                  BoundingBox needed to enclose the grid points 
 * @post                    if range is empty, BoundingBox will default to no range
 */
template<typename InputIterator>
BoundingBox_<typename InputIterator::value_type> get_extent(InputIterator first, 
        InputIterator last, float buffer = 0.0);

/** Get the center of all grid points in range and a half width needed to enclose them all
 *
 * @tparam  InputIterator   must have value_type of cv::Point3_<>
 * @param[in] first         InputIterator to first node of interest
 * @param[in] last          InputIterator 1 past the last node of interest
 * @param[in,out] center    midpoint between max/min in each direction
 * @param[in] add_buffer    if true, add 1% cushion on each end
 * @return                  half width needed to enclose points (+1% cushion)
 */
template<typename InputIterator>
typename InputIterator::value_type::value_type get_extent(InputIterator first, 
        InputIterator last, typename InputIterator::value_type& center,
        bool add_buffer = true);

/** Create a minimal BoundingBox2D needed to enclose all points in range
 *
 * @tparam  InputIterator   must have value_type cv::Point2_<>
 * @param[in] first         InputIterator to first node of interest
 * @param[in] last          InputIterator 1 past the last node of interest
 * @param[in] add_buffer    if true, add 1% cushion on each end
 * @return                  BoundingBox needed to enclose the grid points 
 * @post                    if range is empty, BoundingBox will default to no range
 */
template<typename InputIterator>
BoundingBox_<typename InputIterator::value_type> get_extent2D(InputIterator first, 
        InputIterator last, bool add_buffer = true); 

/** Compute the normal direction of the triangle
 *
 * @param[in] tri   the triangle of interest
 * @return          unit vector in normal direction
 *
 * @post    normal direction assumes order nodes[0],nodes[1],nodes[2] and right hand rule
 */
template<typename FP>
cv::Point3_<FP> normal(const Triangle<FP>& tri);

/** Compute the normal direction of the polygon
 *
 * @param[in] poly  polygon
 * @return          unit vector in normal direction
 *
 * @pre nodes must be ordered
 */
template<typename FP>
cv::Point3_<FP> normal(const Polygon<FP>& poly);

/** Get the area of a triangle
 * Heron's formula 
 * @param[in] tri   triangle
 * @return          area of @a tri
 */
template<typename FP>
FP area(const Triangle<FP>& tri);

/** Checks if two axially aligned bounding boxes overlap
 *
 * @param[in] bb1   first bounding box
 * @param[in] bb2   second bounding box
 * @return          true if the boxes overlap (including edges), else false
 */
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb1, 
        const BoundingBox_<cv::Point3_<FP>>& bb2);

/** Check if a point is within an axially aligned bounding box
 *
 * @param[in] bb    bounding box
 * @param[in] pt    3D point
 * @return          true if pt is within bb [,) , else false
 */
template<typename FP1, typename FP2>
bool intersects(const BoundingBox_<cv::Point3_<FP1>>& bb, const cv::Point3_<FP2>& pt);

/** Check if a plane intersects an axially aligned bounding box
 * 
 * @param[in] bb    bounding box
 * @param[in] pl    plane
 * @return          true if intersection, else false
 */
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb, const Plane<FP>& pl);

/** Get the shortest distance between a bounding box and a point
 *
 * @param[in] bb    bounding box
 * @param[in] pt    3D point
 * @return          distance between bounding box and point (0 if @a pt is in @a bb)
 */
template<typename FP>
FP distance(const BoundingBox_<cv::Point3_<FP>>& bb, const cv::Point3_<FP>& pt);

/** Combine multiple camera solutions into a single solution on a model
 * solutions are weighted by the angle between the node normal and the incoming
 * camera ray
 *
 * @param[in] model     the model upon which each solution is given
 * @param[in] sols      vector of solutions
 * @param[in] centers   the position of each camera's center
 * @param[in,out] comb  the combined solution
 * 
 * @pre if sols[i][j] == NaN, then camera i has no valid solution for point j
 *      and it is entirely neglected
 * @pre for every i in [0,sols.length()) the length of the array (or vector) 
 *      == model.size()
 * @pre centers.length() == sols.length()
 * @post comb.length() == model.size()
 */
template<typename T, typename FP>
void combine_solutions(T& model, std::vector<std::vector<FP>>& sols, 
        std::vector<cv::Point3d>& centers, std::vector<FP>& comb);

/** Find the model Node approximately closest to a given point
 *
 * @param[in] tree      octree containing the model of interest
 * @param[in] pt        the point in the same 3D space as the @a model
 * @return              the node in the @a model that is closest (euclidean distance) from
 *                      @a pt
 *
 *  @pre root node in @a tree must contain @a pt
 */
template<typename M>
typename M::Node approx_nearest_node(const Octree<M,typename M::Node>& tree, 
        cv::Point3_<typename M::data_type> pt);

/** Find all model nodes that are within a tolerance of the given node
 *  and that satisfy the predicate
 *
 * @param[in] tree      octree containing the model of interest
 * @param[in] node      Node in the model
 * @param[in] tol       tolerance
 * @param[in] pred      predicate that a node must satisfy to be valid
 * @return              nodes in the model that are within @a tol of @a node
 *                      (euclidean distance) while satisfying @a pred
 *
 *  @pre root node in @a tree must contain @a node
 *  @post if no nodes in @a tree satisfy @a pred, then return invalid node
 *  @post will not return @a node as a neighbor
 */
template<typename M, typename Pred = AlwaysTrue<typename M::Node>>
std::vector<typename M::Node> nearest_neighbors_tol(const Octree<M,typename M::Node>& tree, 
        typename M::Node node, float tol, Pred pred = AlwaysTrue<typename M::Node>());

/** Find the model Node closest to a given node that satisfies the predicate
 *
 * @param[in] tree      octree containing the model of interest
 * @param[in] node      Node in the model
 * @param[in] pred      predicate that a node must satisfy to be valid
 * @return              the node in the model that is closest 
 *                      (euclidean distance) to @a node while satisfying @a pred
 *
 *  @pre root node in @a tree must contain @a node
 *  @post if no nodes in @a tree satisfy @a pred, then return invalid node
 *  @post will not return @a node as a neighbor
 */
template<typename M, typename Pred>
typename M::Node nearest_neighbor(const Octree<M,typename M::Node>& tree, typename M::Node node, 
        Pred pred = AlwaysTrue<typename M::Node>());

/** Find the k model Nodes closest to a given point
 *
 * @param[in] tree      octree containing the model of interest
 * @param[in] pt        point in the model coordinate system
 * @param[in] k         number of neighbors to find
 * @return              vector of up to k nodes (less if unavailable)
 *
 * @pre root node in @a tree must contain @a pt
 */
template<typename M>
std::vector<typename M::Node> nearest_k_neighbors(
        const Octree<M,typename M::Node>& tree, cv::Point3_<typename M::data_type> pt,
        unsigned int k);

/** Find the k model Nodes closest to a given point that satisfy the predicate
 *
 * @param[in] tree      octree containing the model of interest
 * @param[in] pt        point in the model coordinate system
 * @param[in] k         number of neighbors to find
 * @param[in] pred      predicate that a node must satisfy to be valid
 * @return              vector of up to k valid nodes (less if unavailable)
 *
 * @pre root node in @a tree must contain @a pt
 */
template<typename M, typename Pred>
std::vector<typename M::Node> nearest_k_neighbors(
        const Octree<M,typename M::Node>& tree, cv::Point3_<typename M::data_type> pt,
        unsigned int k, Pred pred);

/** Rotate 2D points about the positive z axis by some angle
 *
 * @tparam InputIterator    must have value type cv::Point2d
 * @param[in] first         iterator to the first position to rotate
 * @param[in] last          one past the last position to rotate
 * @param[in] angle         angle in radians to rotate points
 * @param[in] center        point about which to rotate
 */
template<typename InputIterator>
void rotate2D(InputIterator first, InputIterator last, double angle, cv::Point2d center);

/** Determine the shortest distance between a plane and a point
 *
 * @param[in] plane     plane
 * @param[in] pt        pt
 *
 * @post let n be the shortest vector from @a plane to @a pt, 
 *       if arccos(n*plane.normal) > 90deg, @return < 0, else @return >= 0
 */
template<typename FP>
FP shortest_distance(const Plane<FP>& plane, const cv::Point3_<FP>& pt);

/** Determine if a point is contained with a bounding box
 *
 * @param[in] bb        axis-aligned 3D bounding box
 * @param[in] pt        point (x,y,z)
 */
template<typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const cv::Point3_<FP>& pt);

/** Determine if a point is contained within a convex polyhedron
 *
 * @param[in] poly      convex polyhedron
 * @param[in] pt        point (x,y,z)
 *
 * @post if pt is on the boundary of the polyhedron, return true
 */
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const cv::Point3_<FP>& pt);

/** Determine if a bounding box is contained within a polyhedron
 *
 * @param[in] poly      convex polyhedron
 * @param[in] bb        axis-aligned 3D bounding box
 */
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const BoundingBox_<cv::Point3_<FP>>& bb);

/** Determine if a convex polyhedron is contained within a bounding box
 *
 * @param[in] bb        axis-aligned 3D bounding box
 * @param[in] poly      convex polyhedron
 */
template<size_t Size, typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const Polyhedron<Size,FP>& poly);

/** Determine if a convex polyhedron is contained within another convex polyhedron
 *
 * @param[in] poly_out     outer convex polyhedron
 * @param[in] poly_in      inner convex polyhedron
 */
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly_out, const Polyhedron<Size,FP>& poly_in);

/** Determine if a convex polyhedron contains a triangle
 * 
 * @param[in] poly      covex polyhedron
 * @param[in] tri       triangle
 * @param[in] tol       if > 0, use this tol, otherwise use poly tol
 */
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const Triangle<FP>& tri, FP tol=-1.0);

/** Determine if an axially aligned bounding box contains a triangle
 *
 * @param[in] bb        axially-aligned bounding box
 * @param[in] tri       triangle
 */
template<typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const Triangle<FP>& tri);

/** Determine if a convex polyhedron and bounding box intersect
 *
 * @param[in] poly      convex polyhedron
 * @param[in] bb        axis-aligned 3D bounding box
 *
 * @pre @a poly must have edges defined
 */
template<size_t Size, typename FP>
bool intersects(const Polyhedron<Size,FP>& poly, const BoundingBox_<cv::Point3_<FP>>& bb); 

/** Find all Model nodes within a convex polyhedron
 * only model nodes within the Octree are considered
 *
 * @param[in] poly      convex polyhedron
 * @param[in] tree      Octree containing Model nodes
 */
template<size_t Size, typename FP, typename M>
std::set<typename M::node_idx> nodes_within(const Polyhedron<Size,FP>& poly, 
                                        const Octree<M,typename M::Node>& tree);

/** Find all Model Faces that are contained within or intersect a convex polyhedron
 * only model faces within the Octree are considered
 *
 * @param[in] poly      convex polyhedron
 * @param[in] tree      Octree containing Model Faces
 */
template<size_t Size, typename FP, typename M>
std::set<typename M::face_idx> faces_within(const Polyhedron<Size,FP>& poly,
                                            const Octree<M,typename M::Face>& tree);

/** Find all Model Faces that may be contained within or intersect a convex polyhedron
 *  find all faces in Octree nodes that intersect or are contained within the poly
 *
 * @param[in] poly      convex polyhedron
 * @param[in] tree      Octree containing Model Faces
 */
template<size_t Size, typename FP, typename M>
std::set<typename M::face_idx> approx_faces_within(const Polyhedron<Size,FP>& poly,
                                                   const Octree<M,typename M::Face>& tree);

/** Find the point where a plane intersects an edge (defined by 2 points)
 *
 * @param[in] pl        plane
 * @param[in] pt1       first point
 * @param[in] pt2       second point
 *
 * @pre the plane must intersect the line between the two points
 */
template<typename FP>
cv::Point3_<FP> intersect(const Plane<FP>& pl, const cv::Point3_<FP>& pt1, 
                          const cv::Point3_<FP>& pt2);

/** Find the distance along an edge where a plane intersects that edge
 *
 * @param[in] pl        plane
 * @param[in] pt1       first point
 * @param[in] pt2       second point
 * @return  fraction of distance along the edge [pt1,pt2] where the 
 *          intersection occurs value in range [0,1]
 * 
 * @pre the plane must intersect the line between the two points
 */
template<typename FP>
FP intersect_dist(const Plane<FP>& pl, const cv::Point3_<FP>& pt1, 
                  const cv::Point3_<FP>& pt2);

/** Check that the three planes intersect into a point
 *
 * @param[in] pl1       first plane
 * @param[in] pl1       second plane
 * @param[in] pl1       third plane
 * @param[in] threshold minimum value of determinants that qualifies as non-zero   
 */
template<typename FP>
bool intersects(const Plane<FP>& pl1, const Plane<FP>& pl2, const Plane<FP>& pl3,
                float threshold = 0.00001);

/** Find the point of intersection between three planes
 *
 * @param[in] pl1       first plane
 * @param[in] pl2       second plane
 * @param[in] pl3       third plane
 *
 * @pre intersects(pl1,pl2,pl3)
 */
template<typename FP>
cv::Point3_<FP> intersect(const Plane<FP>& pl1, const Plane<FP>& pl2, 
                          const Plane<FP>& pl3);

/** Split a triangle by a plane
 *
 * @param[in] tri       triangle
 * @param[in] pl        plane
 * @return all new triangles with information about the relationship to the
 *         original triangle @a tri
 * 
 * @post if triangle is not split by plane, @return.size() == 0
 * @post if triangle is split by plane: if any node in @a tri is on @a pl, then 
 *                                      @return.size() == 2, else @return.size() == 3
 * @post if triangle is split by plane: all new nodes in @return (not in @a tri),
 *                                      will be on the edges of @a tri
 */
template<typename FP>
std::vector<SplitTri<FP>> split(const Triangle<FP>& tri, const Plane<FP>& pl);

/** Determine the shortest distance from a point (within a convex polyhedron)
 *  to a face of the polyhedron
 *
 * @param[in] poly      polyhedron
 * @param[in] pt        point in the convex polyhedron
 *
 * @pre contains(poly, pt)
 * @post @return >= 0.0
 */
template<typename FP, size_t S>
FP shortest_distance(const Polyhedron<S,FP>& poly, const cv::Point3_<FP>& pt);

/** Determine which planes a point sits on
 *  does not need to be within the polyhedron
 *
 * @param[in] poly      polyhedron
 * @param[in] pt        point
 * @return              indices of the planes point lies on within the poly.tol
 *                      will be empty if it does not lie on any planes within the
 *                      the tolerance
 */
template<typename FP, size_t S>
std::vector<unsigned int> find_intersecting_planes(const Polyhedron<S,FP>& poly,
        const cv::Point3_<FP>& pt);

/** Convert a polyhedron into a set of triangles
 *
 * @param[in] poly      convex polyhedron
 * @return              vector of triangles
 *
 * @pre poly must contain vertices and planes
 */
template<typename FP, size_t S>
std::vector<Triangle<FP>> convert(const Polyhedron<S,FP>& poly);

/** Determine if a triangle is split by a polyhedron
 *
 * @param[in] tri       triangle
 * @param[in] poly      polyhedron
 */
template<typename FP, size_t S>
bool intersects(const Triangle<FP>& tri, const Polyhedron<S,FP>& poly);

/** Determine if a triangle is split by a bounding box
 * could be improved for this specific case, for now using poly
 * 
 * @param[in] tri       triangle
 * @param[in] bb        axially-aligned bounding box
 *
 */
template<typename FP>
bool intersects(const Triangle<FP>& tri, const BoundingBox_<cv::Point3_<FP>>& bb);

/** Split a triangle by all planes in a polyhedron
 *  utilizes split(tri, poly.planes[i]) to successively split triangle
 *
 * @param[in] tri       triangle
 * @param[in] poly      polyhedron
 * @return  all new triangles with information about the relationship to the
 *          original triangle @a tri
 * 
 * @post if triangle is not split by any planes in the polyhedron, @return.size() == 0
 */
template<typename FP, size_t S>
std::vector<SplitTri<FP>> split(const Triangle<FP>& tri, const Polyhedron<S,FP>& poly);

/** Create a minimal bounding box for a convex polyhedron
 *
 * @param[in] poly      convex polyhedron
 * 
 * @pre @a poly must have vertices defined
 */
template<typename FP, size_t S>
BoundingBox_<cv::Point3_<FP>> minimal_bound(const Polyhedron<S,FP>& poly);

} /* end namespace upsp */

#include "../lib/models.ipp"

#endif /* UFML_MODELS_H_ */
