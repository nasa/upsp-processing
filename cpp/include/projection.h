/** @file
 *  @brief  Project between Image Frame and Model
 *  @date   July 27, 2017
 *  @author jmpowel2
 */

#ifndef UFML_PROJECTION_H_
#define UFML_PROJECTION_H_

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>

#include <Eigen/Sparse>
#include <opencv2/opencv.hpp>
#include <set>
#include <stack>
#include <vector>
#include <queue>

#include "data_structs.h"
#include "models.h"
#include "utils/general_utils.h"
#include "utils/cv_extras.h"

#include "CameraCal.h"

namespace upsp {

/** Return true if the pt is included in the image defined by @a sz */
bool contains(const cv::Size& sz, const cv::Point2i& pt);

/** Functor for checking if a model node is occluded by other faces
 * in the model 
 */
template<typename Tr>
struct OccludedPredicate {

    typedef typename Tr::Elem           model_node; 
    typedef typename Tr::model_idx      model_idx;
    typedef typename Tr::Node           tree_node;
    typedef typename Tr::Model          Model;
    typedef typename Tr::Model::Face    Face;
    typedef typename Tr::FP             FP;

    /** Initialize the tree structure and tolerance */
    OccludedPredicate(Tr& tree, FP tol = 0.0);

    /** Binary operator returns true if the node is occluded from the incoming ray
     * 
     * @param[in] n     node that is affected by @a r
     * @param[in] r     ray 
     * @return          true if distance from @a r.origin to n.get_position() minus
     *                  tol_ is greater than the distance from @a r.origin to 
     *                  any triangle in the model not adjacent to @a n
     * 
     * @pre @a r should intersect @a n
     */
    bool operator()(const model_node& n, const Ray<FP>& r) const;

    /***************************************************************/

    Tr& tree_;
    FP tol_;
};

/** Functor for checking if a model node is at an oblique angle to a ray */
template<typename Model>
struct ObliquePredicate {

    typedef typename Model::Node        model_node;
    typedef typename Model::data_type   FP;

    /** Initialize the model and the maximum allowable angle */
    ObliquePredicate(Model& model, FP max_ang=70.0);

    /** Binary operator determines if a ray intersects a node at an oblique angle
     *
     * @param[in] n     model node
     * @param[in] r     ray
     * @return          true if @a r intersects @a n at angle > max_ang_, else false
     *
     * @pre @a r intersects @a n
     */
    bool operator()(const model_node& n, const Ray<FP>& r) const;

    /***************************************************************/

    FP max_ang_;
    Model& model_;
};

/** Combines two functors with binary operators expecting a model node and ray */
template<typename P1, typename P2>
struct ProjectionPredicate {

    /** Load the two predicates */
    ProjectionPredicate(const P1& p1, const P2& p2);

    /** Return true if either of the two predicates returns true, else false */
    bool operator()(typename P1::model_node& n, const Ray<typename P1::FP>& r) const;

    /***************************************************************/

    P1 p1_;
    P2 p2_;

};

/** Functor to check if a model node's x-value is greater than some value */
template<typename Model>
struct GreaterXPredicate {
    typedef typename Model::Node model_node;
    typedef typename Model::data_type FP;

    /** Load in the maximum allowable x location */
    GreaterXPredicate(FP x_max_in);

    /** Return true if the position of the node is greater than the maximum
     *  allowable x value 
     */
    bool operator()(const model_node& n, const Ray<FP>& r) const;
    
    /***************************************************************/

    FP x_max;
};

/** Functor to check if a point is visible from a camera */
template<typename Model, typename Pred>
struct VisiblePredicate : std::unary_function<cv::Point3_<typename Model::data_type>, bool> {

    typedef typename Model::data_type FP;
    typedef typename Model::Node model_node;

    /** Load the camera calibration, octree containing the model, and any 
     *  additional predicates that should be satisfied
     */
    VisiblePredicate(const CameraCal& cal, const Octree<Model,model_node>& tree, 
            Pred pred);

    /** 
     * @return      true if the point is visible from the camera, else false
     */
    bool operator()(cv::Point3_<FP> pt) const;
    
    /***************************************************************/

    const CameraCal& cal;
    cv::Size sz;
    cv::Point3_<FP> cam_center;
    const Octree<Model,model_node>& tree;
    Pred pred;

};

/** Functor to check if a Node is a non-data node */
template<typename Model>
struct NonDataNodePredicate {
    typedef typename Model::Node model_node;
    typedef typename Model::data_type FP;

    /** Return true if the Node does not hold data */
    bool operator()(const model_node& n) const { return !(n.is_datanode()); }

    /** Return true if the Node does not hold data */
    bool operator()(const model_node& n, const Ray<FP>& r) const { 
        return this->operator()(n);
    }
};

/** Helper function to create a ProjectionPredicate */
template<typename P1, typename P2>
ProjectionPredicate<P1,P2> make_combined_predicate(const P1& p1, const P2& p2);

/** Helper function to create a VisiblePredicate */
template<typename Model, typename Pred>
VisiblePredicate<Model, Pred> make_visible_predicate(const CameraCal& cal, 
        const Octree<Model,typename Model::Node>& tree, Pred pred);

/** Selects the Camera with the best view of the node
 */
template<typename T>
struct BestView : std::unary_function<std::vector<T>, std::vector<T>> {

    /** Takes in angles, and returns weights of each camera
      *
      * @param[in] angles  = angles in degrees between cameras and the node
      * @return              weights {0,1} for each camera (@a angles.size()) 
      *
      * @pre  @a angles.size() > 0
      * @post @return.size() == @a angles.size()
      * @post for 0 <= i < @a angles.size(), @return[i] == 0, unless 
      *       i is the first camera with @a angles[i] == max(@a angles)
      */
    std::vector<T> operator()(const std::vector<T>& angles) const;

};

/** Averages all cameras with a view of the node based on the
 *  angle between the camera and the node
 */
template<typename T>
struct AverageViews : std::unary_function<std::vector<T>, std::vector<T>> {

    /** Takes in angles, and returns weights of each camera
      *
      * @param[in] angles  = angles in degrees between cameras and the node
      * @return              weights [0,1] for each camera (@a angles.size()) 
      *
      * @pre  @a angles.size() > 0
      * @post @return.size() == @a angles.size()
      * @post for 0 <= i < @a angles.size(), @return[i] = @a angles[i] / sum(@a angles)
      */
    std::vector<T> operator()(const std::vector<T>& angles) const;

};

/** Determine if a ray interects a bounding box
 * Algorithm from A. Williams with addition of edge handling (NaN)
 *
 * @param[in] bb    bounding box
 * @param[in] r     ray
 * @return          true if the ray intersects the bounding box, else false
 * 
 * @post    returns true if ray passes through [,) range of box (disclude max edge)
 *          will return true if ray origin is inside of @a bb
 */
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb, const Ray<FP>& r);

/** Determine the parametric time at which a ray strikes a bounding box
 *
 * @param[in] bb    bounding box
 * @param[in] r     ray
 * @return          time at which the ray first strikes the bounding box 
 *                  using pos = r.origin + r.dir*t
 *                  if it does strike the box, otherwise return < -1.0
 */
template<typename FP>
FP intersect_time(const BoundingBox_<cv::Point3_<FP>>& bb, const Ray<FP>& r);

/** Determine if a ray intersects a convex polyhedron
 *
 * @param[in] poly  convex polyhedron
 * @param[in] r     ray
 * @return          time at which the ray first strikes the polyhedron
 *                  using pos = r.origin + r.dir*t
 *                  if it does not strike the polyhedron return < -1.0
 *
 * @pre @a poly must contain all planes and vertices
 * @pre if @a poly.tris.size() == 0, will use triangulation in poly
 */
template<typename FP, size_t S>
FP intersect_time(const Polyhedron<S,FP>& poly, const Ray<FP>& r);

/** Determine if a ray intersects a triangle
 * Moller-Trumbore
 *
 * @param[in] tri   triangle
 * @param[in] r     incoming ray
 * @return          distance from ray origin to the intersection point if intersect, 
 *                  else < 0.0
 */
template<typename FP>
FP intersects(const Triangle<FP>& tri, const Ray<FP>& r);

/** Trace a ray through an octree and find all leaf nodes it intersects
 * Simple octree search algorithm
 *
 * @param[in] tree  tree data structure
 * @param[in] r     ray to trace
 * @param[in,out] n vector to be filled with all leaf nodes intersected
 *
 * @post    @a n will contain leaf nodes that contain the ray origin or are crossed by the ray
 */
template<typename Model>
void simple_ray_trace(const Octree<Model,typename Model::Node>& tree, const Ray<typename Model::data_type>& r, 
        std::vector<typename Octree<Model,typename Model::Node>::Node*>& n);

/** Trace a ray through an octree and find all leaf nodes it intersects
 * Revelles algorithm, modified to allow ray to begin inside of root node
 *
 * @param[in] tree  tree data structure
 * @param[in] ray   ray to trace
 * @param[in,out] n vector to be filled with all leaf nodes intersected
 *
 * @pre     octree octants are ordered as a grid with Z the fastest dimension, then Y,X
 * @post    n contains all leaf nodes for which the ray intersects with [,) bounds on x,y,z
 *          will contain the leaf containing the origin if applicable
 */
template<typename Model>
void ray_trace(const Octree<Model,typename Model::Node>& tree, Ray<typename Model::data_type> r, 
        std::vector<typename Octree<Model,typename Model::Node>::Node*>& n);

/** Find all nodes that are contained within a convex polyhedron
 *
 * @param[in] tree  tree data structure
 * @param[in] poly  polyhedron
 *
 */
template<typename Model, typename Poly>
std::set<typename Model::node_idx> find_nodes(const Octree<Model,typename Model::Node>& tree, 
        const Poly& poly);

/** Create the projection matrix for mapping image data onto a model surface
 *
 * @param[in] model     model that will projected onto
 * @param[in] cal       camera calibration mapping this frame to the model
 * @param[in,out] smat  the sparse projection matrix will be written here
 * @param[in] pred      functor that takes as input a model node and ray and returns
 *                      true if the node should be neglected
 * @post    smat will have shape #rows = #nodes, #cols = #pixels 
 */
template<typename Model, typename Pred, typename FP>
void create_projection_mat(const Model& model, CameraCal& cal, 
        Eigen::SparseMatrix<FP,Eigen::RowMajor>& smat, Pred pred); 

/** Identify which nodes are not filled by the sparse matrix
 *
 * @param[in] smat      sparse projection matrix
 * @param[out] skipped  vector containing all rows of smat without non-zero entries
 *
 */
template<typename T>
void identify_skipped_nodes(Eigen::SparseMatrix<T, Eigen::RowMajor>& smat, 
        std::vector<unsigned int>& skipped);

/** Identify which nodes are not filled by any of the sparse matrices
 *
 * @param[in] smats     sparse projection matrices
 * @param[out] skipped  vector containing all rows where every smat 
 *                      is without non-zero entries
 *
 * @pre for 1 <= i < smats.size() , smats[i].rows() == smats[0].rows()
 */
template<typename T>
void identify_skipped_nodes(std::vector<Eigen::SparseMatrix<T, Eigen::RowMajor>>& smat, 
        std::vector<unsigned int>& skipped);

/** Project a frame onto a model using a given mapping
 * 
 * @param[in] smat      sparse matrix containing the mapping from pixel to model node
 * @param[in] frame     frame with data to map onto model
 * @param[out] output   vector put the model node data
 *
 * @pre frame must have continuous memory
 */
template<typename T>
void project_frame(const Eigen::SparseMatrix<T, Eigen::RowMajor>& smat,
        const cv::Mat& frame, std::vector<T>& output);

/** Update projection matrices by the weight of that camera in the combined solution
 * @param[in] model         model associated with the solutions
 * @param[in] centers       centers of each of the cameras
 * @param[in,out] projs     projection matrices for each of the cameras
 * @param[in] weighter      unary operator, given a vector of angles between
 *                          the camera and a node, returns weights for each
 *                          camera
 * 
 * @pre model.size() == centers.size()
 */
template<typename T, typename FP, typename Op>
void adjust_projection_for_weights(T& model, std::vector<cv::Point3d>& centers, 
        std::vector<Eigen::SparseMatrix<FP, Eigen::RowMajor>>& projs,
        const Op& weighter);

/** Approximate the diameters of targets in pixels
 *   approximation is best in areas where curvature is small relative to target size
 *   it is exact for a flat surface 
 *
 * @param[in] tree      octree containing the model 
 * @param[in] cal       camera calibration
 * @param[in] targs     targets located on the model
 * @param[out] diams    diameters of the targets in pixels
 *
 * @pre for all 0 <= i < @a targs.size(), 
 *      @a targs[i].uv == @a cal.map_point_to_image( @a targs[i].xyz)
 * @post for all 0 <= i < @a targs.size(), if (!contains(@a cal.get_size(), @a targs[i].uv)) 
 *      then @a diams[i] == 0
 */
template<typename Model, typename T>
void get_target_diameters(const Octree<Model,typename Model::Node>& model, const CameraCal& cal,
        const std::vector<T>& targs, std::vector<typename T::data_type>& diams);

/** Write the projection matrix to a file */
template<typename T>
void write_projection_matrix(std::string filename, Eigen::SparseMatrix<T, Eigen::RowMajor>& smat);

/** Read the projection matrix from a file */
template<typename T>
void read_projection_matrix(std::string filename, Eigen::SparseMatrix<T, Eigen::RowMajor>& smat);

} /* end namespace upsp */

#include "../lib/projection.ipp"

#endif /* UFML_PROJECTION_H_ */
