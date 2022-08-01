/** @file
 *  @brief Face Grid Model
 *  @date  February 6, 2019
 *  @author jmpowel2
 */

#ifndef UFML_TRIGRID_H
#define UFML_TRIGRID_H

#include <algorithm>
#include <array>
#include <boost/iterator/counting_iterator.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <iterator>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "data_structs.h"
#include "models.h"
#include "utils/general_utils.h"
#include "utils/pspKdtree.h"

#include "Model.h"

namespace upsp {

/** Triangulated Grid Model
 *  can dynamically modify the grid:
 *      can add/remove faces
 *      can modify face component IDs
 *      can add nodes
 * 
 * @tparam FP   floating point (either single or double)
 */
template<typename FP>
class TriModel_ : public Model<FP>, private equality_comparable<TriModel_<FP>> {
public:

    typedef FP data_type;
    
    typedef unsigned int node_idx;
    typedef unsigned int face_idx;
    typedef CircularArray<node_idx,2> simple_edge;
    typedef CircularArray<node_idx,3> simple_tri;
    typedef int component;

    using Model<FP>::is_datanode;
    using Model<FP>::set_node_nondata;
    using Model<FP>::using_nodes;
    using Model<FP>::is_dataface;
    using Model<FP>::set_face_nondata;

    class Node;   
    class Edge;
    class Face;
    class NodeIterator;
    class FaceIterator;
    class CompFaceIterator;
    class AdjNodeIterator;
    class NAdjFaceIterator;
    class AdjFaceIterator;
    class CompAdjFaceIterator;

    /** Create an empty Face Grid Model */
    TriModel_(): n_tris_(0) {}

    /** Load a *.tri file and organize the data 
     *
     * @param[in] grid_file     name of the *.tri or *.i.tri file
     * @param[in] intersect     if true, consolidate all overlapping points
     */
    TriModel_(std::string grid_file, bool intersect=true);

    /** Convert a P3DModel to a TriModel 
     *
     * @param[in] p3d       plot3d model
     * @param[in] intersect if true, consolidate all overlapping points
     *                      will respect tolerance from @a p3d
     *
     * @post if intersect=false, p3d.size() == size()
     * @post if intersect=false, for all 0 <= x < p3d.size(), 
     *                           p3d.get_position(x) == get_position(x)
     * @post p3d set normals will not be maintained
     */
    TriModel_(const P3DModel_<FP>& p3d, bool intersect);

    /** Convert an UnstructuredGrid to a TriModel
     *
     * @param[in] ugrid      unstructured grid
     * @param[in] intersect  if true, consolidate all overlapping points
     */
    TriModel_(const UnstructuredGrid<FP>& ugrid, bool intersect=false);

    /** Load a *.tri file
     *
     * @param[in] grid_file     name of the *.tri or *.i.tri file
     * @param[in] intersect     if true, consolidate all overlapping points
     */
    void load_grid(std::string grid_file, bool intersect=true);

    /** Load an UnstructuredGrid into the model
     *
     * @param[in] ugrid     unstructured grid
     * @param[in] intersect if true, consolidate all overlapping points
     */
    void load_grid(const UnstructuredGrid<FP>& ugrid, bool intersect=false);

    /** Extract triangulation that covers entire model surface
     */
    void extract_tris(std::vector<float>& tris, std::vector<int>& triNodes) const;

    /** Write out the grid to a *.tri file */
    void write_grid(std::string grid_file) const;

    /** Add a Node to the model */
    node_idx add_node(FP x, FP y, FP z);

    /** Add a Node to the model */
    node_idx add_node(const cv::Point3_<FP>& pt);

    /** Add a Face to the model 
     *
     * @return  index of the new face
     *
     * @pre n1 < number_of_nodes()
     * @pre n2 < number_of_nodes()
     * @pre n3 < number_of_nodes()
     * 
     * @post if has_components(), @return will be set to component c_id
     */
    face_idx add_face(node_idx n1, node_idx n2, node_idx n3, component c_id=-1);

    /** Remove a Face from the model
     *
     * @pre is_face(f)
     */
    void remove_face(face_idx f);

    /** Initialize all faces to a specified component id */
    void initialize_components(component comp_id=0);

    /** Set component for a face
     *
     * @pre has_components() = true
     */
    void set_component(face_idx f, component comp_id);

    /** Return a Node object */
    Node node(node_idx n) const;

    /** Return an Edge object */
    Edge edge(node_idx n1, node_idx n2);
    /** Return an Edge object */
    Edge edge(simple_edge e) const;
    /** Return a Face object 
     * 
     * @pre is_face(t)
     */
    Face face(face_idx t);
    /** Return a const Face object 
     *
     * @pre is_face(t)
     */
    const Face cface(face_idx t) const;

    /** Return a beginning iterator for looping through all Nodes */
    NodeIterator node_begin();
    /** Return an end iterator for looping through all Nodes */
    NodeIterator node_end();
    /** Return a const beginning iterator for looping through all Nodes */
    NodeIterator cnode_begin() const;
    /** Return a const end iterator for looping through all Nodes */
    NodeIterator cnode_end() const;

    /** Return a beginning iterator for looping through all Faces */
    FaceIterator face_begin();
    /** Return an end iterator for looping through all Faces */
    FaceIterator face_end();
    /** Return a const beginning iterator for looping through all Faces */
    FaceIterator cface_begin() const;
    /** Return a const end iterator for looping through all Faces */
    FaceIterator cface_end() const;

    /** Return a beginning iterator for looping through all Faces in a component */
    CompFaceIterator comp_face_begin(component comp);
    /** Return an end iterator for looping through all Faces in a component */
    CompFaceIterator comp_face_end(component comp);
    /** Return a const beginning iterator for looping through all Faces in a component */
    CompFaceIterator ccomp_face_begin(component comp) const;
    /** Return an const end iterator for looping through all Faces in a component */
    CompFaceIterator ccomp_face_end(component comp) const;
    

    /** Return a beginning iterator for looping through 
     *  all nodes adjacent to @v
     */
    AdjNodeIterator adj_node_begin(node_idx n);
    /** Return an end iterator for looping through 
     *  all nodes adjacent to @v
     */
    AdjNodeIterator adj_node_end(node_idx n);
    /** Return a const beginning iterator for looping through 
     *  all nodes adjacent to @v
     */
    AdjNodeIterator cadj_node_begin(node_idx n) const;
    /** Return a const end iterator for looping through 
     *  all nodes adjacent to @v
     */
    AdjNodeIterator cadj_node_end(node_idx n) const;

    /** Return a beginning iterator for looping through 
     *  all Faces adjacent to @v
     */
    NAdjFaceIterator nadj_face_begin(node_idx n);
    /** Return an end iterator for looping through 
     *  all Faces adjacent to @v
     */
    NAdjFaceIterator nadj_face_end(node_idx n);
    /** Return a const beginning iterator for looping through 
     *  all Faces adjacent to @v
     */
    NAdjFaceIterator cnadj_face_begin(node_idx n) const;
    /** Return a const end iterator for looping through 
     *  all Faces adjacent to @v
     */
    NAdjFaceIterator cnadj_face_end(node_idx n) const;

    /** Return a beginning iterator for looping through
     *  all Faces adjacent to @t
     *
     * @pre is_face(t)
     */
    AdjFaceIterator adj_face_begin(face_idx t);
    /** Return an end iterator for looping through
     *  all Faces adjacent to @t
     *
     * @pre is_face(t)
     */
    AdjFaceIterator adj_face_end(face_idx t);
    /** Return a const beginning iterator for looping through
     *  all Faces adjacent to @t
     *
     * @pre is_face(t)
     */
    AdjFaceIterator cadj_face_begin(face_idx t) const;
    /** Return a const end iterator for looping through
     *  all Faces adjacent to @t
     *
     * @pre is_face(t)
     */
    AdjFaceIterator cadj_face_end(face_idx t) const;

    /** Return a beginning iterator for looping through all 
     *  Faces adjacent to @t and with the same component
     *  id as @t
     *
     * @pre is_face(t)
     */
    CompAdjFaceIterator compadj_face_begin(face_idx t);
    /** Return an end iterator for looping through all 
     *  Faces adjacent to @t and with the same component
     *  id as @t
     *
     * @pre is_face(t)
     */
    CompAdjFaceIterator compadj_face_end(face_idx t);
    /** Return a const beginning iterator for looping through all 
     *  Faces adjacent to @t and with the same component
     *  id as @t
     *
     * @pre is_face(t)
     */
    CompAdjFaceIterator ccompadj_face_begin(face_idx t) const;
    /** Return a const end iterator for looping through all 
     *  Faces adjacent to @t and with the same component
     *  id as @t
     *
     * @pre is_face(t)
     */
    CompAdjFaceIterator ccompadj_face_end(face_idx t) const;

    /** Return true if the face_idx is a valid face */
    bool is_face(face_idx t) const { 
        if (t < valid_tri_.size()) {
            return valid_tri_[t];
        }
        return false;
    }

    /** Get the nodes of @t 
     *
     * @pre is_face(t) == true   
     */
    simple_tri face_nodes(face_idx t) const { return tris_[t]; }

    /** Get the location (x,y,z) of @n */
    cv::Point3_<FP> get_position(node_idx n) const { return {x_[n],y_[n],z_[n]}; }

    /** Check if the face defined by (@a n1, @a n2, @a n3) exists
     *  (must have the same outward normal for true)
     */
    bool is_face(node_idx n1, node_idx n2, node_idx n3);

    /** Return true if components are defined in this Model */
    bool has_components() const { return comps_.size() > 0; }

    /** Return the component id
     *
     * @pre has_components()
     * @pre is_face(t)
     */
    component get_component(face_idx t) { return f2c_[t]; }

    /** Return the number of faces in the model */
    unsigned int number_of_faces() const { return n_tris_; }

    /** Return the number of faces in the component 
     * @param comp_id   component id (1-based)
     * @post if !has_components(), @return 0
     */
    unsigned int number_of_faces(component comp_id) const;

    /** Return the number of nodes in the model */
    unsigned int size() const { return x_.size(); }

    /** Return the number of nodes in the model */
    unsigned int number_of_nodes() const { return x_.size(); }

    /** Return the number of components in the model */
    unsigned int number_of_components() const {
        if (!has_components()) return 1;
        return comps_.size();
    }

    /** Find the number of faces adjacent to @a t */
    unsigned int number_of_adj_faces(face_idx t) const;

    /** Find the number of faces adjacent to @a t
     *  and with the same component id as @a t
     */
    unsigned int number_of_adj_faces_comp(face_idx t) const;

    /** Check validity
     *  check that this is a valid model
     * 
     * For true, must satisfy:
     *  x_.size() == y_.size() == z_.size()
     *  if comps_.size() > 0: comps_.size() == tris_.size()
     *  for all tris_, node indices < x_.size()
     *  n2t_.size() == x_.size()
     *  tris_ and n2t_ are consistent
     */
    bool is_valid() const;

    /** Check for duplicate faces */
    bool has_duplicate_faces() const;

    /** Check for equality with another model 
     *  checks for equality in all nodes, triangles, and components
     *  neglects invalid triangles, so only order of valid triangles matters
     * 
     * @pre this->is_valid() == true
     * @pre tri_m.is_valid() == true
     * 
     */
    bool operator==(const TriModel_<FP>& tri_m) const;

    /** Return reference to the vector of node x locations */
    const std::vector<FP>& get_x() const { return x_; }
    /** Return reference to the vector of node y locations */
    const std::vector<FP>& get_y() const { return y_; }
    /** Return reference to the vector of node z locations */
    const std::vector<FP>& get_z() const { return z_; }

    /** TODO need to rethink how transformations + normals are handled **/

    const std::vector<cv::Point3_<FP> >& get_n() const { return normals_;}

    const kdtree* kdRoot(void) const {return root_;}

    private:
    void calcNormals();

    public:

    /** Split a face by a plane
     *
     * @param[in] t                 face to split
     * @param[in] pl                plane
     * @param[in] split_adj         if true, split adjacent triangles
     *                              to maintain connectivity
     * @return all new faces created by the split indexed by their old non-split face
     *         index
     * 
     * @pre is_face(@a t)
     * @post if triangle is not split by plane, @return.size() == 0
     * @post if triangle is split by plane: if node in @a t is on @a pl, then
     *                                      @return.size(@a t) == 2, else @return.size(@a t) == 3
     * @post if triangle is split by plane: all new nodes in @return (not in @a t),
     *                                      will be on the edges of @a t
     */
    FaceMap<face_idx> split(face_idx t, const Plane<FP>& pl, bool split_adj=true);

    /** Split a face by a polyhedron
     *
     * @param[in] t                 face to split
     * @param[in] pl                polyhedron
     * @param[in] split_adj         if true, split adjacent triangles
     *                              to maintain connectivity
     * @return all new faces created by the split indexed by their old non-split face
     *         index
     *
     * @pre is_face(@a t)
     * @post if triangle is not split by plane, @return.size() == 0
     * @post adjacent triangles will be split if their edge is split
     */
    template<size_t S>
    FaceMap<face_idx> split(face_idx t, const Polyhedron<S,FP>& poly, bool split_adj=true);

    /** Get a set of all faces that contain at least one of the given nodes 
     *
     * @param[in] nodes     set of valid node indices
     */
    std::set<face_idx> gather_faces(const std::set<node_idx>& nodes) const;

    class Node : private equality_comparable<Node> {
    public:
        /** Default constructor creates invalid Node */
        Node() : model_(nullptr) {}

        /** Return the number of adjacent Faces */
        unsigned int number_of_adj_faces() const;

        /** Return the index of this Node */
        node_idx get_nidx() const { return nidx_; }

        node_idx get_idx() const { return nidx_; }

        /** Return true if this is a valid Node */
        bool is_valid() const { return (model_ != nullptr); }

        /** Return true if this node holds data */
        bool is_datanode() const { return model_->is_datanode(nidx_); }

        /** Return the location (x,y,z) */
        cv::Point3_<FP> get_position() const { return model_->get_position(nidx_); }

        /** Return true if all adjacent faces have the same component id 
         *
         *  @post if model_->has_components() == false, return false
         *  @post if number_of_adj_faces() == 0, return false
         */
        bool has_primary_component() const;

        /** Return the primary component if it exists
         *  this is the component of its adjacents faces
         *
         * @pre has_primary_component()
         */
        component get_primary_component() const;

        /** Get a set of the components of all adjacent triangles */
        std::set<component> get_components() const;

        /** Get an approximate normal direction
         *
         * @return  the area weighted average of all adjacent face
         *          normal directions
         */
        cv::Point3_<FP> get_normal() const;

        /** Check if the Node is contained within a bounding box
         */
        bool is_contained(const BoundingBox_<cv::Point3_<FP>>& bb) const;

        /** Check if the Node is contained within a bounding box
         */
        bool is_intersected(const BoundingBox_<cv::Point3_<FP>>& bb) const {
            return is_contained(bb);
        }

        /** Return a beginning iterator for looping over 
         *  adjacent Faces
         */
        NAdjFaceIterator adj_face_begin() const { 
            return model_->nadj_face_begin(nidx_);
        }
        /** Return an end iterator for looping over
         *  adjacent Faces
         */
        NAdjFaceIterator adj_face_end() const { 
            return model_->nadj_face_end(nidx_); 
        }

        /** Compare Nodes for equality (must reference the same node) */
        bool operator==(const Node& n) const {
            return (model_ == n.model_) && (nidx_ == n.nidx_);
        }

    private:
        friend class TriModel_;
    
        /** Create a valid Node object */
        Node(const TriModel_* model, node_idx n);
    
        TriModel_<FP>* model_;
        node_idx nidx_;
    };

    class Edge {
    public:
        typedef FP value_type;

        /** Default constructor creates an invalid Edge object */
        Edge() : model_(nullptr) {}

        /** Get the index of the ith node in this edge
         *
         * @pre i < 2
         */
        node_idx node_index(unsigned int i) const {
            assert( i < 2 );
            return ns_[i];
        }

        /** Get the direction the Edge points (normalized) */
        cv::Point3_<FP> get_direction() const;

        /** Get the location of the ith node in this edge
         *
         * @pre i < 2
         */
        cv::Point3_<FP> get_position(unsigned int i) const {
            assert( i < 2);
            return model_->get_position(ns_[i]);
        }

        // TODO: cv::Point3_<FP> get_normal() const;
        // TODO: FP get_length() const;

    private:
        friend class TriModel_;

        /** Create a valid Edge object
         *
         * @param[in] model pointer to the model the edge is contained in
         * @param[in] n1    first node
         * @param[in] n2    second node
         */
        Edge(const TriModel_* model, node_idx n1, node_idx n2);

        /** Create a valid Edge object
         *
         * @param[in] model pointer to the model the edge is contained in
         * @param[in] e     simple_edge object for this Edge
         */
        Edge(const TriModel_* model, simple_edge& e);

        TriModel_<FP>* model_;
        CircularArray<node_idx,2> ns_;
    };

    class Face : private equality_comparable<Face> {
    public:
    
        /** Default constructor creates an invalid Face object */
        Face() : model_(nullptr) {};

        /** Get the index of the Face 
         *
         * @pre is_valid()
         */
        face_idx get_fidx() const { return tri_; }

        face_idx get_idx() const { return tri_; }

        /** Get the component id */
        component get_component() const { return model_->get_component(tri_); }

        /** Get a CircularArray containing the nodes of the Face 
         *
         * @pre is_valid()
         */
        simple_tri nodes() const { return model_->tris_[tri_]; }

        /** if storing nondata faces, return true if a face has data
         *  else return true if all nodes have data
         *
         * @pre is_valid()
         */
        bool is_dataface() const {
            if (model_->using_nodes()) { 
                return model_->is_datanode(model_->tris_[tri_][0]) &&
                       model_->is_datanode(model_->tris_[tri_][1]) &&
                       model_->is_datanode(model_->tris_[tri_][2]) ;
            } else {
                return model_->is_dataface(tri_);
            }
        }

        /** Return true if this node is part of this face
         * @pre is_valid()
         */
        bool has_node(node_idx nidx) const {
            if ( (model_->tris_[tri_][0] == nidx) ||
                 (model_->tris_[tri_][1] == nidx) ||
                 (model_->tris_[tri_][2] == nidx) ) {
                return true;
            } else {
                return false;
            }
        }
   
        /** Return true if Node @a n is part of this Face */
        bool has_node(const Node& n) const { return has_node(n.get_nidx()); }
 
        /** Get a Triangle representation of this Face
         * @pre is_valid()
         */
        Triangle<FP> get_triangle() const { 
            return {model_->get_position(model_->tris_[tri_][0]),
                    model_->get_position(model_->tris_[tri_][1]),
                    model_->get_position(model_->tris_[tri_][2])};
        }

        /** Get the center of the triangle
         * @pre is_valid()
         */
        cv::Point3_<FP> get_center() const {
            return ( model_->get_position(model_->tris_[tri_][0]) + 
                     model_->get_position(model_->tris_[tri_][1]) + 
                     model_->get_position(model_->tris_[tri_][2]) ) / 3.0;
        }  

        /** Get all of the edges in this Face 
         *
         * @pre is_valid()
         */
        std::array<simple_edge,3> simple_edges() const;

        /** Get the number of adjacent Faces 
         *
         * @pre is_valid()
         */
        unsigned int number_of_adj_faces() const {
            return model_->number_of_adj_faces(tri_); 
        }

        /** Return the normal direction of the triangle 
         *
         * @pre is_valid()   
         */
        cv::Point3_<FP> get_normal() const;

        /** Return the area of the triangle 
         *
         * @pre is_valid()
         */
        FP get_area() const;

        /** Check if this triangle is contained within a bounding box
         *
         * @pre is_valid()
         */
        bool is_contained(const BoundingBox_<cv::Point3_<FP>>& bb) const;

        /** Check if this triangle intersects a bounding box
         *
         * @pre is_valid()
         */ 
        bool is_intersected(const BoundingBox_<cv::Point3_<FP>>& bb) const;

        /** Return a beginning iterator for looping through
         *  all adjacent Faces 
         * 
         * @pre is_valid()
         */
        AdjFaceIterator adj_face_begin() {
            return model_->adj_face_begin(tri_);
        }
        /** Return an end iterator for looping through
         *  all adjacent Faces 
         *
         * @pre is_valid()
         */
        AdjFaceIterator adj_face_end() {
            return model_->adj_face_end(tri_);
        }

        /** Return a beginning iterator for looping through
         *  all adjacent Faces in the same component 
         *
         * @pre is_valid()
         */
        CompAdjFaceIterator compadj_face_begin() {
            return model_->compadj_face_begin(tri_);
        }
        /** Return an end iterator for looping through
         *  all adjacent Faces in the same component 
         *
         * @pre is_valid()
         */
        CompAdjFaceIterator compadj_face_end() {
            return model_->compadj_face_end(tri_);
        }

        /** Check if this is a valid face */
        bool is_valid() const { return (model_ != nullptr) && (model_->is_face(tri_)); }

        /** Check if the two faces are identical */
        bool operator==(const Face& face) const {
            if (face.model_ == model_) {
                if (model_ == nullptr) {
                    return false;
                }
                return face.tri_ == tri_;
            }
            return false;
        }

        /** Get the distance from the ray origin to its intersection on the face
         *
         * @param[in] r         incoming ray
         * @return              if intersects, the distance, else < 0.0
         *
         * @pre is_valid()
         */
        double intersects(const Ray<FP>& r) const;

    private:
        friend class TriModel_;

        /** Create a valid Face object */
        Face(const TriModel_* model, face_idx t); 

        TriModel_<FP>* model_;
        face_idx tri_;
    };

    class NodeIterator : private equality_comparable<NodeIterator> {
    public:
        using value_type        = Node;
        using pointer           = Node*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        NodeIterator() : model_(nullptr) {};
        
        /** Dereference the iterator
         *  @pre iterator must be valid and < end iterator
         */
        Node operator*() const;

        /** Increment the iterator by 1 */
        NodeIterator& operator++();

        /** Test for equality */
        bool operator==(const NodeIterator& n_it) const;

    private:
        friend class TriModel_;

        /** Create a valid NodeIterator
         *
         *     
         * @param[in] grid  model to iterate through
         * @param[in] begin true for a beginning iterator, false for end
         */
        NodeIterator(const TriModel_* model, bool begin=false);

        TriModel_<FP>* model_;
        node_idx nidx_;
    };

    class FaceIterator : private equality_comparable<FaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        FaceIterator() : model_(nullptr) {};

        /** Dereference the iterator 
         *  @pre iterator must be valid and < end iterator
         */
        Face operator*() const;
        /** Increment the iterator by 1 */
        FaceIterator& operator++();
        /** Test for equality */
        bool operator==(const FaceIterator& adj_it) const;

    private:
        friend class TriModel_;

        /** Create a valid FaceIterator
         *
         * @param[in] model model to iterate through
         * @param[in] begin true for a beginning iterator, false for end
         */
        FaceIterator(const TriModel_* model, bool begin=false);

        TriModel_<FP>* model_;
        face_idx idx_;
    };

    /** Iterate through all faces within a component */
    class CompFaceIterator : private equality_comparable<CompFaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        CompFaceIterator() : model_(nullptr) {};

        /** Dereference the iterator 
         *  @pre iterator must be valid and < end iterator
         */
        Face operator*() const;
        /** Increment the iterator by 1 */
        CompFaceIterator& operator++();
        /** Test for equality */
        bool operator==(const CompFaceIterator& cf_it) const;

    private:
        friend class TriModel_;

        /** Create a valid FaceIterator
         *
         * @param[in] model     model to iterate through
         * @param[in] comp      component ID
         * @param[in] begin     true for a beginning iterator, false for end
         */
        CompFaceIterator(const TriModel_* model, component comp, bool begin=false);

        TriModel_<FP>* model_;
        component comp_;
        std::unordered_set<face_idx>::iterator it_;
    };

    class AdjNodeIterator : private equality_comparable<AdjNodeIterator> {
    public:
        using value_type        = Node;
        using pointer           = Node*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        AdjNodeIterator() : model_(nullptr) {}

        /** Get the index of the current Node */
        node_idx index() const { return adj_[idx_]; }

        /** Dereference the iterator
         * @pre iterator must be valid and < end iterator
         */
        Node operator*() const;

        /** Increment the iterator by 1 */
        AdjNodeIterator& operator++();
    
        /** Test for equality */
        bool operator==(const AdjNodeIterator& adj_it) const;

    private:
        friend class TriModel_;

        /** Create a valid AdjNodeIterator
         *
         * @param[in] model model to iterate through
         * @param[in] n     central node, iterate over adjacent nodes
         * @param[in] begin true for beginning iterator, else end iterator
         */
        AdjNodeIterator(const TriModel_* model, node_idx n, bool begin=false);

        TriModel_<FP>* model_;
        node_idx n_;
        int idx_;
        std::vector<node_idx> adj_;
    };

    class NAdjFaceIterator : private equality_comparable<NAdjFaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        NAdjFaceIterator() : model_(nullptr) {}; 

        /** Dereference the iterator
         * @pre iterator must be valid and < end iterator
         */
        Face operator*() const;

        /** Increment the iterator by 1 */
        NAdjFaceIterator& operator++();

        /** Test for equality */
        bool operator==(const NAdjFaceIterator& adj_it) const;

    private:
        friend class TriModel_;

        /** Create a valid NAdjFaceIterator
         *
         * @param[in] model model to iterate through
         * @param[in] n     central node, iterate over it's Faces
         * @param[in] begin true for beginning iterator, else end iterator
         */
        NAdjFaceIterator(const TriModel_* model, node_idx n, bool begin=false);

        TriModel_<FP>* model_;
        node_idx n_;
        unsigned int idx_;
        unsigned int count_;
        std::set<face_idx>::iterator it_;
    };

    class AdjFaceIterator : private equality_comparable<AdjFaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        AdjFaceIterator() : model_(nullptr) {};

        /** Dereference the iterator
         *  @pre iterator must be valid and < end iterator
         */
        Face operator*() const;

        /** Increment the iterator by 1 */
        AdjFaceIterator& operator++();

        /** Test for equality */
        bool operator==(const AdjFaceIterator& adj_it) const;

    private:
        friend class TriModel_;

        /** Create a valid AdjFaceIterator
         *
         * @param[in] model  model to iterate through
         * @param[in] t     central face, iterator over adjacent Faces
         * @param[in] begin true for beginning iterator, else end iterator
         */
        AdjFaceIterator(const TriModel_* model, face_idx t, bool begin=false);

        TriModel_<FP>* model_;
        face_idx t_;
        unsigned int idx_;
        unsigned int count_;
        std::array<face_idx,3> adj_;
    };

    class CompAdjFaceIterator : private equality_comparable<CompAdjFaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid object */
        CompAdjFaceIterator() : model_(nullptr) {};

        /** Dereference the iterator
         *  @pre iterator must be valid and < end iterator 
         */
        Face operator*() const;

        /** Increment the iterator by 1 */
        CompAdjFaceIterator& operator++();

        /** Test for equality */
        bool operator==(const CompAdjFaceIterator& adj_it) const;

    private:
        friend class TriModel_;

        /** Create a valid CompAdjFaceIterator
         *
         * @param[in] model model to iterate through
         * @param[in] t     central face, iterate over adjacent Faces
         *                  within the component
         * @param[in] begin true for beginning iterator, else end iterator
         */
        CompAdjFaceIterator(const TriModel_* model, face_idx t, bool begin=false);

        TriModel_<FP>* model_;
        face_idx t_;
        unsigned int idx_;
        unsigned int count_;
        std::array<face_idx,3> adj_;
    };

private:

    void generate_kd_tree();

    /** Identifies nodes that align with other nodes
     *  and collapses these into a single node
     * 
     * @return  number of unique overlapping points found
     *          plus the number of dropped nodes (no faces)
     */
    int intersect_grid();

    /***************************************************************/
    std::vector<FP> x_;
    std::vector<FP> y_;
    std::vector<FP> z_;

    kdtree* root_;        // general distance queries: all nodes on the surface
    std::vector<cv::Point3_<FP>> normals_;

    // if true, node can hold data
    // if false, node is just there to complete the surface (watertight)
    // std::vector<bool> data_nodes_; -> from base class

    // face_idx are universal, do not change with deletions
    unsigned int n_tris_; // number of triangles
    std::vector<CircularArray<node_idx,3>> tris_; // indices are universal index
    std::vector<bool> valid_tri_; // true if valid, false if deleted
    std::vector<std::set<face_idx>> n2t_; // map nodes to faces

    // Arbitrary component IDs, do not need to start with 1
    // note that *.tri or *.i.tri do need to start with 1 and be ordered
    // so that may require shifting
    std::unordered_map<component,std::unordered_set<face_idx>> comps_; // 1 map per component
    std::vector<int> f2c_; // map face_idx to component (no change upon deletion)
    
};

} /* end namespace upsp */

#include "../lib/TriModel.ipp"

#endif /* UFML_TRIGRID_H */
