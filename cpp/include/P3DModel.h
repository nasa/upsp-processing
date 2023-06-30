/** @file
 *  @brief  Plot3D Grid Model
 *  @date   July 17, 2017
 *  @author jmpowel2
 */

#ifndef UFML_P3DMODEL_H_
#define UFML_P3DMODEL_H_

#include <algorithm>
#include <array>
#include <assert.h>
#include <boost/iterator/transform_iterator.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "utils/general_utils.h"

#include "data_structs.h"
#include "grids.h"
#include "models.h"
#include "projection.h"

#include "Model.h"

// SANDSTROM: 2019-Jun-03: replace octree with bvh

// #include "Octree.h"

//#define DBG
#include "utils/pspTimer.h" // BlockTimer
#include "utils/pspKdtree.h" // kdtree
// SANDSTROM: ... 2019-Jun-03

namespace upsp {

struct GridIndex : private equality_comparable<GridIndex> {
    GridIndex() : zone(-1) {}
    GridIndex(int zone_in, int j_in, int k_in, int l_in=0) : zone(zone_in), 
            j(j_in), k(k_in), l(l_in) {}

    bool operator==(const GridIndex& gidx) const {
        return ( (zone == gidx.zone) && (j == gidx.j) && (k == gidx.k) && (l == gidx.l) );
    }

    inline int pts() const { return j*k*l;}

    int zone;
    int j;
    int k;
    int l;
};

/** Add Grid Index to stream */
std::ostream& operator<<(std::ostream& os, const GridIndex& gidx);

/** Plot3D Model
 *
 * @tparam FP       floating point (either single or double)
 */
template<typename FP>
class P3DModel_ : public Model<FP> {
public:
    
    using grid_size = GridIndex;
    using node_idx = unsigned int;
    using face_idx = unsigned int;

    typedef FP data_type;

    using Model<FP>::is_datanode;
    using Model<FP>::set_node_nondata;
    using Model<FP>::using_nodes;
    using Model<FP>::is_dataface;
    using Model<FP>::set_face_nondata;
    
    class Node;
    class Face;
    class NodeIterator;
    class FaceIterator;
    class EdgeIterator;
    class AdjNodeIterator;
    class AdjFaceIterator;

    /** Create an empty model */
    P3DModel_() : zones_(0), n_vert_(0), n_face_(0) {}

    /** Loads the p3d file and computes overlap between zones
     *
     * @param[in] filename  p3d data file
     *
     * @pre The grid must be a 2D surface grid; for each zone i,  
     *      jmax[i*3+2] == 1 && ( jmax[i*3+1] != 1 && jmax[i*3] != 1 )
     * @pre All zones in the grid must align pointwise with adjacent zones
     *      (need a block zonal grid)
     */
    P3DModel_(std::string filename, FP tol = 0.0);

    /** Generates a p3d model from a structured grid 
     *
     * @param[in] grid      structured grid (can be multiple zones)
     *
     * @pre Must be a 2D surface grid, for all 0 <= i < grid.size() grid_size[i][2] == 1
     * @pre All zones in grid must align pointwise with adjacent zones 
     *      (need a block zonal grid)
     */
    template<typename FPin>
    P3DModel_(const upsp::StructuredGrid<FPin>& grid, FP tol = 0.0);

    /** Loads a structured grid into the model
     *
     * @param[in] grid      structured grid (can be multiple zones)
     *
     * @pre Must be a surface grid, for all 0 <= i < grid.size() grid_size[i][2] == 1
     * @pre All zones in grid must align pointwise with adjacent zones 
     *      (need a block zonal grid)
     */
    template<typename FPin>
    void load_grid(const upsp::StructuredGrid<FPin>& grid, FP tol = 0.0);

    /** Copy duplicate data to duplicate nodes
     *
     * @param[in,out] sol   solution vector
     */
    void adjust_solution(std::vector<FP>& sol) const;

    /** Set the normal direction for a node (override default)
     *
     * @param[in] nidx      node index
     * @param[in] dir       normal direction (untransformed)
     */
    void set_normals(node_idx nidx, cv::Point3_<FP> dir);

    /** Convert to unstructured grid
     *  maintain node indexing
     *
     * @post @return.size() == size()
     * @post @return.get_x() == get_x()
     * @post @return.get_y() == get_y()
     * @post @return.get_z() == get_z()
     * @post cv::Point3_<FP>(@return.x[i],@return.y[i],@return.z[i]) == get_position(i)
     *       for all 0 <= i < size()
     */
    UnstructuredGrid<FP> triangulate() const;

    /** Cast position data */
    template<typename FPout>
    P3DModel_<FPout> cast() const;

    /** Write out all the points that are considered overlapping with other points
     * written in xyz format
     *
     * @param[in] filename  file to write the data to
     */
    void write_overlapped_pts(std::string filename) const;

    /** Mark every grid point that is considered overlapping with other points
     *
     * @param[out] marks        if overlap, 1, otherwise 0
     *
     * @post marks.size() == size()
     */
    template<typename T> 
    void mark_overlapped_pts(std::vector<T>& marks) const;

    /** Extract triangulation that covers entire model surface
     */
    void extract_tris(std::vector<float>& tris, std::vector<int>& triNodes) const;

    /** Write out an unformatted plot3d file
     */
    void write_grid(std::string filename) const;

    /** Write out the grid as a triangulation
     *
     * @param[in] filename  file to write the data to
     */
    void write_tri_grid(std::string filename) const;

    /** Write out a .triq single solution file 
     *
     * @param[in] filename  file that will be written to
     * @param[in] sol       a vector containing the solution at each node point
     *
     * @pre @a sol is length this->size()
     */
    template<typename T>
    void write_triq_sol(std::string filename, const std::vector<T>& sol) const;

    //void write_tec_sol(std::string filename, const std::vector<double>& sol, double time=0.0) const;

    /** Get the position of a grid point
     *
     * @param[in] gidx  grid index of interest
     * @return          the position of the grid point in 3D space
     *
     * @pre gidx must be a valid grid index
     */
    cv::Point3_<FP> get_position(const node_idx nidx) const;

    /** Get the position of a grid point
     *
     * @param[in] nidx  node index of interest
     * @return          the position of the grid point in 3D space
     *
     * @pre nidx must be a valid node index
     */
    cv::Point3_<FP> get_position(const GridIndex gidx) const;

    /** Return true if the model is structured (always true) */
    bool is_structured() const { return true; }

    /** Determine if the node is overlapping 
     *  @param[in] nidx     node index of interest
     */
    bool is_overlapping(node_idx nidx) const;

    /** Determine if a node is overlapped and the overlapping node has a lower node_idx
     *
     * @param[in] nidx  node index of interest
     */
    bool is_superceded(node_idx nidx) const;

    /** Find the lowest node index that overlaps with the given index
     *
     * @param[in] nidx  node index 
     * @return          node_idx such that overlap_pts_[@return][0] < @return
     */
    node_idx get_low_nidx(node_idx nidx) const;

    /** Return the number of zones */
    unsigned int num_zones() const { return zones_;}

    /** Return the number of zones (components) */
    unsigned int number_of_components() const { return zones_; }

    /** Return the number of nodes */
    unsigned int size() const { return x_.size();}

    /** Return the size of a zone (j,k,l) */
    grid_size size(unsigned int zone) const;

    /** Return the number of faces */
    unsigned int number_of_faces() const;

    /** Return the number of nodes in a zone */
    unsigned int zone_size(unsigned int zone) const;

    /** Return the number of nodes in zone for a single index (0,1,2) = (j,k,l) */
    unsigned int zone_size(unsigned int zone, unsigned int idx) const;

    /** Return the first node index within a zone */
    unsigned int zone_start_idx(unsigned int zone) const;

    /** Return a Node given the node index */
    Node node(node_idx nidx) const { return Node(this, nidx2_gidx(nidx));}
    /** Return a Node given the grid index */
    Node node(GridIndex gidx) const { return Node(this, gidx);}

    /** Return a Face given the face index */
    Face face(face_idx fidx) const { return Face(this, fidx); }

    /** Return a Face given three nodes */
    Face face(GridIndex gidx1, GridIndex gidx2, GridIndex gidx3) const {
        return Face(this, gidx1, gidx2, gidx3);
    }

    /** Create a NodeIterator to the first node (node index = 0) */
    NodeIterator node_begin() const;
    /** Create a NodeIterator to one past the last node */
    NodeIterator node_end() const;

    /** Create a NodeIterator to the first node (node index = 0) */
    NodeIterator cnode_begin() const { return node_begin(); }
    /** Create a NodeIterator to one past the last node */
    NodeIterator cnode_end() const { return node_end(); }

    /** Create a FaceIterator the first face */
    FaceIterator face_begin() const;
    /** Create a FaceIterator to one past the last face */
    FaceIterator face_end() const;

    /** Create a FaceIterator the first face */
    FaceIterator cface_begin() const { return face_begin(); }
    /** Create a FaceIterator to one past the last face */
    FaceIterator cface_end() const { return face_end(); }

    /** Create an EdgeIterator to the first edge in a zone */
    EdgeIterator edge_begin(int zone) const;
    /** Create an EdgeIterator one past the last edge in a zone */
    EdgeIterator edge_end(int zone) const;

    /** Create an AdjNodeIterator to the first adjacent node 
     * of the grid index 
     */
    AdjNodeIterator adj_node_begin(GridIndex gidx) const;
    /** Create an AdjNodeIterator to one past the last adjacent node 
     *  of the grid index 
     */
    AdjNodeIterator adj_node_end(GridIndex gidx) const;

    /** Create an AdjFaceIterator to the first adjacent face
     * of the grid index 
     */
    AdjFaceIterator adj_face_begin(GridIndex gidx) const;
    /** Create an AdjFaceIterator to the first adjacent face
     * of the node index 
     */
    AdjFaceIterator adj_face_begin(node_idx nidx) const;
    /** Create an AdjFaceIterator to one past the last adjacent face
     * of the grid index 
     */
    AdjFaceIterator adj_face_end(GridIndex gidx) const;
    /** Create an AdjFaceIterator to one past the last adjacent face
     * of the node index 
     */
    AdjFaceIterator adj_face_end(node_idx nidx) const;

    /** Create an AdjFaceIterator to the first adjacent face
     * of the grid index 
     */
    AdjFaceIterator cadj_face_begin(GridIndex gidx) const { return adj_face_begin(gidx); }
    /** Create an AdjFaceIterator to the first adjacent face
     * of the node index 
     */
    AdjFaceIterator cadj_face_begin(node_idx nidx) const { return adj_face_begin(nidx); }
    /** Create an AdjFaceIterator to one past the last adjacent face
     * of the grid index 
     */
    AdjFaceIterator cadj_face_end(GridIndex gidx) const { return adj_face_end(gidx); }
    /** Create an AdjFaceIterator to one past the last adjacent face
     * of the node index 
     */
    AdjFaceIterator cadj_face_end(node_idx nidx) const { return adj_face_end(nidx); }

    /** Return a reference to the vector of node x locations */
    const std::vector<FP>& get_x() const { return x_;}
    /** Return a reference to the vector of node y locations */
    const std::vector<FP>& get_y() const { return y_;}
    /** Return a reference to the vector of node z locations */
    const std::vector<FP>& get_z() const { return z_;}

    // SANDSTROM: 2019-Jun-03: added pre-transformed [i.e. run conditions] normal access
    /** Return a reference to the vector of xformd normals */
    const std::vector<cv::Point3_<FP> >& get_n() const { return normals_;}
    // SANDSTROM: ... 2019-Jun-03

    // SANDSTROM: 2019-Jul-10: provide access to the tree for queries
    const kdtree* kdRoot(void) const {return root;}
    // SANDSTROM: ... 2019-Jul-10:

    private:
    void calcNormals();

    public:

    class Node : private totally_ordered<Node> {
    public:
        /** Default constructor creates invalid Node */
        Node() { model_ = nullptr;}

        /** Get the zone number of the Node */
        int get_zone() const { return model_->get_zone(nidx_); }

        /** Get the (x,y,z) position of the Node */
        cv::Point3_<FP> get_position() const { return model_->get_position(nidx_); } 

        /** Get the grid index of the Node */
        GridIndex get_gidx() const { return model_->nidx2_gidx(nidx_); }
        /** Get the node index of the Node */
        node_idx get_nidx() const { return nidx_; }
        node_idx get_idx() const { return nidx_; }

        /** Check if this node overlaps another 
         *
         * @post if n overlaps this node or node(n) == *this, return true
         */
        bool overlaps(node_idx n) const;

        /** Return true if this node holds data */
        bool is_datanode() const { return model_->is_datanode(nidx_); }

        /** Return true, all structured grid nodes are part of a single component */
        bool has_primary_component() const { return true; }

        /** Return zone number */
        int get_primary_component() const { return get_zone(); }

        /** Get an approximate normal direction for a node
         *
         * @return  if normal is set, use that, otherwise the unit 
         *          normal direction defined as 
         *          the area weighted average of all adjacent face 
         *          normal directions
         */
        cv::Point3_<FP> get_normal() const;

        /** Return true if this is a valid Node */
        bool is_valid() const { return (model_ != nullptr); }

        /** Check if this node is contained within a bounding box
         */
        bool is_contained(const BoundingBox_<cv::Point3_<FP>>& bb) const {
            return contains(bb, get_position());
        }

        /* Check if this node is contained within a bounding box
         */
        bool is_intersected(const BoundingBox_<cv::Point3_<FP>>& bb) const {
            return is_contained(bb);
        }

        /** Create an iterator to the first adjacent node */
        AdjNodeIterator adj_node_begin() const { 
            return model_->adj_node_begin(get_gidx());
        }
        /** Create an iterator to one past the last adjacent node */
        AdjNodeIterator adj_node_end() const {
            return model_->adj_node_end(get_gidx());
        }

        /** Create an iterator to the first adjacent face */
        AdjFaceIterator adj_face_begin() const {
            return model_->adj_face_begin(nidx_);
        }
        /** Create an iterator to one past the last adjacent face */
        AdjFaceIterator adj_face_end() const {
            return model_->adj_face_end(nidx_);
        }

        /** Compare to another Node, return true if less than that Node 
         * ordered by node index
         */
        bool operator<(const Node& node) const;
        /** Compare to another Node, return true if they are equal */
        bool operator==(const Node& node) const;

        /** Write out the grid index to the stream */
        friend std::ostream& operator<<(std::ostream& os, const Node& node) {
            GridIndex gidx = node.get_gidx();
            return operator<<(os,gidx);
        }

    private:
        friend class P3DModel_;

        /** The only way to create a valid Node is through the P3DModel_ methods
         *
         * @param[in] model P3DModel_ that is calling this constructor
         * @param[in] gidx  grid index of the desired Node
         */
        Node(const P3DModel_* model, GridIndex gidx);

        node_idx nidx_;
        P3DModel_* model_;
    };

    class Face : private equality_comparable<Face> {
    public:
        /** Default constructor creates invalid face */
        Face() { model_ = nullptr; }

        /** Return true if this is a valid Face */
        bool is_valid() const { return model_ != nullptr; }

        /** Return the face_idx */
        face_idx get_fidx() const { return fidx_; }

        /** Get the center of the quad */
        cv::Point3_<FP> get_center() const {
            return ( model_->get_position(nidx_[0]) + 
                     model_->get_position(nidx_[1]) + 
                     model_->get_position(nidx_[2]) + 
                     model_->get_position(nidx_[3]) ) / 4.0;
        } 

        /** Determine if the face includes a particular node
         *
         * @param[in] nidx  node index
         * @return          true if nidx or an overlap is in nidx_
         */
        bool has_node(node_idx nidx) const;

        /** Return true if the Node @a n is part of this Face */
        bool has_node(const Node& n) const { return has_node(n.get_nidx()); }

        /** Get surface normal direction
         *
         * @return  unit vector representing normal direction
         *
         * @post    unit vector found using nodes 1,2,3; assume 4 is coplanar
         */
        cv::Point3_<FP> get_normal() const;

        /** Get the area of the node
         *
         * @return  area
         */
        double get_area() const;

        /** Get adjacent face
         * 
         * @param[in] g_dir     direction along the grid surface
         *
         * @post if there is no adjacent face in that direction, return invalid Face
         */
        Face get_adjacent(GridDir g_dir) const;

        /** Get edge Nodes
         *
         * @param[in] g_dir     direction along the grid surface
         * @return  the two nodes on this edge ([0] will represent the lower index,
         *          either in j or k, according to this face's zone's directions)
         */
        std::array<Node,2> get_edge_nodes(GridDir g_dir) const;

        /** Get the distance from the ray origin to its intersection on the face
         *
         * @param[in] r     incoming ray
         * @return          if intersects, the distance , else < 0.0
         */
        double intersects(const upsp::Ray<FP>& r) const;

        /** Get a vector of all node indices that make up this Face 
         *
         * @post @return will be ordered such that the loop formed by the
         *       face creates the correct normal with the right hand rule
         */
        const std::vector<node_idx>& get_nodes() const { return nidx_;}

        /** Determine the zone in which the face lies
         *
         * @return  zone that contains the face
         */
        int get_zone() const;

        /** Check if two faces are equivalent
         * @return  true if they are from the same model and contain 
         *          at least 3 of the same nodes
         */
        bool operator==(const Face& face) const;

        /** Add all Nodes in the face to the stream */
        friend std::ostream& operator<<(std::ostream& os, const Face& face) {
            for (int i=0; i < face.nidx_.size(); ++i) {
                os << face.model_->node(face.nidx_[i]); 
                if (i != face.nidx_.size() - 1) {
                    os << " ";
                }
            }   
            return os;
        }

    private:
        friend class P3DModel_;
    
        /** The only way to create a valid Face is through the P3DModel_ functions
         *
         * @param[in] model P3DModel_ that is calling this constructor
         * @param[in] gidx1 first node adjacent to Face
         * @param[in] gidx2 second node adjacent to Face
         * @param[in] gidx3 third node adjacent to Face
         *
         * @pre for a valid Face two of the inputted nodes must be adjacent to another
         * @post if 4th node cannot be found to complete quad, invalid Face is formed
         * @post nodes are ordered, so 1 connects to 2 and 4, 2 connects to 1 and 3, ...
         * @post the nodes are ordered, so that using the right hand rule 1,2,3,4 gives
         *      the correct outward pointing normal direction
         * @post all nodes are the lowest nidx of any of their overlapped points
         */
        Face(const P3DModel_* model, GridIndex gidx1, GridIndex gidx2, GridIndex gidx3);

        /** The only way to create a valid Face is through the P3DModel_ functions
         *
         * @param[in] model P3DModel_ that is calling this constructor
         * @param[in] fidx  face index
         */
        Face(const P3DModel_* model, face_idx fidx);

        /** Move all gidx to the common zone
         *
         * @param[in] gidx1 first node
         * @param[in] gidx2 second node
         * @param[in] gidx3 third node
         * @return the face index if a valid face can be formed, else -1
         *
         * @pre     gidx2 is adjacent to both gidx1 and gidx3
         */
        int move_to_common(GridIndex& gidx1, GridIndex& gidx2, GridIndex& gidx3) const;

        /** Reorder the first 3 Nodes in the Face so that 1,2,3 gives the correct
         * outward pointing normal direction
         *
         * @param[in] gidx1 first node
         * @param[in] gidx2 second node
         * @param[in] gidx3 third node
         *
         * @pre     model_ is valid and contains the 3 nodes
         * @pre model_->are_simply_adjacent(gidx1,gidx2) && 
         *      model_->are_simply_adjacent(gidx2,gidx3)
         * @pre gidx1,gidx2,gidx3 form a face
         * @post    gidx1,gidx2,gidx3 are ordered such that using the right hand rule gives
         *          the correct outward pointing normal direction
         * @post model_->are_simply_adjacent(gidx1,gidx2) && 
         *       model_->are_simply_adjacent(gidx2,gidx3)
         */
        void order_nodes(GridIndex& gidx1, GridIndex& gidx2, GridIndex& gidx3) const;

        face_idx fidx_;
        std::vector<node_idx> nidx_;
        P3DModel_* model_;
    };

    class NodeIterator : private equality_comparable<NodeIterator> {
    public:
        using value_type        = Node;
        using pointer           = Node*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid iterator */
        NodeIterator() { this->model_ = nullptr; }

        /** Dereference the iterator
         */
        Node operator*() const;

        /** Increment the iterator by 1
         * will skip over points with overlapping points of lower node_idx
         * will stop iterating when end is reached
         */
        NodeIterator& operator++();

        /** Check whether the node iterators are equivalent
         *
         * @param[in] ei    NodeIterator to compare against
         * @return          true if NodeIterators point to the same grid point 
         *                  or equivalent end NodeIterators
         */
        bool operator==(const NodeIterator& ni) const;

    private:
        friend class P3DModel_;

        /** The only way to create a valid NodeIterator is through P3DModel_ methods
         * If gidx.zone >= model->zones_, will construct an end iterator
         * will allow constructing an iterator on a grid point that would be passed over
         * using the increment operator
         *
         * @param[in] model P3DModel_ that is call this constructor
         * @param[in] gidx  GridIndex on which to create the iterator
         */
        NodeIterator(const P3DModel_* model, GridIndex gidx); 

        node_idx nidx_;
        P3DModel_* model_;
    };

    class FaceIterator : private equality_comparable<FaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid iterator */
        FaceIterator() {this->model_ = nullptr;}

        /** Dereference the iterator
         */
        Face operator*() const;

        /** Increment the iterator by 1
         * will stop when end is reached
         */
        FaceIterator& operator++();

        /** Check whether the face iterators are equivalent
         *
         * @param[in] fi    FaceIterator to compare against
         * @return          true if FaceIterators point to the same grid point or equivalent
         *                  end FaceIterators
         */
        bool operator==(const FaceIterator& fi) const;

    private:
        friend class P3DModel_;

        /** The only way to create a valid FaceIterator is through the P3DModel_ methods
         *
         * @param[in] model P3DModel_ that is calling this constructor
         * @param[in] fidx  face at which to begin iterating
         *
         * @post    if fidx is not valid in model, then create end iterator
         */
        FaceIterator(const P3DModel_* model, face_idx fidx);

        face_idx fidx_;
        P3DModel_* model_;
    };

    class EdgeIterator : private equality_comparable<EdgeIterator> {
    public:
        using value_type        = Node;
        using pointer           = Node*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid iterator */
        EdgeIterator() { this->model_ = nullptr;}

        /** Dereference the iterator
         */
        Node operator*() const;

        /** Increment the iterator by one
         */
        EdgeIterator& operator++();

        /** Determine if iterators are equivalent
         */
        bool operator==(const EdgeIterator& ei) const;

    private:
        friend class P3DModel_;

        /** The only way to create a valid EdgeIterator is through the P3DModel_ methods
         * which can call this private constructor
         *
         * @param[in] model P3DModel_ that is calling this constructor
         * @param[in] zone  zone over which to create the iterator
         * @param[in] end   true if create an ending iterator, else create beginning
         */
        EdgeIterator(const P3DModel_* model, int zone, bool end=false); 

        int path_;
        node_idx nidx_;
        P3DModel_* model_;
    };

    /** Iterate over all nodes adjacent to another Node */
    class AdjNodeIterator : private equality_comparable<AdjNodeIterator> {
    public:
        using value_type        = Node;
        using pointer           = Node*;
        using reference         = Node&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid iterator */
        AdjNodeIterator() { this->model_ = nullptr;}

        /** Dereference the iterator
         */
        Node operator*() const;

        /** Increment the iterator by 1
         */
        AdjNodeIterator& operator++();

        /** Determine if iterators are equivalent
         */
        bool operator==(const AdjNodeIterator& ani) const;

        /** Return the central Node */
        Node get_center() const;

    private:
        friend class P3DModel_;

        /** The only way to create a valid AdjNodeIterator is through 
         *  the P3DModel_ methods
         *  which can call this private constructor
         *
         * @param[in] model     P3DModel_ that is calling this constructor
         * @param[in] gidx      node for which to traverse adjacent nodes
         * @param[in] end       true if create an ending iterator, else create beginning
         *
         * @post    if end, adj_.size() == 0, else adj_.size() > 0
         */
        AdjNodeIterator(const P3DModel_* model, GridIndex gidx, bool end=false);

        node_idx nidx_;
        std::vector<node_idx> adj_;
        P3DModel_* model_;
    };

    class AdjFaceIterator : private equality_comparable<AdjFaceIterator> {
    public:
        using value_type        = Face;
        using pointer           = Face*;
        using reference         = Face&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructor creates an invalid iterator */
        AdjFaceIterator() { this->model_ = nullptr;}

        /** Dereference the iterator
         */
        Face operator*() const;

        /** Increment the iterator by 1
         */
        AdjFaceIterator& operator++();

        /** Determine if the iterators are equivalent
         */
        bool operator==(const AdjFaceIterator& afi) const;

    private:
        friend class P3DModel_;

        /** The only way to create a valid AdjFaceIterator is through 
         *  the P3DModel_ methods
         *  which can call this private constructor
         *
         * @param[in] model     P3DModel_ that is calling this constructor
         * @param[in] gidx      node for which to traverse adjacent faces
         * @param[in] end       true if create an ending iterator, else create beginning
         *
         * @post    if end, adj_.size() == 0, else adj_.size() > 0
         */
        AdjFaceIterator(const P3DModel_* model, GridIndex gidx, bool end=false);

        node_idx nidx_;
        std::vector<Face> adj_;
        P3DModel_* model_;
    };

private:
    template<typename T>
    friend class P3DModel_;


    /** Reads a p3d file into the object
     *
     * @param[in] filename  p3d data file
     */
    void read_p3d_file(std::string filename);

    /** Identify which zone a node_idx belongs to
     *
     * @param[in] nidx  index into x_,y_,and z_ representing a Node
     * @return          if nidx valid, zone nidx belongs to, else -1
     */
    int get_zone(node_idx nidx) const;

    /** Identifies the full GridIndex a given index belongs to
     *
     * @param[in] nidx  index into x_,y_,and z_ representing  Node
     * @return          if idx is valid, full GridIndex, otherwise empty GridIndex
     */
    GridIndex nidx2_gidx(node_idx nidx) const;
    /** Identify the vector index given the full GridIndex
     *
     * @param[in] gidx  GridIndex defining the node
     * @return          if gidx is valid, the idx corresponding to gidx
     */
    node_idx gidx2_nidx(const GridIndex& gidx) const;

    /** Identify the minimum grid point given the face index
     *
     * @param[in] fidx  face index
     * @return the node with the minimum j/k that is part of this face
     */
    GridIndex fidx2_min_gidx(face_idx fidx) const;

    /** Check if a GridIndex represents a valid node in the model
     *
     * @param[in] gidx  GridIndex to check
     * @return          true if valid, else false
     */
    bool is_valid_gidx(const GridIndex& gidx) const;

    /** Check whether or not a grid is a surface grid
     *
     * @param[in] sz    grid size object for the grid
     * @return          true if exactly one of sz.j,sz.k,sz.l == 1, else false
     */
    bool is_surface(const grid_size& sz) const;

    /** Check whether a node is on the edge of its zone
     *
     * @param[in] gidx  node in the grid to check
     * @return          true if node is on the edge of its zone, else false
     *
     */
    bool is_on_edge(const GridIndex& gidx) const;

    /** Check if two nodes are adjacent
     *
     * @param[in] gidx1 first node
     * @param[in] gidx2 second node
     * @return          true if gidx1 and gidx2 are adjacent else false
     */
    bool are_adjacent(const GridIndex& gidx1, const GridIndex& gidx2) const;

    /** Check if two zones are in the same zone and one grid point apart
     * 
     * @param[in] gidx1 first node
     * @param[in] gidx2 second node
     * @return          true if gidx1 and gidx2 are simply adjacent else false
     */
    bool are_simply_adj(const GridIndex& gidx1, const GridIndex& gidx2) const;

    /** Adjust GridIndex to point to the corner of interest
     * 
     * @param[in] gidx      GridIndex about which to find simply adjacent nodes
     * @param[in] corner    corner id (0=j--,1=j++,2=k--,3=k++)
     * @return              if corner exists, GridIndex of the corner, else gidx
     *
     * @post if corner exists, gidx is the corner, else gidx is unchanged
     */
    GridIndex get_adj_corner(const GridIndex& gidx, int corner) const;

    /** Find all nodes adjacent to a node
     *
     * @param[in] nidx      node to find adjacent nodes of
     * @param[in,out] adj   vector full of adjacent nodes
     *
     * @post    if i in adj, for all j in overlap_pts_[i], j > i 
     *          "discard duplicates, keep lowest nidx" 
     * @post for all i,j s.t. 0 <= i < j < @a adj.siz(), adj[i] < adj[j]
     */
    void find_adjacent(node_idx nidx, std::vector<node_idx>& adj) const;

    /** Find all faces that are adjacent to a node
     *
     * @param[in] nidx      node about which to find faces
     * @param[in,out] adj   vector containing all adjacent faces
     */
    void find_adjacent_faces(node_idx nidx, std::vector<Face>& adj) const;

    /** Get the edge id that a given index rests on
     *
     * @param[in] gidx  grid index of interest
     * @return          if valid, edge id for the node (0 if j==j_start, 1 if j==j_end, 
     *                      2 if k==k_start, 3 if k=k_end)
     *                  else -1
     */ 
    int get_edge_id(const GridIndex& gidx) const;

    // SANDSTROM: 2019-Jun-03: use kdtree to identify overlap, store into overlap_pts_
    std::pair<int,int> identifyOverlap(float tol);
    // SANDSTROM: ... 2019-Jun-03

    /*************************************************************************/
    int zones_;
    std::vector<grid_size> sz_; 

    FP overlap_tol_;

    std::vector<FP> x_;
    std::vector<FP> y_;
    std::vector<FP> z_;

    std::map<node_idx,std::vector<node_idx>> overlap_pts_;

    int32_t n_vert_;
    int32_t n_face_;

    // SANDSTROM: 2019-Jun-03: added kdtree-base overlap
    // these store node positions in "run" coords, xformed: model->tunnel->run
    kdtree* overlapRoot; // overlap testing only: contains only nodes along zone boundaries
    kdtree* root;        // general distance queries: all nodes on the surface

    // SANDSTROM: 2019-Jun-03: added pre-calc'd xformed normals: model->tunnel->run
    std::vector<cv::Point3_<FP>> normals_;
    // std::unordered_map<node_idx,cv::Point3_<FP>> normals_;
    // SANDSTROM: ... 2019-Jun-03

    // if true,  node can hold data
    // if false, node is just there to complete surface (watertight)
    // std::vector<bool> data_nodes_; -> inherit from base
};

} /* end namespace upsp */

#include "../lib/P3DModel.ipp"

#endif /* UFML_P3DMODEL_H_ */
