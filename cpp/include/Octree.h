/** @file
 *  @brief  Octree containing Model Grid
 *  @date   July 24, 2017
 *  @author jmpowel2
 */

#ifndef UFML_OCTREE_H_
#define UFML_OCTREE_H_

#include <boost/iterator/transform_iterator.hpp>
#include <cstdlib>
#include <fstream>
#include <stack>
#include <queue>

#include "data_structs.h"
#include "models.h"
#include "projection.h"
#include "utils/general_utils.h"
#include "utils/cv_extras.h"

namespace upsp {

// Helper Struct for getting model elements
template<typename Model>
struct Model_Elements {

    typedef typename Model::Node Node;
    typedef typename Model::Face Face;
    typedef unsigned int model_idx;

    Model_Elements(const Model* model) : model_(model) {}

    Node get_element(const Node& u_null, model_idx midx) const {
        return model_->node(midx);
    }

    Face get_element(const Face& u_null, model_idx midx) const {
        return model_->cface(midx);
    }

    const Model* model_;
};

#define MAX_DEPTH 14

/** Octree which holds model elements
 *
 * @taparam T       Model 
 * @tparam  E       Model Element (Node or Face)
 */
template<typename T, typename E>
class Octree {
public:

    typedef T Model;
    typedef E Elem;
    typedef unsigned int model_idx;

    typedef typename T::data_type FP;

    struct Node;
    class BreadthFirstIterator;
    class BreadthFirstIterator2;
    class DepthFirstIterator;

    /** Load nodes into the model into the tree and sub-divide as necessary
     *
     * @param[in] model         A model containing points that should be added to the tree
     * @param[in] begin         start of model node range of points to add
     * @parma[in] end           one past the end of the model node range of points to add
     * @param[in] max_per_node  maximum number of model nodes allowed per octant
     *
     * @pre model is valid
     * @post octree may have more than max_per_node in an octant if there are duplicate 
     *      points or MAX_DEPTH is reached
     */
    template<typename InputIterator>
    Octree(const T* model, InputIterator begin, InputIterator end, unsigned int max_per_node=15); 

    /** Load nodes into the model into the tree and sub-divide as necessary
     *
     * @param[in] model         A model containing points that should be added to the tree
     * @param[in] begin         start of model node range of points to add
     * @parma[in] end           one past the end of the model node range of points to add
     * @param[in] bb            bounding box encompassing all points that may be added
     * @param[in] max_per_node  maximum number of model nodes allowed per octant
     *
     * @pre model is valid
     * @pre bb bounds all points in range [begin,end)
     * @post octree may have more than max_per_node in an octant if there are duplicate 
     *      points or MAX_DEPTH is reached
     */
    template<typename InputIterator>
    Octree(const T* model, InputIterator begin, InputIterator end, BoundingBox_<cv::Point3_<FP>> bb, 
            unsigned int max_per_node=15);

    /** Deallocate all memory from the tree
     */
    ~Octree();

    /** Add points to the tree
     *
     * @param[in] begin start of range of model_node to add
     * @param[in] end   one past the end of the range of model_node to add
     * @return          number of octants added
     *
     * @pre root node must bound all nodes that will be added
     */
    template<typename InputIterator>
    int add_points(InputIterator begin, InputIterator end);

    /** For each leaf, sort the points in ascending order
     */
    void sort_points();

    /** Get the Root node */
    Node* get_root() const { return root_;}

    /** Get the underlying Model */
    const T* get_model() const { return model_;}

    /** Get an element from the underlying Model */
    Elem get_model_node(model_idx midx) const {return mod_helper_.get_element(Elem(), midx);}
    
    /** Find the smallest node that contains the element
     *
     * @param[in] elem  model element 
     * @return          node that contains this element (may not be a leaf)
     *
     * @pre root node must contain the point
     */
    Node* bounding_node(const E& elem) const;

    /** Find the leaf node that contains the point
     *
     * @param[in] pt    3D point
     * @return          leaf node containing @a pt
     *
     * @pre root node must contain the point
     */
    Node* bounding_node(cv::Point3_<FP> pt) const;

    /** Check if an element is contained within the Root node 
     */
    bool contains(const E& elem) const { return root_->contains(elem); }

    /** Check if a point is contained within the Root node
     */
    bool contains(cv::Point3_<FP> pt) const { return root_->contains(pt); }

    /** Write out the tree data structure for visualization
     *  format is ascii tecplot
     *
     * @param[in] filename  file for writing
     */
    void write_tree_structure(std::string filename) const;

    /** Get the maximum depth the Octree reaches */
    int get_max_depth() const { return depth_;}

    /** Get a beginning iterator for a Breadth First tree traversal 
     *  store queue in iterator
     */
    BreadthFirstIterator bf_begin() const { return BreadthFirstIterator(root_);}
    /** Get an end iterator for a Breadth First tree traversal 
     *  store queue in iterator
     */
    BreadthFirstIterator bf_end() const { return BreadthFirstIterator(nullptr);}

    /** Get a beginning iterator for a Breadth First tree traversal
     *  do not store queue in iterator
     */
    BreadthFirstIterator2 bf2_begin() const { return BreadthFirstIterator2(root_);}
    /** Get an end iterator for a Breadth First tree traversal
     *  do not store queue in iterator 
     */
    BreadthFirstIterator2 bf2_end() const { return BreadthFirstIterator2(nullptr);}

    /** Get a beginning iterator for a Depth First tree traversal */
    DepthFirstIterator df_begin() const { return DepthFirstIterator(root_);}
    /** Get an end iterator for a Depth First tree traversal */
    DepthFirstIterator df_end() const {return DepthFirstIterator(nullptr);}

    /** Octree Node Element */    
    struct Node {
        /** Create an invalid tree Node */
        Node() : parent(nullptr), children(nullptr), is_leaf_(true) {}

        /** Create a valid tree Node
         *
         * @param[in] parent_in     pointer to this new Node's parent
         * @param[in] center_in     the position of the center of this new Node
         * @param[in] half_width_in the half_width of the cubic octant
         */
        Node(Node* parent_in, cv::Point3_<FP> center_in, FP half_width_in);

        /** Destroy the Node and its descendents recursively
         */
        ~Node();

        /** Return true if there are no points within this node */
        bool empty() {return elems.size() == 0;}

        /** Return true if this is a leaf node */
        bool is_leaf() const {return is_leaf_;}

        /** Get the depth that this node is located at
         *
         * @return  depth of the node (root node is 0)
         *
         * @pre this must be a valid node
         */
        int get_depth() const;

        /** Get the child node
         * 
         * @param[in] i     identifier of the child in range [0,7]
         *
         * @pre !is_leaf()
         */
        Node* child(unsigned int i) const {
            assert(!is_leaf()); assert(i < 8);
            return &children[i];
        }

        /** Get a beginning iterator for a Depth First tree traversal beginning at
         *  this node 
         */
        DepthFirstIterator begin() { return DepthFirstIterator(this); }
        /** Get an end iterator for a Depth First tree traversal beginning at
         * this node
         */
        DepthFirstIterator end() { return DepthFirstIterator(nullptr); }

        /** Check if a model element is within this Node
         *
         * @param[in] elem  element to consider
         * @return          true if the entire element is within the node
         */
        bool contains(const E& elem) const;

        /** Check if a point is within this Node
         *
         * @param[in] pt    point to consider
         * @return          true if for each k = x,y,z, 
         *                  center_.k-half_width_ <= pt.k < center_.k+half_width_
         *                  else false
        */
        bool contains(cv::Point3_<FP> pt) const;

        /** Check if a model element intersects this Node
         */
        bool elem_intersects(const E& elem) const;

        /** Determine if a ray passes through a Node
         *
         * @param[in] r     ray
         */
        bool intersects(const Ray<FP>& r) const;

        /** Get the corners of the Node
         *
         * @param[in,out] corners   vector for returning 8 corners
         *
         * @post corners[0] is minimum extent, corners[7] is maximum extent
         */
        void get_corners(std::vector<cv::Point3_<FP>>& corners) const;

        /** Return the bounding box of the Node
         */
        BoundingBox_<cv::Point3_<FP>> get_bounding_box() const;

        cv::Point3_<FP> center;
        FP half_width;
        Node* parent;
        std::vector<model_idx> elems;

    private:
        friend class Octree;
        bool is_leaf_;
        Node* children;
    };
    
    class BreadthFirstIterator : private equality_comparable<BreadthFirstIterator> {
    public:
        using value_type        = Node*;
        using pointer           = Node**;
        using reference         = Node*&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::forward_iterator_tag;

        /** Default constructor creates an iterator to end */
        BreadthFirstIterator() {}

        /** Return the node pointed to by the iterator
         * 
         * @pre this iterator is valid and not an end iterator
         */
        Node* operator*() const {return qu_.front();}

        /** Increment the iterator by 1
         */
        BreadthFirstIterator& operator++();

        /** Compare iterators, return true if equal
         */
        bool operator==(const BreadthFirstIterator& bfi) const;

        /** Remove this node and all descendents from the search path 
         * 
         *  @return the new current iterator
         */
        BreadthFirstIterator& clip() { 
            qu_.pop(); 
            return *this;
        }
    
    private:
        friend class Octree;    
    
        /** Creates an iterator that will traverse the octree defined by an input "root"
         * breadth first
         *
         * @param[in] start root node of the tree to traverse, nullptr is an end iterator
         */
        BreadthFirstIterator(Node* start);

        std::queue<Node*> qu_;
    };

    class BreadthFirstIterator2 : private equality_comparable<BreadthFirstIterator2> {
    public:
        using value_type        = Node*;
        using pointer           = Node**;
        using reference         = Node*&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::forward_iterator_tag;

        /** Default constructor creates an iterator to end */
        BreadthFirstIterator2() { this->current_ = nullptr; }

        /** Return the node pointed to by the iterator
         *
         * @pre this iterator is valid and not an end iterator
         */
        Node* operator*() const {return current_;}

        /** Increment the iterator by 1
         */
        BreadthFirstIterator2& operator++();

        /** Compare iterators, return true if equal
         */
        bool operator==(const BreadthFirstIterator2& bfi) const;
    
    private:
        friend class Octree;    
    
        /** Creates an iterator that will traverse the octree breadth first
         * 
         * @param[in] start Node at which to start the traversal, nullptr is an end iterator
         */
        BreadthFirstIterator2(Node* start);

        /** Find the next Node at a given depth starting from a given Node
         * this is a breadth search
         *
         * @param[in] start Node at which to begin the search
         * @param[in] depth depth at which to find Node 
         * @return          next Node at this level if exists, otherwise nullptr
         */
        Node* find_next_atdepth(Node* start, int depth) const;

        Node* current_;
    };

    class DepthFirstIterator : private equality_comparable<DepthFirstIterator> {
    public:
        using value_type        = Node*;
        using pointer           = Node**;
        using reference         = Node*&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::forward_iterator_tag;

        /** Default constructor creates an iterator to end */
        DepthFirstIterator() {this->current_ = nullptr;}

        /** Return the node pointed to by the iterator
         * 
         * @pre this iterator is valid and not an end iterator
         */
        Node* operator*() const {return current_;}

        /** Increment the iterator by 1
         */
        DepthFirstIterator& operator++();

        /** Compare iterators, return true if equal
         */
        bool operator==(const DepthFirstIterator& dfi) const;

    private:
        friend class Octree;

        /** Creates an iterator that will traverse the octree depth first
         * 
         * @param[in] start Node at which to start the traversal, nullptr is an end iterator
         */
        DepthFirstIterator(Node* start);

        Node* current_;
        Node* root_;
    };
    
private:

    /** Transform a leaf node into an inner node
     *
     * @param[in] n     leaf node to convert
     */
    void divide(Node* n);

    const T* model_;
    Node* root_;
    unsigned int max_per_node_;
    unsigned int depth_;

    Model_Elements<T> mod_helper_;
};


} /* end namespace upsp */

#include "../lib/Octree.ipp"

#endif /* UFML_OCTREE_H_ */
