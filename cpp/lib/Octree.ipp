/** @file
 *  @brief  Octree containing Model Grid
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */

namespace upsp {

/********************************************************************* 
 * Octree
 ********************************************************************/

/*****************************************************************************/
template<typename T, typename E>
template<typename InputIterator>
Octree<T,E>::Octree(const T* model, InputIterator begin, InputIterator end, unsigned int max_per_node) : Octree<T,E>(model, begin, end, get_extent(boost::make_transform_iterator(begin, Node2_Point<T>() ), boost::make_transform_iterator(end, Node2_Point<T>() )), max_per_node) {}

/*****************************************************************************/
template<typename T, typename E>
template<typename InputIterator>
Octree<T,E>::Octree(const T* model, InputIterator begin, InputIterator end, BoundingBox_<cv::Point3_<FP>> bb, unsigned int max_per_node) : 
        model_(model), max_per_node_(max_per_node), depth_(0), mod_helper_(model) {

    assert(model != nullptr);

    // Round the bounding box for clean divisions (reduced numerical error)
    bb.bounds[0] = floor(bb.bounds[0]);
    bb.bounds[1] = ceil(bb.bounds[1]);

    std::cout << "Bounding Box = " << bb.bounds[0] << " " << bb.bounds[1] << std::endl;

    // Get the extents of the model for bounding root node
    cv::Point3_<FP> center = 0.5*(bb.bounds[1] - bb.bounds[0]) + bb.bounds[0];
    cv::Point3_<FP> dist = bb.bounds[1] - center;
    FP half_width = max(dist.x, dist.y, dist.z);

    std::cout << "Center = " << center << std::endl;
    std::cout << "Half_width = " << half_width << std::endl;

    // Create the root node
    root_ = new Node(nullptr, center, half_width);
    
    // Add points to the octree
    int total_octants = add_points(begin,end) + 1;

    std::cout << std::endl;
    std::cout << "Created Octree: " << std::endl;
    std::cout << "  octant count        = " << total_octants << std::endl;
    std::cout << "  max depth           = " << depth_ << std::endl;
    std::cout << "  max points per leaf = " << max_per_node << std::endl;
    std::cout << std::endl;

}

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::~Octree() {
    delete root_;
}

/*****************************************************************************/
template<typename T, typename E>
template<typename InputIterator>
int Octree<T,E>::add_points(InputIterator begin, InputIterator end) {

    std::cout << "Adding points to octree..." << std::endl;

    Node* n;
    int total_octants = 0;
    int sz = 0;
    int count = 0;
    for (auto it=begin; it != end; ++it) {

        E e = *it;

        // Find the Node that this element fits in
        n = bounding_node(e);
    
        // if the node is not a leaf node, find all the children
        // that it intersects
        std::vector<Node*> int_nodes;
        if (n->is_leaf()) {
            int_nodes.push_back(n);
        } else {
            std::queue<Node*> check_nodes;
            check_nodes.push(n);
            while (!check_nodes.empty()) {
                Node* n_check = check_nodes.front();
                check_nodes.pop();
                if ( (n_check->contains(e)) || (n_check->elem_intersects(e)) ) {
                    if (n_check->is_leaf()) {
                        int_nodes.push_back(n_check);
                    } else {
                        for (unsigned int i=0; i < 8; ++i) {
                            check_nodes.push(n_check->child(i));
                        }
                    }
                }
            }
        }
        //assert(int_nodes.size() > 0); // without this could be dropping nodes/tris
        if (int_nodes.size() == 0) {
            std::cout << "Completely dropped element " << count << std::endl;
        }

        // Add this element to all leaf nodes
        for (unsigned int i=0; i < int_nodes.size(); ++i) {

            n = int_nodes[i];            

            n->elems.push_back( e.get_idx() );
    
            // If this Node now exceeds the maximum allowable points per node, divide
            // allow up to 5 attempts for the division to succeed
            if ( (n->elems.size() > max_per_node_) && 
                 (n->elems.size() <= (max_per_node_+5))) {
                // only subdivide if depth would not go beyond MAX_DEPTH
                Node* tmp = n;
                int count2 = 0;
                while (tmp->parent != nullptr) {
                    tmp = tmp->parent;
                    ++count2;
                }

                if (count2 <= MAX_DEPTH) {
                    divide(n);
                    total_octants += 8;

                    if (count2 > depth_) {
                        depth_ = count2;
                    }
                }
            }
        }
        ++count;
        if (count % 50000 == 0) {
            std::cout << "  finished " << count << " elems" << std::endl;
        }
    }

    if (!(count % 50000 == 0)) {
        std::cout << "  finished " << count << " elems" << std::endl;
    }

    return total_octants;
}

/*****************************************************************************/
template<typename T, typename E>
void Octree<T,E>::sort_points() {

    Node* n;
    for (auto it=bf_begin(); it != bf_end(); ++it) {
        n = *it;
        if (n->is_leaf()) {
            std::sort(n->elems.begin(),n->elems.end());
        }
    }

}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::Node* Octree<T,E>::bounding_node(const E& elem) const {

    Node* n = root_;
    assert(n->contains(elem)); // root node must contain the element

    // Find the smallest possible node that contains this element
    // may not be a leaf node
    while (!n->is_leaf()) {
        bool found_child = false;
        for (int i=0; i < 8; ++i) {
            if (n->child(i)->contains(elem)) {
                n = n->child(i);
                found_child = true;
                break;
            }
        }
        // if it is not contained in the child, then it is bound by the current
        // non-leaf node
        if (!found_child) {
            break;
        }
    }

    return n;
}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::Node* Octree<T,E>::bounding_node(cv::Point3_<FP> pt) const {

    Node* n = root_;
    assert(n->contains(pt)); // root node must contain the element

    while (!n->is_leaf()) {
        bool found_child = false;
        // Find child that contains this point
        for (int i=0; i < 8; ++i) {
            if (n->child(i)->contains(pt)) {
                n = n->child(i);
                found_child = true;
                break;
            }
        }
        // if it is not contained in the child, then it is bound by the current
        // non-leaf node
        if (!found_child) {
            std::cerr << "Failed on point: " << pt << std::endl;
            BoundingBox_<cv::Point3_<FP>> bb = n->get_bounding_box();
            std::cerr << "Parent = " << bb.bounds[0] << " " << bb.bounds[1] << std::endl;
            std::cerr << "Children: " << std::endl;
            for (int i=0; i < 8; ++i) {
                bb = n->child(i)->get_bounding_box();
                std::cerr << i << " " << bb.bounds[0] << " " << bb.bounds[1] << std::endl;
                if (bb.bounds[0].x > pt.x) std::cerr << "Failed on 1" << std::endl;
                if (bb.bounds[1].x <= pt.x) std::cerr << "Failed on 2" << std::endl;
                if (bb.bounds[0].y <= pt.y) std::cerr << "Failed on 3" << std::endl;
                if (bb.bounds[1].y > pt.y) std::cerr << "Failed on 4" << std::endl;
                if (bb.bounds[0].z <= pt.z) std::cerr << "Failed on 5" << std::endl;
                if (bb.bounds[1].z > pt.z) std::cerr << "Failed on 6" << std::endl;

            }
            std::cerr << "Depth " << depth_ << std::endl;
            std::abort();
            break;
        }
    }
    return n;
}

/*****************************************************************************/
template<typename T, typename E>
void Octree<T,E>::write_tree_structure(std::string filename) const {

    // Open file for writing
    std::ofstream fs_out(filename);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write octree"));
    }
        
    fs_out.precision(8);
    fs_out << "TITLE=\"PSP Octree\"" << std::endl;
    fs_out << "VARIABLES=\"X\" \"Y\" \"Z\" \"depth\"" << std::endl;

    // Loop through all nodes and print to file
    std::vector<cv::Point3_<FP>> corners;
    int depth;
    for (auto it = bf_begin(); it != bf_end(); ++it) {
        (*it)->get_corners(corners);
        depth = (*it)->get_depth();
        for (int i=0; i < corners.size(); ++i) {
            fs_out << corners[i].x << " " << corners[i].y << " ";
            fs_out << corners[i].z << " " << depth << std::endl;
        }
    }

    fs_out.close();
}

/*****************************************************************************/
template<typename T, typename E>
void Octree<T,E>::divide(Node* n) {

    // Create room for children
    n->children = new Node[8];

    //std::cout << "Dividing bb = " << (n->get_bounding_box()) << std::endl;
    
    // Create new leaf nodes
    int count = 0;
    for (int i=-1; i < 2; i+=2) {
        for (int j=-1; j < 2; j+=2) {
            for (int k=-1; k < 2; k+=2) {
                cv::Point3_<FP> shift(0.5*i*n->half_width,0.5*j*n->half_width,0.5*k*n->half_width);
                n->children[count].parent = n;
                n->children[count].center = n->center + shift;
                n->children[count].half_width = 0.5 * n->half_width;
                ++count;
            }
        }
    }

    // Fill leaf nodes with points
    for (int p=0; p < n->elems.size(); ++p) {
        Elem e = get_model_node(n->elems[p]);
        unsigned int count = 0;
        for (int i=0; i < 8; ++i) {
            Node* curr = &n->children[i];
            //std::cout << "Testing element " << (n->elems[p]) << " for child overlap:";
            //std::cout << std::endl;
            //std::cout << "Checking intersection of BB " << (curr->get_bounding_box());
            //std::cout << " and triangle " << (e.get_idx()) << std::endl;
            if ( (n->children[i].elem_intersects(e)) ||
                 (n->children[i].contains(e)) ) {
                //std::cout << "   found child" << std::endl;
                n->children[i].elems.push_back(n->elems[p]);
                ++count;
            }
        }
        //assert(count > 0); // without this requirement will be dropping elements
        if (count == 0) {
            std::cout << "Dropped element " << (n->elems[p]) << " on division" << std::endl;
        }
    }

    // remove elems from n and ensure the memory is reallocated
    std::vector<model_idx>().swap(n->elems);
    
    // n is now an inner node
    n->is_leaf_ = false;

}

/********************************************************************* 
 * Node 
 ********************************************************************/

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::Node::Node(Node* parent_in, cv::Point3_<FP> center_in, FP half_width_in) : 
        parent(parent_in), center(center_in), half_width(half_width_in), is_leaf_(true) {
}

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::Node::~Node() {
    if (!this->is_leaf()) {
        delete[] this->children;
    }
}

/*****************************************************************************/
template<typename T, typename E>
int Octree<T,E>::Node::get_depth() const {
    int depth = 0;
    Node* n = this->parent;
    while (n != nullptr) {
        ++depth;
        n = n->parent;
    }
    return depth;
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::Node::contains(const E& elem) const {
    return elem.is_contained(this->get_bounding_box());
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::Node::contains(cv::Point3_<FP> pt) const {
    return ::upsp::contains(this->get_bounding_box(), pt);
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::Node::elem_intersects(const E& elem) const {
    return elem.is_intersected(this->get_bounding_box());
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::Node::intersects(const Ray<FP>& r) const {
    return ::upsp::intersects(this->get_bounding_box(),r);
}

/*****************************************************************************/
template<typename T, typename E>
void Octree<T,E>::Node::get_corners(std::vector<cv::Point3_<FP>>& corners) const {

    corners.resize(8);

    int count = 0;
    for (int i=-1; i < 2; i+=2) {
        for (int j=-1; j < 2; j+=2) {
            for (int k=-1; k < 2; k+=2) {
                cv::Point3_<FP> shift(i*half_width,j*half_width,k*half_width);
                corners[count] = center + shift;
                ++count;
            }
        }
    }
}

/*****************************************************************************/
template<typename T, typename E>
BoundingBox_<cv::Point3_<typename Octree<T,E>::FP>> Octree<T,E>::Node::get_bounding_box() const {
    cv::Point3_<FP> shift(half_width,half_width,half_width);
    return BoundingBox_<cv::Point3_<FP>>(center-shift,center+shift);
}

/********************************************************************* 
 * BreadthFirstIterator
 ********************************************************************/

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::BreadthFirstIterator::BreadthFirstIterator(Node* root) {
    if (root != nullptr) {
        qu_.push(root);
    }
}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::BreadthFirstIterator& Octree<T,E>::BreadthFirstIterator::operator++() {
    // remove current node and add all children
    Node* curr = qu_.front();
    qu_.pop();
    if (!curr->is_leaf()) {
        for (int i=0; i < 8; ++i) {
            qu_.push(curr->child(i));
        }
    }
    return *this;
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::BreadthFirstIterator::operator==(const BreadthFirstIterator& bfi) const {
    if (qu_.empty()) {
        if (bfi.qu_.empty()) {
            return true;
        } else {
            return false;
        }
    } else {
        return qu_ == bfi.qu_;
    }
}

/********************************************************************* 
 * BreadthFirstIterator2 
 ********************************************************************/

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::BreadthFirstIterator2::BreadthFirstIterator2(Node* start) :
        current_(start) {}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::Node* Octree<T,E>::BreadthFirstIterator2::find_next_atdepth(Node* start, int depth) const {

    Node* curr = start;
    Node* par = curr->parent;

    // Initialize stack by getting children above and flipping order 
    std::stack<Node*> tmp_init;
    std::stack<Node*> next_node;
    while (par != nullptr) {
        int child_num = -1;
        // Determine which child is current
        for (int i=0; i < 8; ++i) {
            if (par->child(i) == curr) {
                child_num = i;
                break;
            }
        }
        assert(child_num >= 0);
    
        // add siblings in regular order since it will be flipped
        for (int i=child_num+1; i < 8; ++i) {
            tmp_init.push(par->child(i));
        }

        // move up the tree
        curr = par;
        if (par != nullptr) {
            par = curr->parent;
        }

    }
    while (!tmp_init.empty()) {
        next_node.push(tmp_init.top());
        tmp_init.pop();
    }

    // If starting at root, just add root
    if (start->parent == nullptr) {
        if (depth == 0) {
            return nullptr; // cannot include current node
        }
        next_node.push(curr);
    }

    // Run through stack to find next node at the correct depth
    int d;
    while (!next_node.empty()) {

        curr = next_node.top();
        next_node.pop();
        d = curr->get_depth();

        if (d == depth) {
            return curr;
        } else {
            // add children if it has any
            if (!curr->is_leaf()) {
                for (int i=7; i >= 0; --i) {
                    next_node.push(curr->child(i));
                }
            }
        }
    }

    return nullptr; // no such node exists

}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::BreadthFirstIterator2& Octree<T,E>::BreadthFirstIterator2::operator++() {
    
    // Don't iterate if already nullptr
    if (current_ == nullptr) {
        return *this;
    }

    // Try to get the next node at the current depth
    Node* next = find_next_atdepth(current_,current_->get_depth());

    // If it doesn't exist try next depth
    if (next == nullptr) {
        // get the root node
        next = current_;
        while (next->parent != nullptr) {
            next = next->parent;
        }

        next = find_next_atdepth(next, current_->get_depth()+1);
    }

    current_ = next;
    return *this;

}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::BreadthFirstIterator2::operator==(const BreadthFirstIterator2& bfi) const {
    return (current_ == bfi.current_);
}

/********************************************************************* 
 * DepthFirstIterator 
 ********************************************************************/

/*****************************************************************************/
template<typename T, typename E>
Octree<T,E>::DepthFirstIterator::DepthFirstIterator(Node* start) :
        current_(start), root_(start) {
    //assert(start != nullptr);
}

/*****************************************************************************/
template<typename T, typename E>
typename Octree<T,E>::DepthFirstIterator& Octree<T,E>::DepthFirstIterator::operator++() {

    // Don't iterate if nullptr
    if (current_ == nullptr) {
        return *this;
    }

    // If has a child, go to child
    if ( !(current_->is_leaf()) ) {
        current_ = current_->child(0);
        return *this;
    } 
    
    Node* parent = current_->parent;
    while (parent != (root_->parent)) {
        int child_num = -1;
        // Determine which child is current
        for (int i=0; i < 8; ++i) {
            if (parent->child(i) == current_) {
                child_num = i;
                break;
            }
        }
        assert(child_num >= 0);

        if (child_num < 7) {
            // go to nearest sibling
            current_ = parent->child(child_num+1);
            return *this;
        } else {
            // Must ascend in tree to search additional avenues
            current_ = parent;
            parent = current_->parent;
        }
    }

    current_ = nullptr;  // out of possible nodes
    return *this;
}

/*****************************************************************************/
template<typename T, typename E>
bool Octree<T,E>::DepthFirstIterator::operator==(const DepthFirstIterator& dfi) const {
    if (this->current_ == nullptr) {
        return (dfi.current_ == nullptr);
    } else {
        return ((this->root_ == dfi.root_) && (this->current_ == dfi.current_));
    }
}

} /* end namespace upsp */

