/** @file
 *  @brief  Operations on Grid Models
 *  @date   Nov 15, 2017
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename InputIterator>
BoundingBox_<typename InputIterator::value_type> get_extent(
        InputIterator first, InputIterator last, float buffer /* = 0.0*/) {

    typedef typename InputIterator::value_type::value_type FP;

    if (first == last) {
        return BoundingBox_<cv::Point3_<FP>>();
    }

    cv::Point3_<FP> curr;
    cv::Point3_<FP> max_pt(*first);
    cv::Point3_<FP> min_pt(*first);
    for (auto it=++first; it != last; ++it) {
        curr = *it;
        
        max_pt.x = fmax(max_pt.x, curr.x);
        max_pt.y = fmax(max_pt.y, curr.y);
        max_pt.z = fmax(max_pt.z, curr.z);

        min_pt.x = fmin(min_pt.x, curr.x);
        min_pt.y = fmin(min_pt.y, curr.y);
        min_pt.z = fmin(min_pt.z, curr.z);
    }

    // since bounding box is [), need to push out the max bounds
    max_pt.x = std::nextafter(max_pt.x, std::numeric_limits<FP>::max());
    max_pt.y = std::nextafter(max_pt.y, std::numeric_limits<FP>::max());
    max_pt.z = std::nextafter(max_pt.z, std::numeric_limits<FP>::max());

    // Add buffer
    max_pt = max_pt + cv::Point3_<FP>(buffer, buffer, buffer);
    min_pt = min_pt - cv::Point3_<FP>(buffer, buffer, buffer);

    return BoundingBox_<cv::Point3_<FP>>(min_pt,max_pt);
}

/*****************************************************************************/
template<typename InputIterator>
typename InputIterator::value_type::value_type get_extent(
        InputIterator first, InputIterator last, 
        typename InputIterator::value_type& center, bool add_buffer/*=true*/) {

    typedef typename InputIterator::value_type::value_type FP;

    if (first == last) {
        return 0.0;
    }

    cv::Point3_<FP> max_pt(*first);
    cv::Point3_<FP> min_pt(*first);
    for (auto it=++first; it != last; ++it) {
        center = *it;
        
        max_pt.x = fmax(max_pt.x, center.x);
        max_pt.y = fmax(max_pt.y, center.y);
        max_pt.z = fmax(max_pt.z, center.z);

        min_pt.x = fmin(min_pt.x, center.x);
        min_pt.y = fmin(min_pt.y, center.y);
        min_pt.z = fmin(min_pt.z, center.z);
    }

    // since bounding box is [), need to push out the max bounds
    max_pt.x = std::nextafter(max_pt.x, std::numeric_limits<FP>::max());
    max_pt.y = std::nextafter(max_pt.y, std::numeric_limits<FP>::max());
    max_pt.z = std::nextafter(max_pt.z, std::numeric_limits<FP>::max());
    
    center.x = (max_pt.x - min_pt.x) * 0.5;
    center.y = (max_pt.y - min_pt.y) * 0.5;
    center.z = (max_pt.z - min_pt.z) * 0.5;

    FP half_width = max(center.x, center.y, center.z);

    center.x += min_pt.x;
    center.y += min_pt.y;
    center.z += min_pt.z;

    if (add_buffer) {
        return half_width * 1.01;
    } else {
        return half_width;
    }
}

/*****************************************************************************/
template<typename InputIterator>
upsp::BoundingBox_<typename InputIterator::value_type> get_extent2D(
        InputIterator first, InputIterator last, bool add_buffer/*=true*/) {

    typedef typename InputIterator::value_type::value_type FP;

    if (first == last) {
        return BoundingBox2D();
    }

    cv::Point_<FP> curr;
    cv::Point_<FP> max_pt(*first);
    cv::Point_<FP> min_pt(*first);
    for (auto it=first; it != last; ++it) {
        curr = *it;
        
        max_pt.x = fmax(max_pt.x, curr.x);
        max_pt.y = fmax(max_pt.y, curr.y);

        min_pt.x = fmin(min_pt.x, curr.x);
        min_pt.y = fmin(min_pt.y, curr.y);
    }

    // since bounding box is [), need to push out the max bounds
    max_pt.x = std::nextafter(max_pt.x, std::numeric_limits<FP>::max());
    max_pt.y = std::nextafter(max_pt.y, std::numeric_limits<FP>::max());

    // Add 1% cushion
    if (add_buffer) {
        (max_pt.x < 0.0) ? max_pt.x *= 0.99 : max_pt.x *= 1.01;
        (max_pt.y < 0.0) ? max_pt.y *= 0.99 : max_pt.y *= 1.01;

        (min_pt.x < 0.0) ? min_pt.x *= 1.01 : min_pt.x *= 0.99;
        (min_pt.y < 0.0) ? min_pt.y *= 1.01 : min_pt.y *= 0.99;
    }

    return BoundingBox_<cv::Point_<FP>>(min_pt,max_pt);
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> normal(const Triangle<FP>& tri) {

    cv::Point3_<FP> vec1 = tri.nodes[2] - tri.nodes[1];
    cv::Point3_<FP> normal = vec1.cross( tri.nodes[0] - tri.nodes[1]);
    FP n = cv::norm(normal);
    if (n != 0) {
        return normal / cv::norm(normal);
    } else {
        return normal;
    }
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> normal(const Polygon<FP>& poly) {
    assert(poly.nodes.size() > 2);

    return normal(Triangle<FP>(poly.nodes[0],poly.nodes[1],poly.nodes[2]));
}

/*****************************************************************************/
template<typename FP>
FP area(const Triangle<FP>& tri) {
    FP a = cv::norm(tri.nodes[1] - tri.nodes[0]); 
    FP b = cv::norm(tri.nodes[2] - tri.nodes[1]); 
    FP c = cv::norm(tri.nodes[2] - tri.nodes[0]); 

    // for numerical stability, sort s.t. a >= b >= c
    if (b > a) {
        FP tmp = a;
        a = b;
        b = tmp;
    }
    if (c > a) {
        FP tmp = a;
        a = c;
        c = b;
        b = tmp;
    } else if (c > b) {
        FP tmp = b;
        b = c;
        c = tmp;
    } 

    // could be negative due to small numerical error,
    FP pos_neg = std::abs(c - (a-b));
    return 0.25 * std::sqrt((a + (b+c))*pos_neg*(c + (a-b))*(a + (b-c)));
}

/*****************************************************************************/
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb1, 
        const BoundingBox_<cv::Point3_<FP>>& bb2) {

    if (bb1.bounds[1].x < bb2.bounds[0].x) return false; 
    if (bb1.bounds[0].x > bb2.bounds[1].x) return false;
    if (bb1.bounds[1].y < bb2.bounds[0].y) return false; 
    if (bb1.bounds[0].y > bb2.bounds[1].y) return false;
    if (bb1.bounds[1].z < bb2.bounds[0].z) return false; 
    if (bb1.bounds[0].z > bb2.bounds[1].z) return false;
        
    return true;
}

/*****************************************************************************/
template<typename FP1, typename FP2>
bool intersects(const BoundingBox_<cv::Point3_<FP1>>& bb, 
        const cv::Point3_<FP2>& pt) {

    if (bb.bounds[0].x > pt.x) return false;
    if (bb.bounds[1].x <= pt.x) return false;
    if (bb.bounds[0].y > pt.y) return false;
    if (bb.bounds[1].y <= pt.y) return false;
    if (bb.bounds[0].z > pt.z) return false;
    if (bb.bounds[1].z <= pt.z) return false;

    return true;
}

/*****************************************************************************/
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb, const Plane<FP>& pl) {

    cv::Point3_<FP> c = 0.5*(bb.bounds[0] + bb.bounds[1]);
    cv::Point3_<FP> r = 0.5*(bb.bounds[1] - bb.bounds[0]);
    
    // largest projected distance
    FP rho = std::abs(pl.n[0] * r.x) + std::abs(pl.n[1] * r.y) + std::abs(pl.n[2] * r.z);

    // shortestest distance from center of bb to plane
    FP Dc = shortest_distance(pl, c);

    if (Dc <= rho) {
        return true;
    }
    return false;
}

/*****************************************************************************/
template<typename FP>
FP distance(const BoundingBox_<cv::Point3_<FP>>& bb, 
        const cv::Point3_<FP>& pt) {

    // check if the point is in the bounding box
    if (intersects(bb, pt)) {
        return 0.0;
    }

    // compute distance
    cv::Point3_<FP> range = bb.bounds[1] - bb.bounds[0];
    cv::Point3_<FP> center = (bb.bounds[1] + bb.bounds[0]) / 2.0;
    FP dx = std::max(std::abs(pt.x - center.x) - range.x/2.0, 0.0);
    FP dy = std::max(std::abs(pt.y - center.y) - range.y/2.0, 0.0);
    FP dz = std::max(std::abs(pt.z - center.z) - range.z/2.0, 0.0);

    return std::sqrt( dx*dx + dy*dy + dz*dz );
}

/*****************************************************************************/
template<typename T, typename FP>
void combine_solutions(T& model, std::vector<std::vector<FP>>& sols, 
        std::vector<cv::Point3d>& centers, std::vector<FP>& comb) {

    typedef typename T::Node model_node;
    typedef typename T::node_idx node_idx;

    // Initialize combined solution with NaNs
    comb.assign(model.size(), std::numeric_limits<double>::quiet_NaN());

    cv::Point3d dir;
    model_node n;
    node_idx nidx;
    bool at_least_one;
    FP ang, tot_sol, tot_ang;
    for (auto it=model.node_begin(); it != model.node_end(); ++it) {
        n = *it;
        
        // Get the full index of the node
        nidx = n.get_nidx();

        tot_ang = 0.0;
        tot_sol = 0.0;
        at_least_one = false;
        for (int i=0; i < sols.size(); ++i) {
            // Skip if there is no solution at a point
            if (std::isnan(sols[i][nidx])) {
                continue;
            } else {
                at_least_one = true;
            }

            // Get a vector pointing from the camera center to node
            dir = n.get_position() - centers[i];
            ang = angle_between(dir, n.get_normal());
            
            // add solution to total
            tot_sol += sols[i][nidx] * ang;
            tot_ang += ang;
        }

        if (at_least_one) {
            comb[nidx] = tot_sol / tot_ang;
        }
    }

}

/*****************************************************************************/
template<typename M>
typename M::Node approx_nearest_node(const Octree<M,typename M::Node>& tree, 
        cv::Point3_<typename M::data_type> pt) {

    typedef typename M::Node model_node;
    typedef typename Octree<M,model_node>::Node tree_node;

    // get the tree node that contains the point 
    tree_node* n = tree.bounding_node(pt);
    if (n == nullptr) {
        std::cerr << "octree does not contain point" << std::endl;
        std::abort();
    }

    // if the tree_node contains no model_nodes then take its parent
    // catches invalid octrees
    double min_dist = std::numeric_limits<double>::max();
    double dist;
    model_node closest, mn;
    tree_node* n2;
    if (n->elems.size() == 0) {
        if (n->parent == nullptr) {
            std::cerr << "octree contains no model points" << std::endl;
            std::abort();
        }
        n = n->parent;

        // iterate through all points contained in parent
        for (auto it=n->begin(); it != n->end(); ++it) {
            n2 = *it;
            for (int j=0; j < n2->elems.size(); ++j) {
                mn = tree.get_model_node(n2->elems[j]);
                dist = cv::norm(mn.get_position() - pt);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest = mn;
                }
            }
        }
    } else {
        for (int i=0; i < n->elems.size(); ++i) {
            mn = tree.get_model_node(n->elems[i]);
            dist = cv::norm(mn.get_position() - pt);
            if (dist < min_dist) {
                min_dist = dist;
                closest = mn;
            }
        }
    }

    assert(closest.is_valid());

    return closest;
}

/*****************************************************************************/
template<typename M, typename Pred>
std::vector<typename M::Node> nearest_neighbors_tol(const Octree<M,typename M::Node>& tree, 
        typename M::Node node, float tol, 
        Pred pred /*= AlwaysTrue<typename M::Node>()*/) {

    typedef typename M::Node model_node;
    typedef typename Octree<M,model_node>::Node tree_node;
    typedef typename M::data_type FP;

    cv::Point3_<FP> pos = node.get_position();

    // define structure to hold tree nodes and model nodes and compare 
    struct NeighborCont {
        NeighborCont() {}
        NeighborCont(tree_node* tr_node_in, cv::Point3_<FP> pos) : 
                is_model_node(false), tr_node(tr_node_in) {
            dist = distance(tr_node_in->get_bounding_box(), pos);
        }
        NeighborCont(model_node m_node_in, cv::Point3_<FP> pos) : 
                is_model_node(true), m_node(m_node_in) {
            dist = cv::norm(pos - m_node_in.get_position());
        }

        bool operator>(const NeighborCont& nc) const {
            return this->dist > nc.dist;
        }   
        
        double dist;
        bool is_model_node;
        tree_node* tr_node;
        model_node m_node;
    };

    // Create priority queue and start with root node
    std::priority_queue<NeighborCont,std::vector<NeighborCont>,
            std::greater<NeighborCont>> qu;
    qu.push(NeighborCont(tree.get_root(), pos));

    // Iterate through priority queue until it is either empty or all valid
    // nodes are found
    NeighborCont curr;
    std::vector<model_node> neighbors;
    while (!qu.empty()) {
        curr = qu.top();
        qu.pop();

        if (curr.is_model_node) {
            if (node != curr.m_node) {
                if (pred(curr.m_node)) {
                    if (curr.dist <= tol) {
                        neighbors.push_back(curr.m_node);
                    } else {
                        break;
                    }
                }
            }
        } else {
            if (curr.tr_node->is_leaf()) {
                for (unsigned int i=0; i < curr.tr_node->elems.size(); ++i) {
                    qu.push(NeighborCont(tree.get_model_node(curr.tr_node->elems[i]),pos));
                }
            } else {
                for (unsigned int i=0; i < 8; ++i) {
                    qu.push(NeighborCont(curr.tr_node->child(i),pos));
                }
            }
        }
    }

    return neighbors;

}

/*****************************************************************************/
template<typename M, typename Pred>
typename M::Node nearest_neighbor(const Octree<M,typename M::Node>& tree, typename M::Node node, 
        Pred pred /*= AlwaysTrue<typename M::Node>()*/) {

    typedef typename M::Node model_node;
    typedef typename Octree<M,model_node>::Node tree_node;
    typedef typename M::data_type FP;

    cv::Point3_<FP> pos = node.get_position();

    // define structure to hold tree nodes and model nodes and compare 
    struct NeighborCont {
        NeighborCont() {}
        NeighborCont(tree_node* tr_node_in, cv::Point3_<FP> pos) : 
                is_model_node(false), tr_node(tr_node_in) {
            dist = distance(tr_node_in->get_bounding_box(), pos);
        }
        NeighborCont(model_node m_node_in, cv::Point3_<FP> pos) : 
                is_model_node(true), m_node(m_node_in) {
            dist = cv::norm(pos - m_node_in.get_position());
        }

        bool operator>(const NeighborCont& nc) const {
            return this->dist > nc.dist;
        }   
        
        double dist;
        bool is_model_node;
        tree_node* tr_node;
        model_node m_node;
    };

    // Create priority queue and start with root node
    std::priority_queue<NeighborCont,std::vector<NeighborCont>,
            std::greater<NeighborCont>> qu;
    qu.push(NeighborCont(tree.get_root(), pos));

    // Iterate through priority queue until it is either empty or a valid
    // node is found
    NeighborCont curr;
    while (!qu.empty()) {
        curr = qu.top();
        qu.pop();

        if (curr.is_model_node) {
            if (node != curr.m_node) {
                if (pred(curr.m_node)) {
                    return curr.m_node;
                }
            }
        } else {
            if (curr.tr_node->is_leaf()) {
                for (unsigned int i=0; i < curr.tr_node->elems.size(); ++i) {
                    qu.push(NeighborCont(tree.get_model_node(curr.tr_node->elems[i]),pos));
                }
            } else {
                for (unsigned int i=0; i < 8; ++i) {
                    qu.push(NeighborCont(curr.tr_node->child(i),pos));
                }
            }
        }
    }

    // No node is found that satisfies predicate, return an invalid Node
    model_node mn_out;
    return mn_out;
}

/*****************************************************************************/
template<typename M>
std::vector<typename M::Node> nearest_k_neighbors(const Octree<M,typename M::Node>& tree, 
        cv::Point3_<typename M::data_type> pos, unsigned int k) {
    return nearest_k_neighbors(tree, pos, k, AlwaysTrue<typename M::Node>());
}

/*****************************************************************************/
template<typename M, typename Pred>
std::vector<typename M::Node> nearest_k_neighbors(const Octree<M,typename M::Node>& tree, 
        cv::Point3_<typename M::data_type> pos, unsigned int k, 
        Pred pred) {

    typedef typename M::Node model_node;
    typedef typename Octree<M,model_node>::Node tree_node;
    typedef typename M::data_type FP;

    // define structure to hold tree nodes and model nodes and compare 
    struct NeighborCont {
        NeighborCont() {}
        NeighborCont(tree_node* tr_node_in, cv::Point3_<FP> pos) : 
                is_model_node(false), tr_node(tr_node_in) {
            dist = distance(tr_node_in->get_bounding_box(), pos);
        }
        NeighborCont(model_node m_node_in, cv::Point3_<FP> pos) : 
                is_model_node(true), m_node(m_node_in) {
            dist = cv::norm(pos - m_node_in.get_position());
        }

        bool operator>(const NeighborCont& nc) const {
            return this->dist > nc.dist;
        }   
        
        double dist;
        bool is_model_node;
        tree_node* tr_node;
        model_node m_node;
    };

    // Create priority queue and start with root node
    std::priority_queue<NeighborCont,std::vector<NeighborCont>,
            std::greater<NeighborCont>> qu;
    qu.push(NeighborCont(tree.get_root(), pos));

    // Iterate through priority queue until it is either empty or all valid
    // nodes are found
    NeighborCont curr;
    std::vector<model_node> neighbors;
    while (!qu.empty()) {
        curr = qu.top();
        qu.pop();

        if (curr.is_model_node) {
            if (pred(curr.m_node)) {
                neighbors.push_back(curr.m_node);
                if (neighbors.size() == k) {
                    break;
                }
            }
        } else {
            if (curr.tr_node->is_leaf()) {
                for (unsigned int i=0; i < curr.tr_node->elems.size(); ++i) {
                    qu.push(NeighborCont(tree.get_model_node(curr.tr_node->elems[i]),pos));
                }
            } else {
                for (unsigned int i=0; i < 8; ++i) {
                    qu.push(NeighborCont(curr.tr_node->child(i),pos));
                }
            }
        }
    }

    return neighbors;

}

/*****************************************************************************/
template<typename InputIterator>
void rotate2D(InputIterator first, InputIterator last, double angle, 
        cv::Point2d center) {

    typedef typename std::iterator_traits<InputIterator>::value_type Pt;

    double cosang = std::cos(angle);
    double sinang = std::sin(angle);
    Pt p;
    for (auto it= first; it != last; ++it) {
        p = (*it) - center;
        (*it).x =  cosang * p.x + sinang * p.y + center.x;
        (*it).y = -sinang * p.x + cosang * p.y + center.y; 
    }

}

/*****************************************************************************/
template<typename FP>
FP shortest_distance(const Plane<FP>& pl, const cv::Point3_<FP>& pt) {

    // reformulate as Ax + By + Cz = D
    FP D = -(pl.n[0] * pl.pt[0] + pl.n[1] * pl.pt[1] + pl.n[2] * pl.pt[2]);

    // compute distance
    FP dist = (pl.n[0]*pt.x + pl.n[1]*pt.y + pl.n[2]*pt.z + D) /
              sqrt( pow(pl.n[0],2) + pow(pl.n[1],2) + pow(pl.n[2],2) );
   
    return dist;
}

/*****************************************************************************/
template<typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const cv::Point3_<FP>& pt) {

    if ((pt.x < bb.bounds[0].x) || (pt.x >= bb.bounds[1].x)) {
        return false;
    } else if ((pt.y < bb.bounds[0].y) || (pt.y >= bb.bounds[1].y)) {
        return false;
    } else if ((pt.z < bb.bounds[0].z) || (pt.z >= bb.bounds[1].z)) {
        return false;
    }
    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const cv::Point3_<FP>& pt) {

    // To contain the point, it must be on the opposite side of each plane
    for (unsigned int i=0; i < Size; ++i) {
        if (shortest_distance(poly.planes[i], pt) > poly.tol) {
            return false;
        }
    }

    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const BoundingBox_<cv::Point3_<FP>>& bb) {

    // true, if all nodes of bb are within poly (due to convexity)
    for (unsigned int i=0; i < 2; ++i) {
        FP x = bb.bounds[i].x;
        for (unsigned int j=0; j < 2; ++j) {
            FP y = bb.bounds[j].y;
            for (unsigned int k=0; k < 2; ++k) {
                FP z = bb.bounds[k].z;
                cv::Point3_<FP> pt(x,y,z);
                if (!contains(poly, pt)) {
                    return false;
                }
            }
        }
    }
    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const Polyhedron<Size,FP>& poly) {

    // true if all nodes of poly are within bb (due to convexity)
    // note that a BoundingBox is [x_min,x_max) (also y,z)
    for (auto it = poly.vertices.begin(); it != poly.vertices.end(); ++it) {
        if (!contains(bb, *it)) {
            return false;
        }
    }
    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly_out, const Polyhedron<Size,FP>& poly_in) {

    // true if all nodes of poly_in are within poly_out (due to convexity)
    for (auto it = poly_in.vertices.begin(); it != poly_in.vertices.end(); ++it) {
        if (!contains(poly_out, *it)) {
            return false;
        }
    }
    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool contains(const Polyhedron<Size,FP>& poly, const Triangle<FP>& tri, FP tol/*=-1*/) {

    if (tol < 0.0) {
        tol = poly.tol;
    }

    for (unsigned int i=0; i < 3; ++i) {
        for (unsigned int p=0; p < Size; ++p) {
            if (shortest_distance(poly.planes[p], tri.nodes[i]) > tol) {
                return false;
            }
        }
    }
    return true;
}

/*****************************************************************************/
template<typename FP>
bool contains(const BoundingBox_<cv::Point3_<FP>>& bb, const Triangle<FP>& tri) {
    for (unsigned int i=0; i < 3; ++i) {
        if (!contains(bb, tri.nodes[i])) {
            return false;
        }
    }
    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP>
bool intersects(const Polyhedron<Size,FP>& poly, const BoundingBox_<cv::Point3_<FP>>& bb) {

    // Use Separating Axis Theorem
    // could use some edge culling for speed-up

    assert(poly.edges.size() > 0);
    assert(poly.vertices.size() > 0);
    
    // Shift polyhedron and bb so that bb is centered at origin
    cv::Point3_<FP> c = 0.5*(bb.bounds[0] + bb.bounds[1]);
    cv::Point3_<FP> h = 0.5*(bb.bounds[1] - bb.bounds[0]);

    Polyhedron<Size,FP> poly2 = poly;
    for (unsigned int i=0; i < poly2.vertices.size(); ++i) {
        poly2.vertices[i] -= c;
    }

    BoundingBox_<cv::Point3_<FP>> bb2 = bb;
    bb2.bounds[0] -= c;
    bb2.bounds[1] -= c;

    // adjust bb for positive edge
    bb2.bounds[1].x = std::nextafter(bb2.bounds[1].x, std::numeric_limits<FP>::min()); 
    bb2.bounds[1].y = std::nextafter(bb2.bounds[1].y, std::numeric_limits<FP>::min()); 
    bb2.bounds[1].z = std::nextafter(bb2.bounds[1].z, std::numeric_limits<FP>::min()); 

    // Test BB normals
    BoundingBox_<cv::Point3_<FP>> min_bb = minimal_bound(poly2);
    if (!intersects(bb2, min_bb)) {
        return false;
    }
   
    // Test Poly normals
    for (unsigned int i=0; i < poly2.planes.size(); ++i) {
        cv::Point3_<FP> a;
        a.x = poly2.planes[i].n[0];
        a.y = poly2.planes[i].n[1];
        a.z = poly2.planes[i].n[2];

        // project poly onto a
        FP minp = a.dot(poly2.vertices[0]);
        FP maxp = a.dot(poly2.vertices[0]);
        for (unsigned int k=1; k < poly2.vertices.size(); ++k) {
            minp = std::min(minp, a.dot(poly2.vertices[k]));
            maxp = std::max(maxp, a.dot(poly2.vertices[k]));
        }
        
        // project the box onto a
        FP r = h.x * std::abs(a.x) + h.y * std::abs(a.y) + h.z * std::abs(a.z);

        // axis test
        if ( (minp > r) || (maxp < -r) ) {
            return false;
        }
    }

    // Test Cross-product of edges axes
    // could do some trimming, won't for now
    std::array<cv::Point3_<FP>,3> e;
    e[0] = {1,0,0};
    e[1] = {0,1,0};
    e[2] = {0,0,1};
    for (unsigned int i=0; i < poly2.edges.size(); ++i) {
        for (unsigned int j=0; j < 3; ++j) {
            cv::Point3_<FP> a = e[j].cross(poly2.edges[i]);
            
            // project poly onto a
            FP minp = a.dot(poly2.vertices[0]);
            FP maxp = a.dot(poly2.vertices[0]);
            for (unsigned int k=1; k < poly2.vertices.size(); ++k) {
                minp = std::min(minp, a.dot(poly2.vertices[k]));
                maxp = std::max(maxp, a.dot(poly2.vertices[k]));
            }
            
            // project the box onto a
            FP r = h.x * std::abs(a.x) + h.y * std::abs(a.y) + h.z * std::abs(a.z);
    
            // axis test
            if ( (minp > r) || (maxp < -r) ) {
                return false;
            }

        }
    }

    return true;
}

/*****************************************************************************/
template<size_t Size, typename FP, typename M>
std::set<typename M::node_idx> nodes_within(const Polyhedron<Size,FP>& poly,
                                        const Octree<M,typename M::Node>& tree) {

    typedef typename M::Node Node;
    typedef typename Octree<M,Node>::Node tree_node;
    typedef typename M::node_idx node_idx;

    std::set<node_idx> nodes;

    // If the root node does not intersect the polyhedron, then return empty set
    tree_node* root = tree.get_root();
    if (!intersects(poly, root->get_bounding_box())) {
        return nodes;
    }

    // Find all of the Octree LeafNodes that intersect with the polyhedron
    std::set<tree_node*> t_nodes;
    auto it = tree.bf_begin();
    while (it != tree.bf_end()) {
        auto bb = (*it)->get_bounding_box();
        if (intersects(poly, bb)) {
            if ((*it)->is_leaf()) {
                t_nodes.insert(*it);
            }
            ++it;
        } else {
            it = it.clip(); // do not check descendents of this tree node
        }
    }
            
    // For each Octree Leaf Node, check all contained points to see if they
    // are within the polyhedron
    const M* model = tree.get_model();
    for (auto it = t_nodes.begin(); it != t_nodes.end(); ++it) {
        tree_node* tn = *it;
        if (contains(poly, tn->get_bounding_box())) {
            // add all nodes
            nodes.insert(tn->elems.begin(), tn->elems.end());
        } else {
            // check if point is in the polyhedron
            for (unsigned int i=0; i < tn->elems.size(); ++i) {
                if (contains(poly, model->get_position(tn->elems[i]))) {
                    nodes.insert(tn->elems[i]);
                }
            }
        }
    }

    return nodes;
}

/*****************************************************************************/
template<size_t Size, typename FP, typename M>
std::set<typename M::face_idx> faces_within(const Polyhedron<Size,FP>& poly,
                                        const Octree<M,typename M::Face>& tree) {

    typedef typename M::Face Face;
    typedef typename Octree<M,Face>::Node tree_node;
    typedef typename M::face_idx face_idx;
    typedef Triangle<FP> Triangle;

    std::set<face_idx> faces;

    // If the root node does not intersect the polyhedron, then return empty set
    tree_node* root = tree.get_root();
    if (!intersects(poly, root->get_bounding_box())) {
        return faces;
    }

    // Find all of the Octree LeafNodes that intersect with the polyhedron
    std::set<tree_node*> t_nodes;
    auto it = tree.bf_begin();
    while (it != tree.bf_end()) {
        auto bb = (*it)->get_bounding_box();
        if (intersects(poly, bb)) {
            if ((*it)->is_leaf()) {
                t_nodes.insert(*it);
            }
            ++it;
        } else {
            it = it.clip(); // do not check descendents of this tree node
        }
    }
   
    // Create a list of checked faces, to avoid duplicate work
    std::set<face_idx> checked_faces;

    // Go ahead and triangulate poly if needed
    Polyhedron<Size,FP> poly2(poly);
    if (poly2.tris.size() == 0) {
        poly2.tris = convert_triangles(poly2);
    }

    // For each Octree Leaf Node, check all contained points to see if they
    // are within the polyhedron
    const M* model = tree.get_model();
    for (auto it = t_nodes.begin(); it != t_nodes.end(); ++it) {
        tree_node* tn = *it;
        if (contains(poly2, tn->get_bounding_box())) {
            // add all nodes
            faces.insert(tn->elems.begin(), tn->elems.end());
        } else {
            // check if triangle is in the polyhedron
            for (unsigned int i=0; i < tn->elems.size(); ++i) {
                // triangles may be in multiple leaf nodes, so skip 
                // intersection test if possible
                if (faces.count(tn->elems[i]) == 1) {
                    continue;
                }
                if (checked_faces.count(tn->elems[i]) == 1) {
                    continue;
                }
            
                // Perform the intersection check
                Face f = model->cface(tn->elems[i]);
                Triangle tri = f.get_triangle();
                if ( (contains(poly2, tri)) || (intersects(tri, poly2)) ) {
                    faces.insert(tn->elems[i]);
                } else {
                    checked_faces.insert(tn->elems[i]);
                }
            }
        }
    }

    return faces;
}

/*****************************************************************************/
template<size_t Size, typename FP, typename M>
std::set<typename M::face_idx> approx_faces_within(const Polyhedron<Size,FP>& poly,
                                        const Octree<M,typename M::Face>& tree) {

    typedef typename M::Face Face;
    typedef typename Octree<M,Face>::Node tree_node;
    typedef typename M::face_idx face_idx;

    std::set<face_idx> faces;

    // If the root node does not intersect the polyhedron, then return empty set
    tree_node* root = tree.get_root();
    if (!intersects(poly, root->get_bounding_box())) {
        return faces;
    }

    // Find all of the Octree LeafNodes that intersect with the polyhedron
    // and add faces
    auto it = tree.bf_begin();
    while (it != tree.bf_end()) {
        tree_node* tn = *it;
        auto bb = tn->get_bounding_box();
        //std::cout << "Checking BB " << bb << std::endl;
        if (intersects(poly, bb)) {
            //std::cout << "  intersects" << std::endl;
            if (tn->is_leaf()) {
                //std::cout << "  added " << (tn->elems.size()) << " elements";
                //std::cout << std::endl;
                faces.insert((tn->elems).begin(), (tn->elems).end());
            }
            ++it;
        } else {
            it = it.clip(); // do not check descendents of this tree node
        }
    }
   
    return faces;
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> intersect(const Plane<FP>& pl, const cv::Point3_<FP>& pt1, 
                          const cv::Point3_<FP>& pt2) {
    // solve for t (0->1)
    FP t = intersect_dist(pl, pt1, pt2);

    // evaluate equation for line at t
    cv::Point3_<FP> new_pt = pt1 + t*(pt2 - pt1);

    return new_pt;
}

/*****************************************************************************/
template<typename FP>
FP intersect_dist(const Plane<FP>& pl, const cv::Point3_<FP>& pt1, 
                          const cv::Point3_<FP>& pt2) {
    // reformulate as Ax + By + Cz + D = 0
    FP D = -(pl.n[0] * pl.pt[0] + pl.n[1] * pl.pt[1] + pl.n[2] * pl.pt[2]);
    
    // solve for line segement slopes
    cv::Point3_<FP> slope = pt2 - pt1;

    // solve for t (0->1)
    float t = ( -D - pl.n[0]*pt1.x - pl.n[1]*pt1.y - pl.n[2]*pt1.z ) / 
        ( pl.n[0]*slope.x + pl.n[1]*slope.y + pl.n[2]*slope.z );

    return t;
}

/*****************************************************************************/
template<typename FP>
bool intersects(const Plane<FP>& pl1, const Plane<FP>& pl2, const Plane<FP>& pl3,
        float threshold /*= 0.00001 */) {
    // if the planes intersect at a point, then the rank of the coefficient matrix
    // must equal 3
    // will check rank with the determinant (using threshold due to floating point
    // calculations)   

    // form the coefficient matrix [A B C]
    Eigen::Matrix<FP, 3, 3> mat;
    mat(0,0) = pl1.n[0];
    mat(0,1) = pl1.n[1];
    mat(0,2) = pl1.n[2];
    mat(1,0) = pl2.n[0];
    mat(1,1) = pl2.n[1];
    mat(1,2) = pl2.n[2];
    mat(2,0) = pl3.n[0];
    mat(2,1) = pl3.n[1];
    mat(2,2) = pl3.n[2];

    // check if the matrix is invertible
    FP det = std::abs(mat.determinant());

    if (det > threshold) {
        return true;
    } else {
        return false;
    }
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> intersect(const Plane<FP>& pl1, const Plane<FP>& pl2,
                          const Plane<FP>& pl3) {

    // reformulate each plane as Ax + By + Cz = D
    FP D1 = pl1.n[0] * pl1.pt[0] + pl1.n[1] * pl1.pt[1] + pl1.n[2] * pl1.pt[2];
    FP D2 = pl2.n[0] * pl2.pt[0] + pl2.n[1] * pl2.pt[1] + pl2.n[2] * pl2.pt[2];
    FP D3 = pl3.n[0] * pl3.pt[0] + pl3.n[1] * pl3.pt[1] + pl3.n[2] * pl3.pt[2];
    
    // form the coefficient matrix [A B C]
    Eigen::Matrix<FP, 3, 3> mat;
    mat(0,0) = pl1.n[0];
    mat(0,1) = pl1.n[1];
    mat(0,2) = pl1.n[2];
    mat(1,0) = pl2.n[0];
    mat(1,1) = pl2.n[1];
    mat(1,2) = pl2.n[2];
    mat(2,0) = pl3.n[0];
    mat(2,1) = pl3.n[1];
    mat(2,2) = pl3.n[2];

    // form the vector [D]
    Eigen::Matrix<FP, 3, 1> b;
    b(0) = D1;
    b(1) = D2;
    b(2) = D3;

    // Solve the system, very small so just compute the inverse
    Eigen::Matrix<FP, 3, 1> sol = mat.inverse() * b;

    return {sol(0),sol(1),sol(2)};
}

/*****************************************************************************/
template<typename FP>
std::vector<SplitTri<FP>> split(const Triangle<FP>& tri, const Plane<FP>& pl) {

    std::vector<SplitTri<FP>> new_tris;

    // Check that the plane intersects the triangle
    std::array<FP,3> dist;
    dist[0] = shortest_distance(pl, tri.nodes[0]);
    dist[1] = shortest_distance(pl, tri.nodes[1]);
    dist[2] = shortest_distance(pl, tri.nodes[2]);

    if ( (dist[0] >= 0) && (dist[1] >= 0) && (dist[2] >= 0) ) {
        return new_tris;
    }
    if ( (dist[0] <= 0) && (dist[1] <= 0) && (dist[2] <= 0) ) {
        return new_tris;
    }

    // If it doesn't appear that any nodes are directly intersected
    if ( (dist[0] != 0) && (dist[1] != 0) && (dist[2] != 0) ) {
        // the plane cuts 2 edges, choose to create 3 new triangles

        // create reference nodes based on the specific case
        // use a cyclic transformation, so logic only needs to be coded once
        unsigned int r0 = 0, r1 = 0, r2 = 0;
        if (std::signbit(dist[1]) == std::signbit(dist[2])) {
            r0 = 0;
            r1 = 1;
            r2 = 2;
        } else if (std::signbit(dist[0]) == std::signbit(dist[2])) {
            r0 = 1;
            r1 = 2;
            r2 = 0;
        } else {
            r0 = 2;
            r1 = 0;
            r2 = 1;
        }

        // create new nodes on the split edges
        float t1 = intersect_dist(pl, tri.nodes[r0], tri.nodes[r1]);
        float t2 = intersect_dist(pl, tri.nodes[r0], tri.nodes[r2]);
        cv::Point3_<FP> new_pt1 = tri.nodes[r0] + t1*(tri.nodes[r1] - tri.nodes[r0]);
        cv::Point3_<FP> new_pt2 = tri.nodes[r0] + t2*(tri.nodes[r2] - tri.nodes[r0]);

        //std::cout << "  t1 = " << t1 << ", t2 = " << t2 << std::endl;
        //std::cout << " pt1 = " << new_pt1 << ", pt2 = " << new_pt2 << std::endl;

        // check if these intersections really occur
        // may be very small and intersect_dist shows no intersection
        // while shortest_distance does have intersection
        // let inersect_dist take priority
        bool one_cut = false;
        if (new_pt1 == new_pt2) {
            return new_tris; // tolerance issue, no real intersection
        }
        if ((t1 <= 0.0) || (t1 >= 1.0)) {
            if (t2 >= 1.0) {
                return new_tris;
            }
            if (t1 <= 0.0) {
                dist[r0] = 0.0;
                one_cut = true;
            } else {
                dist[r1] = 0.0;
                one_cut = true;
            }
        }
        if (t2 <= 0.0) {
            dist[r0] = 0.0;
            one_cut = true;
        }
        if (t2 >= 1.0) {
            dist[r2] = 0.0;
            one_cut = true;
        }
            
        if (!one_cut) {   
            // add new triangles
            new_tris.push_back(SplitTri<FP>(tri.nodes[r0], new_pt1, new_pt2));
            new_tris.push_back(SplitTri<FP>(new_pt1, tri.nodes[r1], tri.nodes[r2]));
            new_tris.push_back(SplitTri<FP>(new_pt1, tri.nodes[r2], new_pt2));

            // store information about relationship to old nodes
            new_tris[0].mapping(0,r0) = 1.0;
            new_tris[0].mapping(1,r0) = t1;
            new_tris[0].mapping(1,r1) = 1.0 - t1;
            new_tris[0].mapping(2,r0) = t2;
            new_tris[0].mapping(2,r2) = 1.0 - t2;

            new_tris[1].mapping(0,r0) = t1;
            new_tris[1].mapping(0,r1) = 1.0 - t1;
            new_tris[1].mapping(1,r1) = 1.0;
            new_tris[1].mapping(2,r2) = 1.0;
        
            new_tris[2].mapping(0,r0) = t1;
            new_tris[2].mapping(0,r1) = 1.0 - t1;
            new_tris[2].mapping(1,r2) = 1.0;
            new_tris[2].mapping(2,r0) = t2;
            new_tris[2].mapping(2,r2) = 1.0 - t2;
        }
    }

    // If one of the nodes is on the plane, then just 1 split is needed
    if ( (dist[0] == 0) || (dist[1] == 0) || (dist[2] == 0) ) {
        // create reference nodes based on the specific case
        // use a cyclic transformation, so logic only needs to be coded once
        unsigned int r0 = 0, r1 = 0, r2 = 0;
        if (dist[0] == 0) {
            r0 = 0;
            r1 = 1;
            r2 = 2;
        } else if (dist[1] == 0) {
            r0 = 1;
            r1 = 2;
            r2 = 0;
        } else {
            r0 = 2;
            r1 = 0;
            r2 = 1;
        }

        // find the distance along the edge where the intersection occurs
        float t = intersect_dist(pl, tri.nodes[r1], tri.nodes[r2]);

        // if this distance is outside of (0,1), then intersection does not really
        // occur
        // numerical error may be driving the choice of this path
        if ((t <= 0.0) || (t >= 1.0)) {
            return new_tris;
        }

        // define the new node
        cv::Point3_<FP> new_pt = tri.nodes[r1] + t*(tri.nodes[r2] - tri.nodes[r1]);

        // add new triangles
        new_tris.push_back(SplitTri<FP>(tri.nodes[r0], tri.nodes[r1], new_pt));
        new_tris.push_back(SplitTri<FP>(tri.nodes[r0], new_pt, tri.nodes[r2]));

        // store information about relationship to old nodes
        new_tris[0].mapping(0,r0) = 1.0;
        new_tris[0].mapping(1,r1) = 1.0;
        new_tris[0].mapping(2,r1) = t;
        new_tris[0].mapping(2,r2) = 1.0 - t;
    
        new_tris[1].mapping(0,r0) = 1.0;
        new_tris[1].mapping(1,r1) = t;
        new_tris[1].mapping(1,r2) = 1.0 - t;
        new_tris[1].mapping(2,r2) = 1.0;
    }

    return new_tris;
}

/*****************************************************************************/
template<typename FP, size_t S>
FP shortest_distance(const Polyhedron<S,FP>& poly, const cv::Point3_<FP>& pt) {

    assert(contains(poly, pt));

    FP min_dist = std::numeric_limits<FP>::max();
    for (unsigned int i=0; i < S; ++i) {
        FP dist = std::abs(shortest_distance(poly.planes[i], pt));
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

/*****************************************************************************/
template<typename FP, size_t S>
std::vector<unsigned int> find_intersecting_planes(const Polyhedron<S,FP>& poly,
        const cv::Point3_<FP>& pt) {

    std::vector<unsigned int> indices;
    for (unsigned int i=0; i < S; ++i) {
        FP dist = std::abs(shortest_distance(poly.planes[i], pt));
        if (dist < poly.tol) {
            indices.push_back(i);
        }
    }
    return indices;
}

/*****************************************************************************/
template<typename FP, size_t S>
std::vector<Triangle<FP>> convert_triangles(const Polyhedron<S,FP>& poly) {

    std::vector<Triangle<FP>> tris;

    // Find the convex hull 
    std::vector<Polygon<FP>> p = convert_polygons(poly);
    
    // For each polygon, triangulate
    for (unsigned int i=0; i < p.size(); ++i) {

        //std::cout << "Polygon " << i << ": " << p[i] << std::endl;
        std::vector<Triangle<FP>> new_tris = triangulate(p[i]);
        //for (unsigned int j=0; j < new_tris.size(); ++j) {
        //    std::cout << "   new Triangles : " << (new_tris[j]) << ", ";
        //}
        //std::cout << std::endl;
        std::copy(new_tris.begin(), new_tris.end(), std::back_inserter(tris));
    }

    return tris;
}

/*****************************************************************************/
template<typename FP, size_t S>
bool intersects(const Triangle<FP>& tri, const Polyhedron<S,FP>& poly) {

    // determine if the polyhedron contains the triangle nodes
    bool has_0 = contains(poly, tri.nodes[0]);
    bool has_1 = contains(poly, tri.nodes[1]);
    bool has_2 = contains(poly, tri.nodes[2]);

    // if the polyhedron contains all 3 nodes
    // then there is no intersection
    if (has_0 && has_1 && has_2) {
        //std::cout << "All nodes within poly" << std::endl;
        return false;
    }

    // Case 1 requires checking all edges for 
    // intersection with the polyhedron
    unsigned int ref = 0;
    bool case_3 = false;
    if (!has_0 && !has_1 && !has_2) {
        //std::cout << "No nodes within poly" << std::endl;
        case_3 = true;
        ref = 0; // doesn't matter which node is ref, want to check all edges
                 // and order doesn't matter
    }

    // Additional Case 1: if 1 node is on boundary, other 2 outside
    // Case 2: if 2 nodes are on boundary, other 1 outside (may also be case 1) 
    bool case_1 = false;
    bool case_2 = false;
    if (has_0 && !has_1 && !has_2) {
        ref = 0;
        case_1 = true;
    } else if (!has_0 && has_1 && !has_2) {
        ref = 1;
        case_1 = true;
    } else if (!has_0 && !has_1 && has_2) {
        ref = 2;
        case_1 = true;
    } else if (has_0 && has_1) {
        ref = 2;
        case_2 = true;
    } else if (has_0 && has_2) {
        ref = 1;
        case_2 = true;
    } else if (has_1 && has_2) {
        ref = 0;
        case_2 = true;
    }

    // test Case 1
    if (case_1 || case_2 || case_3) {
        std::vector<Ray<FP>> rays;
        std::vector<FP> edge_lens;

        // if case_1, first check if the node is on the plane
        // if not, clear intersection
        if (case_1) {
            //std::cout << "Case 1" << std::endl;
            if (std::abs(shortest_distance(poly, tri.nodes[ref])) > poly.tol) {
                return true;
            }
        }

        // if case_2, go ahead and check if the two nodes are on different
        // planes, this would definitely be intersection, but could be difficult
        // to detect with edges due to tolerances
        if (case_2) {
            //std::cout << "Case 2" << std::endl;
            if ( (std::abs(shortest_distance(poly, tri.nodes[(ref+1)%3])) < poly.tol) &&
               (std::abs(shortest_distance(poly, tri.nodes[(ref+2)%3])) < poly.tol) ) {
                //std::cout << "2 nodes on polyhedron" << std::endl;

                // if ref+1 and ref+2 sit on different planes, then there is intersection
                // unless they share at least one plane, then maybe
                std::vector<unsigned int> inter_planes1 = find_intersecting_planes(
                        poly, tri.nodes[(ref+1)%3]);
                std::vector<unsigned int> inter_planes2 = find_intersecting_planes(
                        poly, tri.nodes[(ref+2)%3]);
                bool found_match = false;
                bool found_mismatch = false;
                for (unsigned int i=0; i < inter_planes1.size(); ++i) {
                    for (unsigned int j=0; j < inter_planes2.size(); ++j) {
                        if (inter_planes1[i] == inter_planes2[j]) {
                            found_match = true;
                        } else {
                            found_mismatch = true;
                        }
                    }
                }
                if (found_mismatch && !found_match) {
                    return true;
                }
            } else {
                // two nodes are not on planes, but within poly
                // so clearly intersection
                return true;
            }

            // In this case, ref+1,ref+2 share a plane, so just check for intersection
            // on those 2 edges
            cv::Point3_<FP> dir = tri.nodes[(ref+1)%3] - tri.nodes[ref];
            edge_lens.push_back(cv::norm(dir));
            rays.push_back(Ray<FP>(tri.nodes[ref],dir/edge_lens[0]));
        
            dir = tri.nodes[(ref+2)%3] - tri.nodes[ref];
            edge_lens.push_back(cv::norm(dir));
            rays.push_back(Ray<FP>(tri.nodes[ref],dir/edge_lens[1]));

        } else {

            // test each edge for intersection by the polyhedron
            // start with clearly outside nodes and work in (better for case_1)
            cv::Point3_<FP> dir = tri.nodes[ref] - tri.nodes[(ref+1)%3];
            edge_lens.push_back(cv::norm(dir));
            rays.push_back(Ray<FP>(tri.nodes[(ref+1)%3],dir/edge_lens[0]));
        
            dir = tri.nodes[ref] - tri.nodes[(ref+2)%3];
            edge_lens.push_back(cv::norm(dir));
            rays.push_back(Ray<FP>(tri.nodes[(ref+2)%3],dir/edge_lens[1]));

            dir = tri.nodes[(ref+2)%3] - tri.nodes[(ref+1)%3];
            edge_lens.push_back(cv::norm(dir));
            rays.push_back(Ray<FP>(tri.nodes[(ref+1)%3],dir/edge_lens[2]));
        }

        for (unsigned int i=0; i < rays.size(); ++i) {

            //std::cout << "Ray = " << rays[i] << std::endl;

            // find the distance along the ray where it first
            // intersects the polyhedron
            FP t = intersect_time(poly, rays[i]);
            //std::cout << "intersect time = " << t << " / " << edge_lens[i] << std::endl;

            // if < 0, then no intersection
            if (t < 0.0) {
                continue;
            }

            // if > 0, intersects at some point, 
            // check if it intersects before the ray hits in the end node
            if (t < edge_lens[i]) {
                return true;
            }
        }

        if (case_3) {
            // could test for bounding box intersection first, but in most cases, 
            // this function is going to be called after that has already been
            // performed
            // If all of this has failed, could still have the case where the 
            // triangle basically contains the polyhedron
            // can check this by running the inverse edge test
            // (overall, I think this function deserves serious rework, but
            // for now I'm just concerned with getting it working.  Some of the
            // logic above is important for dealing with possible buildup of
            // numerical error )
            const std::vector<Triangle<FP>>* tris = &poly.tris;
            std::vector<Triangle<FP>> new_tris;
            if (poly.tris.size() == 0) {
                new_tris = convert_triangles(poly);
                tris = &new_tris;
            }

            // will test every edge twice
            for (unsigned int i=0; i < tris->size(); ++i) {
                for (unsigned int j=0; j < 3; ++j) {
                    cv::Point3_<FP> dir = tris->operator[](i).nodes[(j+1)%3] - 
                                          tris->operator[](i).nodes[j];
                    FP edge_len = cv::norm(dir);
                    Ray<FP> ray(tris->operator[](i).nodes[j], dir / edge_len);
            
                    FP t = intersects(tri, ray);

                    // if < 0, then no intersection
                    if (t < 0.0) { 
                        continue;
                    }

                    // if > 0, intersects at some point, 
                    // check if it intersects before the ray hits the end node
                    if (t < edge_len) {
                        return true;
                    }
                }
            }

        }

        return false;
    }

    return true;
}

/*****************************************************************************/
template<typename FP>
bool intersects(const Triangle<FP>& tri, const BoundingBox_<cv::Point3_<FP>>& bb) {
    // Perform the separating axis test

    //typedef double FP;

    //Triangle<double> tri2( tri.nodes[0], tri.nodes[1], tri.nodes[2]);
    //BoundingBox_<cv::Point3_<FP>> bb2(bb.bounds[0], bb.bounds[1]);

    // First shift bb to the origin
    cv::Point3_<FP> c = 0.5*(bb.bounds[0] + bb.bounds[1]);
    cv::Point3_<FP> h = 0.5*(bb.bounds[1] - bb.bounds[0]);

    Triangle<FP> tri2 = tri;
    tri2.nodes[0] -= c;
    tri2.nodes[1] -= c;
    tri2.nodes[2] -= c;

    BoundingBox_<cv::Point3_<FP>> bb2 = bb;
    bb2.bounds[0] -= c;
    bb2.bounds[1] -= c;

    // Create minimal bounding box for triangle and determine if it intersects with 
    // the bb
    // 3 tests
    BoundingBox_<cv::Point3_<FP>> min_bb = minimal_bound(tri2);
    if (!intersects(min_bb, bb2)) {
        //std::cout << "  fails on BB test" << std::endl;
        return false;
    } 
    
    // 9 tests
    std::array<cv::Point3_<FP>,3> e;
    e[0] = {1.0,0.0,0.0};
    e[1] = {0.0,1.0,0.0};
    e[2] = {0.0,0.0,1.0};

    std::array<cv::Point3_<FP>,3> f;
    f[0] = tri2.nodes[1] - tri2.nodes[0];
    f[1] = tri2.nodes[2] - tri2.nodes[1];
    f[2] = tri2.nodes[0] - tri2.nodes[2];

    for (unsigned int i=0; i < 3; ++i) {
        for (unsigned int j=0; j < 3; ++j) {
            // project the triangle onto a
            cv::Point3_<FP> a = e[i].cross(f[j]);
            FP p0 = a.dot(tri2.nodes[0]);
            FP p1 = a.dot(tri2.nodes[1]);
            FP p2 = a.dot(tri2.nodes[2]);
            
            // project the box onto a
            FP r = h.x * std::abs(a.x) + h.y * std::abs(a.y) + h.z * std::abs(a.z);
    
            // axis test
            if ( (min(p0,p1,p2) > r) || (max(p0,p1,p2) < -r) ) {
                //std::cout << "  fails on axes test" << std::endl;
                return false;
            }
        }
    }

    // 1 test : normal of triangle
    cv::Point3_<FP> n = normal(tri2);
    Plane<FP> pl(n, tri2.nodes[0]);
    if (!intersects(bb2, pl)) { 
        //std::cout << "  fails on normal test" << std::endl;
        return false;
    }   

    return true;
}

/*****************************************************************************/
template<typename FP, size_t S>
std::vector<SplitTri<FP>> split(const Triangle<FP>& tri, const Polyhedron<S,FP>& poly) {

    typedef Triangle<FP> Tri;

    std::vector<SplitTri<FP>> new_tris;

    // Check if triangle is split by poly
    if (!intersects(tri, poly)) {
        return new_tris;
    }

    // The triangle is split by the poly, so initialize the new tris with 
    // the original tri
    new_tris.push_back(SplitTri<FP>(tri.nodes[0],tri.nodes[1],tri.nodes[2]));
    new_tris[0].mapping(0,0) = 1.0;
    new_tris[0].mapping(1,1) = 1.0;
    new_tris[0].mapping(2,2) = 1.0;

    // loop through each plane in the polyhedron and split triangles as needed
    for (unsigned int i=0; i < S; ++i) {
        unsigned int curr_size = new_tris.size();
        std::vector<bool> rm_tri(curr_size, false);
        for (unsigned int j=0; j < curr_size; ++j) {
            std::vector<SplitTri<FP>> tmp_tris = split(static_cast<Tri&>(new_tris[j]), 
                    poly.planes[i]);

            // if it is intersected, add new
            if (tmp_tris.size() > 0) {
                rm_tri[j] = true;
            }
            for (unsigned int k=0; k < tmp_tris.size(); ++k) {
                tmp_tris[k].mapping = new_tris[j].mapping * tmp_tris[k].mapping;
                new_tris.push_back(tmp_tris[k]);
            }
        }
        // remove any triangles that were split
        for (int j=curr_size-1; j >= 0; --j) {
            if (rm_tri[j]) {
                //std::cout << "Removing " << j << ": current size = " << new_tris.size();
                //std::cout << std::endl;
                new_tris.erase(new_tris.begin() + j);
            }
        }
    }

    // if new_tris.size() == 1, then this is a tolerance issue, and
    // no intersection occurs
    if (new_tris.size() == 1) {
        new_tris.clear();
    }

    return new_tris;
}

/*****************************************************************************/
template<typename FP, size_t S>
BoundingBox_<cv::Point3_<FP>> minimal_bound(const Polyhedron<S,FP>& poly) {

    assert(poly.vertices.size() > 0);

    BoundingBox_<cv::Point3_<FP>> bb;
    bb.bounds[0] = poly.vertices[0];
    bb.bounds[1] = poly.vertices[0];
    for (unsigned int i=1; i < poly.vertices.size(); ++i) {
        bb.bounds[0].x = std::min(bb.bounds[0].x, poly.vertices[i].x);
        bb.bounds[0].y = std::min(bb.bounds[0].y, poly.vertices[i].y);
        bb.bounds[0].z = std::min(bb.bounds[0].z, poly.vertices[i].z);

        bb.bounds[1].x = std::max(bb.bounds[1].x, poly.vertices[i].x);
        bb.bounds[1].y = std::max(bb.bounds[1].y, poly.vertices[i].y);
        bb.bounds[1].z = std::max(bb.bounds[1].z, poly.vertices[i].z);
    }

    // handle [) edge
    bb.bounds[1].x = std::nextafter(bb.bounds[1].x, std::numeric_limits<FP>::max());
    bb.bounds[1].y = std::nextafter(bb.bounds[1].y, std::numeric_limits<FP>::max());
    bb.bounds[1].z = std::nextafter(bb.bounds[1].z, std::numeric_limits<FP>::max());

    return bb;
}


} /* end namespace upsp */
