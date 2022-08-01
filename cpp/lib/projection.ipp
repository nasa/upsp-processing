/** @file
 *  @brief  Project between Image Frame and Model
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */
#include <omp.h>

namespace upsp {

/********************************************************************
 * OccludedPredicate
********************************************************************/

/*****************************************************************************/
template<typename Tr>
OccludedPredicate<Tr>::OccludedPredicate(Tr& tree, FP tol/*=0.0*/) : 
        tree_(tree), tol_(tol) {}

/*****************************************************************************/
template<typename Tr>
bool OccludedPredicate<Tr>::operator()(const model_node& n, 
        const Ray<typename Tr::FP>& r) const {

    typedef typename Tr::FP FP;

    // Traverse the tree to find all nodes that contain the ray
    std::vector<tree_node*> tn;
    ray_trace<Model>(tree_, r, tn);

    // Get the distance from the ray origin to n
    FP ref_dist = cv::norm(n.get_position() - r.origin) - tol_;

    // Remove all tree nodes that are farther than the reference distance
    // (should be able to get this during the ray trace, look into it)
    auto tn_it = tn.begin();
    while (tn_it != tn.end()) {
        float t_box = intersect_time((*tn_it)->get_bounding_box(), r);
        //assert(t_box >= 0.0); need to look into why this isn't always satisfied
        if (t_box > ref_dist) {
            tn_it = tn.erase(tn_it);
        } else {
            ++tn_it;
        }
    }

    // Add all model nodes from each tree node to search
    std::vector<model_idx> nodes;
    for (int i=0; i < tn.size(); ++i) {
        if (!tn[i]->empty()) {
            for (int j=0; j < tn[i]->elems.size(); ++j) {
                nodes.push_back(tn[i]->elems[j]);
            }
        }
    }

    // Add all adjacent faces to a list
    std::vector<Face> faces;
    const Model* model = tree_.get_model();
    for (int i=0; i < nodes.size(); ++i) {
        for (auto it = model->cadj_face_begin(nodes[i]); 
                it != model->cadj_face_end(nodes[i]); ++it) {
            faces.push_back(*it);
        }
    }

    // Remove duplicate faces
    auto fit1 = faces.begin();
    while (fit1 != faces.end()) {
        auto fit2 = fit1+1;
        while (fit2 != faces.end()) {
            if (*fit1 == *fit2) {
                fit2 = faces.erase(fit2);
            } else {
                ++fit2;
            }
        }
        ++fit1;
    }

    // Remove any faces that are adjacent to n
    fit1 = faces.begin();
    while (fit1 != faces.end()) {   
        if ((*fit1).has_node(n)) {
            fit1 = faces.erase(fit1);
        } else {
            ++fit1;
        }   
    }
    
    // Check if any faces are intersected by the ray
    FP dist;
    for (int i=0; i < faces.size(); ++i) {
        dist = faces[i].intersects(r);
        if (dist > 0.0) {
            // check if the intersection occurs before it intersects n
            if (dist < ref_dist) {
                return true;
            }
        }
    }

    return false;
}


/********************************************************************
 * ObliquePredicate
********************************************************************/

/*****************************************************************************/
template<typename Model>
ObliquePredicate<Model>::ObliquePredicate(Model& model, 
        FP max_ang/*=70.0*/) : model_(model), max_ang_(max_ang) {}

/*****************************************************************************/
template<typename Model>
bool ObliquePredicate<Model>::operator()(const model_node& n, 
        const Ray<typename Model::data_type>& r) const {

    typedef typename Model::data_type FP;

    //std::cout << "Nidx = " << n.get_nidx() << "  normal = " << n.get_normal() << std::endl;

    FP ang = 180.0 - rad2_deg(angle_between(n.get_normal(), r.dir));

    return (ang >= max_ang_);
}

/********************************************************************
 * ProjectionPredicate
********************************************************************/

/*****************************************************************************/
template<typename P1, typename P2>
ProjectionPredicate<P1,P2>::ProjectionPredicate(const P1& p1, const P2& p2) : 
        p1_(p1), p2_(p2) {}

/*****************************************************************************/
template<typename P1, typename P2>
bool ProjectionPredicate<P1,P2>::operator()(typename P1::model_node& n, 
        const Ray<typename P1::FP>& r) const {
    if (p1_(n,r)) {
        return true;
    } else {
        return p2_(n,r);
    }
}

/*****************************************************************************/
template<typename P1, typename P2>
ProjectionPredicate<P1,P2> make_combined_predicate(const P1& p1, const P2& p2) {
    return ProjectionPredicate<P1,P2>(p1,p2);
}

/********************************************************************
 * GreaterXPredicate
********************************************************************/

/*****************************************************************************/
template<typename Model>
GreaterXPredicate<Model>::GreaterXPredicate(FP x_max_in) : x_max(x_max_in) {}

/*****************************************************************************/
template<typename Model>
bool GreaterXPredicate<Model>::operator()(const model_node& n, 
        const Ray<FP>& r) const {
    cv::Point3_<FP> pos = n.get_raw_position();
    if (pos.x > x_max) {
        return true;
    } else {
        return false;
    }
}

/********************************************************************
 * VisiblePredicate
********************************************************************/

/*****************************************************************************/
template<typename Model, typename Pred>
VisiblePredicate<Model,Pred>::VisiblePredicate(const CameraCal& cal, 
        const Octree<Model,typename Model::Node>& tree, Pred pred) : 
        cal(cal), tree(tree), pred(pred) {
    sz = cal.size();
    cam_center = cal.get_cam_center();
}

/*****************************************************************************/
template<typename Model, typename Pred>
bool VisiblePredicate<Model,Pred>::operator()(cv::Point3_<FP> pt) const {
    // check that point is in frame
    cv::Point_<FP> img_pt = cal.map_point_to_image(pt);
    if ( (img_pt.x < 0) || (img_pt.y < 0) || (img_pt.x >= sz.width) ||
            (img_pt.y >= sz.height) ) {
        return false;
    }

    // check that point is contained in octree
    if (!tree.contains(pt)) {
        return false;
    }

    // get model node approximately closest to input point
    typename Model::Node n = approx_nearest_node(tree, pt);

    // create ray from camera center to node
    cv::Point3_<FP> dir = n.get_position() - cam_center;
    dir /= cv::norm(dir);
    Ray<FP> r(cam_center, dir);

    // check predicate
    return !pred(n, r);
}
    
/*****************************************************************************/
template<typename Model, typename Pred>
VisiblePredicate<Model, Pred> make_visible_predicate(const CameraCal& cal, 
        const Octree<Model,typename Model::Node>& tree, Pred pred) {
    return VisiblePredicate<Model, Pred>(cal, tree, pred);
}

/********************************************************************
 * BestView
********************************************************************/

/*****************************************************************************/
template<typename T>
std::vector<T> BestView<T>::operator()(const std::vector<T>& angles) const {

    std::vector<T> weights(angles.size(),0);

    if (angles.size() == 0) {
        return weights;
    }

    T max_val = angles[0];
    unsigned int max_idx = 0;
    for (unsigned int i=1; i < angles.size(); ++i) {
        if (angles[i] > max_val) {
            max_idx = i;
            max_val = angles[i];
        }
    }
    weights[max_idx] = 1;

    return weights;
}

/********************************************************************
 * AverageViews
********************************************************************/

/*****************************************************************************/
template<typename T>
std::vector<T> AverageViews<T>::operator()(const std::vector<T>& angles) const {

    std::vector<T> weights(angles.size(),0);

    T ang_sum = 0.0;
    for (unsigned int i=0; i < angles.size(); ++i) {
        ang_sum += angles[i];
    }
    for (unsigned int i=0; i < angles.size(); ++i) {
        weights[i] = angles[i] / ang_sum;
    }

    return weights;
}

/********************************************************************
 * Functions
********************************************************************/

/*****************************************************************************/
template<typename FP>
bool intersects(const BoundingBox_<cv::Point3_<FP>>& bb, 
        const Ray<FP>& r) {

    FP tmin, tmax, tymin, tymax, tzmin, tzmax;

    FP inf = std::numeric_limits<FP>::infinity();

    tmin = (bb.bounds[r.sign[0]].x   - r.origin.x);
    if (tmin != 0.0) {
        tmin *= r.invdir.x;
    }
    tmax = (bb.bounds[1-r.sign[0]].x - r.origin.x) * r.invdir.x;
    tymin = (bb.bounds[r.sign[1]].y   - r.origin.y); 
    if (tymin != 0.0) {
        tymin *= r.invdir.y;
    }
    tymax = (bb.bounds[1-r.sign[1]].y - r.origin.y) * r.invdir.y;

    if ( (tmin > tymax) || (tymin > tmax) ) {
        return false;
    }
    if (tymin > tmin) {
        tmin = tymin;
    }

    // handle NaN on max to not include rays on max edges (order matters)
    tmax = std::min( std::max(-inf, tmax), std::max(-inf, tymax));

    tzmin = (bb.bounds[r.sign[2]].z   - r.origin.z);
    if (tzmin != 0.0) {
        tzmin *= r.invdir.z;
    }
    tzmax = (bb.bounds[1-r.sign[2]].z - r.origin.z) * r.invdir.z;

    if ( (tmin > tzmax) || (tzmin > tmax) ) {
        return false;
    }
    if (tzmin > tmin) {
        tmin = tzmin;
    } 
    
    // handle NaN on max to not include rays on max edges (order matters)
    tmax = std::min(tmax, std::max(-inf, tzmax));

    // handle case of t <= 0 : box is behind the ray 
    // (allow for ray to start in box: tmin < 0)
    if (tmax <= 0) {
        return false;
    }

    return true;
}

/*****************************************************************************/
template<typename FP>
FP intersect_time(const BoundingBox_<cv::Point3_<FP>>& bb, 
        const Ray<FP>& r) {

    FP tmin, tmax, tymin, tymax, tzmin, tzmax;

    FP inf = std::numeric_limits<FP>::infinity();

    tmin = (bb.bounds[r.sign[0]].x   - r.origin.x);
    if (tmin != 0.0) {
        tmin *= r.invdir.x;
    }
    tmax = (bb.bounds[1-r.sign[0]].x - r.origin.x) * r.invdir.x;
    tymin = (bb.bounds[r.sign[1]].y   - r.origin.y); 
    if (tymin != 0.0) {
        tymin *= r.invdir.y;
    }
    tymax = (bb.bounds[1-r.sign[1]].y - r.origin.y) * r.invdir.y;

    if ( (tmin > tymax) || (tymin > tmax) ) {
        return -1.0;
    }
    if (tymin > tmin) {
        tmin = tymin;
    }

    // handle NaN on max to not include rays on max edges (order matters)
    tmax = std::min( std::max(-inf, tmax), std::max(-inf, tymax));

    tzmin = (bb.bounds[r.sign[2]].z   - r.origin.z);
    if (tzmin != 0.0) {
        tzmin *= r.invdir.z;
    }
    tzmax = (bb.bounds[1-r.sign[2]].z - r.origin.z) * r.invdir.z;

    if ( (tmin > tzmax) || (tzmin > tmax) ) {
        return -1.0;
    }
    if (tzmin > tmin) {
        tmin = tzmin;
    } 
    
    // handle NaN on max to not include rays on max edges (order matters)
    tmax = std::min(tmax, std::max(-inf, tzmax));

    // handle case of t <= 0 : box is behind the ray 
    // (allow for ray to start in box: tmin < 0)
    if (tmax <= 0) {
        return -1.0;
    }

    // if the ray origin is within the box, tmax is the intersect time
    if (tmin < 0.0) {
        return tmax;
    } else {
        return tmin;
    }
}

/*****************************************************************************/
template<typename FP, size_t S>
FP intersect_time(const Polyhedron<S,FP>& poly, const Ray<FP>& r) {

    const std::vector<Triangle<FP>>* tris = &poly.tris;

    // For simplicity, triangulate each face of the polyhedron
    // and use intersects(tri, r)
    std::vector<Triangle<FP>> new_tris;
    if (poly.tris.size() == 0) {
        new_tris = convert_triangles(poly);
        tris = &new_tris;
    }

    bool found_time = false;
    FP min_time = std::numeric_limits<FP>::infinity();
    for (unsigned int i=0; i < tris->size(); ++i) {
        FP time = intersects(tris->operator[](i), r);
        //std::cout << "Triangle " << i << ": " << (tris[i]) << std::endl;
        //std::cout << "            intersect time = " << time << std::endl;
        if (time < 0.0) {
            continue;
        }
        min_time = std::min(min_time, time);
        found_time = true;
    }

    if (found_time) {
        return min_time;
    } else {
        return -1.0;
    }
}

/*****************************************************************************/
template<typename FP>
FP intersects(const Triangle<FP>& tri, const Ray<FP>& r) {
    cv::Point3_<FP> e1 = tri.nodes[1] - tri.nodes[0];
    cv::Point3_<FP> e2 = tri.nodes[2] - tri.nodes[0];

    cv::Point3_<FP> pvec = r.dir.cross(e2);
    FP det = e1.dot(pvec);

    // Plane is parallel to the ray
    if (det == 0.0) {
        return -1.0;
    }

    FP inv_det = 1.0 / det;
    cv::Point3_<FP> tvec = r.origin - tri.nodes[0];
    FP u = tvec.dot(pvec) * inv_det;
    if ( (u < 0.0) || (u > 1.0) ) {
        return -1.0;
    }
    
    cv::Point3_<FP> qvec = tvec.cross(e1);
    FP v = r.dir.dot(qvec) * inv_det;
    if ( (v < 0.0) || (u+v > 1.0) ) {
        return -1.0;
    }

    return e2.dot(qvec) * inv_det;
}

/*****************************************************************************/
template<typename Model>
void simple_ray_trace(const Octree<Model,typename Model::Node>& tree, 
        const Ray<typename Model::data_type>& r, 
        std::vector<typename Octree<Model,typename Model::Node>::Node*>& n) {

    typedef typename Octree<Model,typename Model::Node>::Node tree_node;

    // Initialize search stack with root
    std::stack<tree_node*> st;
    st.push(tree.get_root());

    // Check if root is intersected by ray
    if (!st.top()->intersects(r)) {
        return;
    }

    tree_node* curr;
    while (!st.empty()) {
        curr = st.top();
        st.pop();

        if (curr->is_leaf()) {
            n.push_back(curr);
        } else {
            for (int i=0; i < 8; ++i) {
                if (curr->child(i)->intersects(r)) {
                    st.push(curr->child(i));
                }
            }
        }
    }

}

/*****************************************************************************/
template<typename Model>
void ray_trace(const Octree<Model,typename Model::Node>& tree, Ray<typename Model::data_type> ray, 
        std::vector<typename Octree<Model,typename Model::Node>::Node*>& n) {

    typedef typename Model::data_type FP;

    // clear vector
    n.resize(0);

    typedef typename Octree<Model,typename Model::Node>::Node tree_node;

    // Define struct to manage ray/node information
    struct sub_trace {
        sub_trace() {}
        sub_trace(FP tx0,FP ty0, FP tz0, FP tx1, FP ty1,
                FP tz1, tree_node* n_in) : txmin(tx0), tymin(ty0), tzmin(tz0),
                txmax(tx1), tymax(ty1), tzmax(tz1), n(n_in) {}
        FP txmin,tymin,tzmin;
        FP txmax,tymax,tzmax;
        tree_node* n;
    };

    sub_trace curr;
    curr.n = tree.get_root();
    BoundingBox_<cv::Point3_<FP>> bb = curr.n->get_bounding_box();
    Ray<FP> r = ray;

    // If the ray starts inside of the root node, push it outside
    bool in_root = false;
    if (curr.n->contains(r.origin)) {
        in_root = true;
        r.dir *= -1.0;
        FP int_t = intersect_time(bb,r);
        r.origin += r.dir*(int_t + 1.0);
        r.dir *= -1.0;
        in_root = true;
    }

    Ray<FP> r2 = r;

    // reflect the ray on opposite side of bounds if negative components
    // modified from original paper to handle case where octree doesn't have
    // min corner at (0,0,0)
    unsigned char a = 0;
    if (r.dir.x < 0.0) {
        r.origin.x = curr.n->center.x * 2.0 - r.origin.x;
        r.dir.x = -r.dir.x;
        r.invdir.x = -r.invdir.x;
        a |= 4;
    }
    if (r.dir.y < 0.0) {
        r.origin.y = curr.n->center.y * 2.0 - r.origin.y;
        r.dir.y = -r.dir.y;
        r.invdir.y = -r.invdir.y;
        a |= 2;
    }
    if (r.dir.z < 0.0) {
        r.origin.z = curr.n->center.z * 2.0 - r.origin.z;
        r.dir.z = -r.dir.z;
        r.invdir.z = -r.invdir.z;
        a |= 1;
    }

    curr.txmin = (bb.bounds[0].x - r.origin.x);
    if (curr.txmin != 0.0) {
        curr.txmin *= r.invdir.x;
    }
    curr.txmax = (bb.bounds[1].x - r.origin.x) * r.invdir.x;
    curr.tymin = (bb.bounds[0].y - r.origin.y);
    if (curr.tymin != 0.0) {
        curr.tymin *= r.invdir.y;
    }
    curr.tymax = (bb.bounds[1].y - r.origin.y) * r.invdir.y;
    curr.tzmin = (bb.bounds[0].z - r.origin.z);
    if (curr.tzmin != 0.0) {
        curr.tzmin *= r.invdir.z;
    }
    curr.tzmax = (bb.bounds[1].z - r.origin.z) * r.invdir.z;

    // Define function to determine the first child node intersected
    // I think there was an error in the paper, mixing up the planes
    // fast direction for numbering goes ZYX (as paper visuals show)
    auto first_node = [a](sub_trace& curr, FP txm, FP tym, FP tzm) {
        unsigned int plane = max_ind(curr.txmin,curr.tymin,curr.tzmin);
        unsigned char b = 0;
        switch (plane) {
            case 2: // XY Plane
                if (txm < curr.tzmin) {
                    b |= 4;
                }
                if (tym < curr.tzmin) {
                    b |= 2;
                }
                break;
            case 1: // XZ Plane
                if (txm < curr.tymin) {
                    b |= 4;
                }
                if (tzm < curr.tymin) {
                    b |= 1;
                }
                break;
            case 0: // YZ Plane
                if (tym < curr.txmin) {
                    b |= 2;
                } 
                if (tzm < curr.txmin) {
                    b |= 1;
                }
                break;
        }
        return b;
    }; 

    // Define function to determine the next child node intersected
    auto new_node = [a](FP t0, int i0, FP t1, int i1, FP t2, int i2) {
        unsigned int idx = min_ind(t0,t1,t2);
        if (idx == 0) {
            return i0;
        } else if (idx == 1) {
            return i1;
        } else {
            return i2;
        }
    };

    // check that ray intersects root
    if ( max(curr.txmin,curr.tymin,curr.tzmin) >= min(curr.txmax,curr.tymax,curr.tzmax) ) {
        return;
    }

    // Create a queue for traversing tree
    std::queue<sub_trace> qu;
    qu.push(curr);

    int next_node;
    int loop_count = 0;
    while (!qu.empty()) {
        curr = qu.front();
        qu.pop();

        // if the node isn't intersected, next
        if ( (curr.txmax < 0.0) || (curr.tymax < 0.0) || (curr.tzmax < 0.0) ) {
            continue;
        }

        // if it is a leaf, add it
        if (curr.n->is_leaf()) {
            n.push_back(curr.n);
            continue;
        }

        // determine midpoints and check for +inf/-inf (handle NaN)
        FP txm = 0.5 * (curr.txmin + curr.txmax);
        if (std::isnan(txm)) {
            bb = curr.n->get_bounding_box();
            if ( r.origin.x < (0.5*(bb.bounds[0].x + bb.bounds[1].x))) {
                txm = INFINITY;
            } else {
                txm = -INFINITY; 
            }
        }
        FP tym = 0.5 * (curr.tymin + curr.tymax);
        if (std::isnan(tym)) {
            bb = curr.n->get_bounding_box();
            if ( r.origin.y < (0.5*(bb.bounds[0].y + bb.bounds[1].y))) {
                tym = INFINITY; 
            } else {
                tym = -INFINITY; 
            }
        }
        FP tzm = 0.5 * (curr.tzmin + curr.tzmax);
        if (std::isnan(tzm)) {
            bb = curr.n->get_bounding_box();
            if ( r.origin.z < (0.5*(bb.bounds[0].z + bb.bounds[1].z))) {
                tzm = INFINITY;
            } else {
                tzm = -INFINITY; 
            }
        }

        // push all children that are intersected onto stack (automaton)
        next_node = first_node(curr,txm,tym,tzm);
        int count = 0;
        while ( (count == 0) || (next_node < 8) ) {
            switch (next_node) {
                case 0:
                    qu.push(sub_trace(curr.txmin,curr.tymin,curr.tzmin,txm,tym,tzm,
                            curr.n->child(a)));
                    next_node = new_node(txm, 4, tym, 2, tzm, 1);
                    break;
                case 1:
                    qu.push(sub_trace(curr.txmin,curr.tymin,tzm,txm,tym,curr.tzmax,
                            curr.n->child(1^a)));
                    next_node = new_node(txm, 5, tym, 3, curr.tzmax, 8);
                    break;
                case 2:
                    qu.push(sub_trace(curr.txmin,tym,curr.tzmin,txm,curr.tymax,tzm,
                            curr.n->child(2^a)));
                    next_node = new_node(txm, 6, curr.tymax, 8, tzm, 3);
                    break;
                case 3:
                    qu.push(sub_trace(curr.txmin,tym,tzm,txm,curr.tymax,curr.tzmax,
                            curr.n->child(3^a)));
                    next_node = new_node(txm, 7, curr.tymax, 8, curr.tzmax, 8);
                    break;
                case 4:
                    qu.push(sub_trace(txm,curr.tymin,curr.tzmin,curr.txmax,tym,tzm,
                            curr.n->child(4^a)));
                    next_node = new_node(curr.txmax, 8, tym, 6, tzm, 5);
                    break;
                case 5:
                    qu.push(sub_trace(txm,curr.tymin,tzm,curr.txmax,tym,curr.tzmax,
                            curr.n->child(5^a)));
                    next_node = new_node(curr.txmax, 8, tym, 7, curr.tzmax, 8);
                    break;
                case 6:
                    qu.push(sub_trace(txm,tym,curr.tzmin,curr.txmax,curr.tymax,tzm,
                            curr.n->child(6^a)));
                    next_node = new_node(curr.txmax, 8, curr.tymax, 8, tzm, 7);
                    break;
                case 7:
                    qu.push(sub_trace(txm,tym,tzm,curr.txmax,curr.tymax,curr.tzmax,
                            curr.n->child(7^a)));
                    next_node = 8;
                    break;
            }
            ++count;
        }
    }

    // Remove any nodes that are not actually intersected by ray, since ray begins
    // inside of root node
    if (in_root) {
        auto it = n.begin();
        while (it != n.end()) {
            if (!(*it)->intersects(ray)) {
                it = n.erase(it);
                continue;
            }
            ++it;
        }
    }

}

/*****************************************************************************/
template<typename Model, typename Pred, typename FP>
void create_projection_mat(const Model& model, CameraCal& cal, 
        Eigen::SparseMatrix<FP, Eigen::RowMajor>& smat, Pred pred) {

    typedef typename Model::Node model_node;

    typedef Eigen::Triplet<FP> Trip;
    typedef Eigen::SparseMatrix<FP, Eigen::RowMajor> SpMat;

    // Define the size and type of the sparse matrix
    cv::Size f_sz = cal.size();
    smat = SpMat(model.size(), f_sz.width * f_sz.height);
    std::vector<Trip> triplets;

    // Convert 2D image point to 1D coordinate
    auto idx = [f_sz](cv::Point2i pix) {
        return pix.y * f_sz.width + pix.x;
    };

    // Get the camera center for creating rays to model nodes
    cv::Point3_<FP> cam_center = cal.get_cam_center();

    // process each node in the model

    // For parallel processing, we don't make the assumption that
    // the model iterator is random-access. Instead, we just
    // manually stuff a pointer to each node into a std vector,
    // which DOES have random-access.
    typedef typename Model::NodeIterator NodeIterator_t;
    std::vector <NodeIterator_t> ptrArray;
    ptrArray.reserve(model.size());
    for (auto it = model.cnode_begin(); it != model.cnode_end(); ++it) {
      ptrArray.push_back(it);
    }
    unsigned int total_count = 0;
    unsigned int total_accepted_count = 0;

#pragma omp parallel
    {
      if (omp_get_thread_num() == 0) {
        printf("Building projection matrix [%d OpenMP threads]\n",
               omp_get_num_threads());
      }
      std::vector<Trip> local_triplets;
      unsigned int local_total_count = 0;
      unsigned int local_accepted_count = 0;

#pragma omp for schedule(dynamic, 1000)
      for (unsigned int ii = 0; ii < ptrArray.size(); ++ii) {
        ++local_total_count;

        typename Model::Node n = *(ptrArray[ii]);

        // map the 3D model point onto the 2D frame
        cv::Point_<FP> pt = cal.map_point_to_image(n.get_position());

        // skip if the model point does not map onto the 2D frame
        if (!contains(f_sz, pt)) {
            continue;
        }

        // Define the ray from camera center to node
        cv::Point3_<FP> dir = n.get_position() - cam_center;
        dir /= cv::norm(dir);
        Ray<FP> r(cam_center, dir);

        // skip node if predicate returns true
        if (pred(n,r)) {
            continue;
        }

        // Get the closest pixel
        cv::Point2i rpt( round(pt.x), round(pt.y) );

        // Remember this node
        ++total_accepted_count;
        local_triplets.push_back({(int) n.get_nidx(), idx(rpt), 1.0});
      }

#pragma omp critical
      {
        total_count += local_total_count;
        total_accepted_count += local_accepted_count;
        triplets.insert(triplets.end(), local_triplets.begin(),
                        local_triplets.end());
      }
    }

    // Sanity checks
    assert(total_count == ptrArray.size());
    assert(total_accepted_count < total_count);

    printf("Built projection matrix (%d/%d nodes projected)\n",
           total_accepted_count, total_count);

    fflush(stdout);

    // Fill the sparse matrix with triplets
    smat.setFromTriplets(triplets.begin(), triplets.end());
}

/*****************************************************************************/
template<typename T>
void identify_skipped_nodes(Eigen::SparseMatrix<T, Eigen::RowMajor>& smat, 
        std::vector<unsigned int>& skipped) {

    typedef typename Eigen::SparseMatrix<T, Eigen::RowMajor>::InnerIterator SparseIt;

    skipped.clear();

    // For row-major sparse matrix, the outer dimension is the rows
    for (unsigned int i=0; i < smat.outerSize(); ++i) {
        SparseIt it(smat, i);
        if (!it) {
            skipped.push_back(i);
        }
    }

}

/*****************************************************************************/
template<typename T>
void identify_skipped_nodes(std::vector<Eigen::SparseMatrix<T, Eigen::RowMajor>>& smats, 
        std::vector<unsigned int>& skipped) {

    typedef typename Eigen::SparseMatrix<T, Eigen::RowMajor>::InnerIterator SparseIt;

    skipped.clear();

    // For row-major sparse matrix, the outer dimension is the rows
    for (unsigned int i=0; i < smats[0].outerSize(); ++i) {
        bool found_val = false;
        for (unsigned int c=0; c < smats.size(); ++c) {
            SparseIt it(smats[c], i);
            if (it) {
                found_val = true;
                break;
            }
        }
        if (!found_val) {
            skipped.push_back(i);
        }
    }

}

/*****************************************************************************/
template<typename T>
void project_frame(const Eigen::SparseMatrix<T, Eigen::RowMajor>& smat,
        const cv::Mat& frame, std::vector<T>& output) {

    assert(frame.isContinuous());

    cv::Mat_<T> src;
    if (frame.type() != src.type()) {
        frame.convertTo(src, src.type());
    } else {
        src = frame;
    }

    // wrap the cv::Mat with an Eigen map to prevent having to deep copy data
    const T* buf = (T*) src.ptr(0);
    Eigen::Map<const Eigen::Matrix<T,Eigen::Dynamic,1>> m_in(buf, smat.cols());

    // assign 0.0 to all output and resize if necessary
    output.assign(smat.rows(), 0.0);
    //output.resize(smat.rows());
    Eigen::Map<Eigen::Matrix<T,Eigen::Dynamic,1>> m_out(&output[0], smat.rows());

    // perform projection
    m_out = smat * m_in;

}

/*****************************************************************************/
template<typename T, typename FP, typename Op>
void adjust_projection_for_weights(T& model, std::vector<cv::Point3d>& centers, 
        std::vector<Eigen::SparseMatrix<FP, Eigen::RowMajor>>& projs, 
        const Op& weighter) {

    typedef typename T::Node model_node;
    typedef typename T::node_idx node_idx;

    unsigned int cameras = projs.size();

    // Define the structure that holds the iterators in the projection
    // matrices and the camera numbers
    // it also manages the iterators
    struct ProjWeights {
        ProjWeights(T& model_in, Eigen::SparseMatrix<FP, Eigen::RowMajor>& proj_in, 
                cv::Point3d center_in) : row(0), model(model_in), proj(proj_in) {
            center.x = (FP) center_in.x;
            center.y = (FP) center_in.y;
            center.z = (FP) center_in.z;
        
            bool found = false;
            for (; row < proj.outerSize(); ++row) {
                for (typename Eigen::SparseMatrix<FP, Eigen::RowMajor>::InnerIterator 
                        it(proj,row); it; ++it) {
                    found = true;
                    break;
                }
                if (found) {
                    break;
                }
            }
            if (!found) {
                row = -1;
            }
        }

        bool operator>(const ProjWeights& pw) const {
            if (pw.row < 0) {
                return false; // either equal or <
            }
            if (row < 0) {
                return true;
            }
            return this->row > pw.row;
        }   

        bool operator==(const ProjWeights& pw) const {
            if (pw.row < 0) {
                if (row < 0) {
                    return true; // both matrices have been fully traversed
                } else {
                    return false;
                }
            }
            return this->row == pw.row;
        }

        // Move to the next row with a non-zero element
        ProjWeights& operator++() {
            if (row < 0) {
                return *this;
            }
            for (int k=row+1; k < proj.outerSize(); ++k) {
                for (typename Eigen::SparseMatrix<FP, Eigen::RowMajor>::InnerIterator it(proj,k);
                        it; ++it) {
                    row = k;
                    return *this;
                }
            }
            row = -1; // finished iterating through all non-zero elements
            return *this;
        }

        // Get the angle between the current node and the camera
        FP get_angle() const {
            assert(row >= 0);
            model_node n = model.node((node_idx) row);
    
            cv::Point3_<FP> dir = n.get_position() - center;
            return angle_between(dir, n.get_normal());
        }

        // Scale all elements in a row by the scale factor
        void scale_row(FP sf) {
            if (row < 0) {
                return;
            }
            for (typename Eigen::SparseMatrix<FP, Eigen::RowMajor>::InnerIterator it(proj,row); it; 
                    ++it) {
                it.valueRef() *= sf;
            }
        }
    
        int row;
        T& model;
        Eigen::SparseMatrix<FP, Eigen::RowMajor>& proj;
        cv::Point3_<FP> center;
    };

    struct CmpProjWeightsPtrs {
        bool operator()(const ProjWeights* lhs, const ProjWeights* rhs) {
            return lhs->operator>(*rhs);
        }
    };

    // Create priority queue for ordering by node included in projection matrix
    //std::priority_queue<ProjWeights*,std::vector<ProjWeights>,std::greater<ProjWeights>> qu;
    std::priority_queue<ProjWeights*,std::vector<ProjWeights*>,CmpProjWeightsPtrs> qu;

    // Load each camera in the queue
    std::vector<ProjWeights*> pws_out(cameras, nullptr);
    for (unsigned int c=0; c < cameras; ++c) {
        pws_out[c] = new ProjWeights(model, projs[c], centers[c]);
        if (pws_out[c]->row >= 0) {
            qu.push(pws_out[c]);
        }
    }

    // Loop through the priority queue and adjust any points where multiple cameras
    // see a node
    while (!qu.empty()) {
        ProjWeights* pw = qu.top();
        qu.pop();

        if (qu.empty()) {
            break;
        }

        if (qu.top()->operator==(*pw)) {
            // get the angles between each camera and
            // the node
            std::vector<ProjWeights*> pws(1,pw);
            std::vector<FP> angs(1, pw->get_angle());
            while (qu.top()->operator==(*pw)) {
                pws.push_back(qu.top());
                qu.pop();
                angs.push_back(pws[pws.size()-1]->get_angle());
                if (qu.empty()) {
                    break;
                }
            }

            // get the weights for each camera
            std::vector<FP> weights = weighter(angs);

            // scale each camera
            for (unsigned int i=0; i < pws.size(); ++i) {
                pws[i]->scale_row(weights[i]); // scale all elements in the row (1 node)

                // increment and replace in queue
                pws[i]->operator++();
                if ( pws[i]->row >= 0) {
                    qu.push(pws[i]);
                }
            }
        } else {
            // increment and replace in queue
            pw->operator++();
            if (pw->row >= 0) {
                qu.push(pw);
            }
        }
    }

    for (unsigned int c=0; c < cameras; ++c) {
        delete pws_out[c];
    }
}

/*****************************************************************************/
template<typename Model, typename T>
void get_target_diameters(const Octree<Model,typename Model::Node>& tree, const CameraCal& cal,
        const std::vector<T>& targs, std::vector<typename T::data_type>& diams) {
    
    typedef typename T::data_type FP;

    diams.resize(targs.size());

    for (unsigned int i=0; i < targs.size(); ++i) {
        // Check that target is in frame
        if ( (targs[i].diameter == 0.0) || (!contains(cal.size(), targs[i].uv)) ) {
            diams[i] = 0;
            continue;
        }
        
        // Get the model normal at the target using the nearest node
        auto node = approx_nearest_node(tree, targs[i].xyz);
        cv::Point3_<FP> normal = node.get_normal();

        assert(cv::norm(normal) != 0.0);

        // Define a circle in the normal plane
        cv::Point3_<FP> a, b;
        a = get_perpendicular(normal);
        b = a.cross(normal);

        // Get 4 points around the circle, project each to image, and 
        // average the diameter
        FP theta = 0.0;
        FP out_diameter = 0.0;
        for (unsigned int j=0; j < 4; ++j) {
            cv::Point3_<FP> est_pt = targs[i].xyz + 0.5*targs[i].diameter*
                    std::cos(theta)*a + 
                    0.5*targs[i].diameter*std::sin(theta)*b;
            cv::Point_<FP> proj_est_pt = cal.map_point_to_image(est_pt);
            out_diameter += 2.0*cv::norm(proj_est_pt - targs[i].uv);
            theta += 2*PI / 4;
        }
        diams[i] = out_diameter / 4.0;
    }
}

/*****************************************************************************/
template<typename T>
void write_projection_matrix(std::string filename, 
        Eigen::SparseMatrix<T, Eigen::RowMajor>& smat) {

    // open file
    std::ofstream ofs(filename, std::ofstream::binary);
    if (!ofs.is_open()) {
        throw(std::invalid_argument("Unable to open file for writing projection matrix"));
    }

    // Write header
    int check = 1;
    ofs.write((char*)&check, sizeof(check));

    int val_size = sizeof(T);
    ofs.write((char*)&val_size, sizeof(val_size));

    int rows = smat.rows();
    int cols = smat.cols();
    ofs.write((char*)&rows, sizeof(rows));
    ofs.write((char*)&cols, sizeof(cols));

    int non_zero = smat.nonZeros();
    ofs.write((char*)&non_zero, sizeof(non_zero));

    // Write triplets (row, col, value)
    for (int k=0; k < smat.outerSize(); ++k) {
        int row = k;
        for (typename Eigen::SparseMatrix<T, Eigen::RowMajor>::InnerIterator 
                it(smat,k); it; ++it) {
            int col = it.col();
            T value = it.value();
            ofs.write((char*)&row, sizeof(row));
            ofs.write((char*)&col, sizeof(col));
            ofs.write((char*)&value, sizeof(value));
        }
    }

    ofs.close();
}

/*****************************************************************************/
template<typename T>
void read_projection_matrix(std::string filename, 
        Eigen::SparseMatrix<T, Eigen::RowMajor>& smat) {

    typedef Eigen::Triplet<T> Trip;
    typedef Eigen::SparseMatrix<T, Eigen::RowMajor> SpMat;

    // open file
    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs.is_open()) {
        throw(std::invalid_argument("Unable to open projection matrix file"));
    }

    // Check that T is the correct value of the matrix
    int check = 0;
    ifs.read((char*)&check, sizeof(check));
    if (check != 1) {
        throw(std::invalid_argument("Unable to parse projection matrix file"));
    }
    
    int val_size = 0;
    ifs.read((char*)&val_size, sizeof(val_size));
    if (val_size != sizeof(T)) {
        throw(std::invalid_argument("SparseMatrix value type is not consistent with file"));
    }

    // Read header
    int rows = 0;
    int cols = 0;
    int non_zeros = 0;
    ifs.read((char*)&rows, sizeof(rows));
    ifs.read((char*)&cols, sizeof(cols));
    ifs.read((char*)&non_zeros, sizeof(non_zeros));

    // Define the size and type of the sparse matrix
    smat = SpMat(rows, cols);

    // Load non-zero values into triplets
    std::vector<Trip> triplets;
    triplets.reserve(non_zeros);
    int row, col;
    T val;
    for (unsigned int i=0; i < non_zeros; ++i) {
        ifs.read((char*)&row, sizeof(row));
        ifs.read((char*)&col, sizeof(col));
        ifs.read((char*)&val, sizeof(T));
        triplets.push_back(Trip(row, col, val));
    }
    
    // Load triplets into sparse matrix
    smat.setFromTriplets(triplets.begin(), triplets.end());

    ifs.close();    
}

} /* end namespace upsp */
