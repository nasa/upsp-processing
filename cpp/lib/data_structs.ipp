/** @file
 *  @brief  Simple Data Structure Template Implementations
 *  @date   April 30, 2019
 *  @author jmpowel2
 */

namespace upsp {

/********************************************************************
 * Polyhedron
********************************************************************/

/*****************************************************************************/

/********************************************************************
 * Other Functions
********************************************************************/

/*****************************************************************************/
template<typename FP>
Polyhedron<6,FP> convert(const BoundingBox_<cv::Point3_<FP>>& bb) {
 
    Polyhedron<6,FP> poly;

    poly.planes[0] = Plane<FP>(cv::Point3_<FP>(-1,0,0), bb.bounds[0]);
    poly.planes[1] = Plane<FP>(cv::Point3_<FP>(0,-1,0), bb.bounds[0]);
    poly.planes[2] = Plane<FP>(cv::Point3_<FP>(0,0,-1), bb.bounds[0]);
    poly.planes[3] = Plane<FP>(cv::Point3_<FP>(1,0,0), bb.bounds[1]);
    poly.planes[4] = Plane<FP>(cv::Point3_<FP>(0,1,0), bb.bounds[1]);
    poly.planes[5] = Plane<FP>(cv::Point3_<FP>(0,0,1), bb.bounds[1]);

    // assign the vertices
    for (unsigned int i=0; i < 2; ++i) {
        FP x = bb.bounds[i].x;
        for (unsigned int j=0; j < 2; ++j) {
            FP y = bb.bounds[j].y;
            for (unsigned int k=0; k < 2; ++k) {
                FP z = bb.bounds[k].z;
                poly.vertices.push_back({x,y,z});
            }
        }
    }

    // assign the edge axis directions
    poly.edges.push_back({1,0,0});
    poly.edges.push_back({0,1,0});
    poly.edges.push_back({0,0,1});

    return poly;
}

/*****************************************************************************/
template<typename T>
void FaceMap<T>::add(const T& old_f, const T& new_f) {

    // if this triangle has already been split, just add to the list
    if (has(old_f)) {
        old2new_[old_f].insert(new_f);
        new2old_[new_f] = old_f;
        return;
    }

    // if this triangle is already a new triangle, propagate the information
    if (is_new(old_f)) {
        T parent = new2old_[old_f];
        if (old2new_[parent].count(old_f) > 0) {
            old2new_[parent].erase(old_f);
        }
        old2new_[parent].insert(new_f);
        new2old_[new_f] = parent;
        return;
    }

    // this is a brand new split, add it
    old2new_[old_f] = std::set<T>({new_f});
    new2old_[new_f] = old_f;
}

/*****************************************************************************/
template<typename T>
void FaceMap<T>::merge(const FaceMap<T>& o_fmap) {

    // for each individual mapping, pass along the new splits
    for (auto it = o_fmap.old2new_.begin(); it != o_fmap.old2new_.end(); ++it) {
        T old_f = it->first;

        // if this parent is a child in *this, add it's children to the
        // original parent
        if (is_new(old_f)) {
            T parent = new2old_[old_f];

            const std::set<T>& childs = o_fmap.children(old_f);
            for (auto it2 = childs.cbegin(); it2 != childs.cend(); ++it2) {
                old2new_[parent].insert(*it2);
                new2old_[*it2] = parent;
            }
            old2new_[parent].erase(old_f);
        } else {
            old2new_[old_f] = o_fmap.children(old_f);
            for (auto it2 = old2new_[old_f].begin(); it2 != old2new_[old_f].end(); ++it2) {
                new2old_[*it2] = old_f;
            }
        }
    }
} 

} /* end namespace upsp */
