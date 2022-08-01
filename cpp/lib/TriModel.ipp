/** @file
 *  @brief Face Grid Model
 *  @date February 6, 2019
 *  @author jmpowel2
 */

namespace upsp {

/*******************************************************************
 * TriModel_
 *******************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::TriModel_(std::string model_file, bool intersect/*=true*/):
        root_(nullptr) {
    load_grid(model_file, intersect);
}

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::TriModel_(const P3DModel_<FP>& p3d, bool intersect) :
        root_(nullptr),
        x_(p3d.get_x()), y_(p3d.get_y()), z_(p3d.get_z()), n_tris_(0) {

    typedef typename P3DModel_<FP>::Face p_face;
    typedef typename P3DModel_<FP>::node_idx p_nidx;

    n2t_.resize(x_.size());

    // Loop through all faces and add triangles
    for (auto it=p3d.face_begin(); it != p3d.face_end(); ++it) {
         p_face f = *it;

        const std::vector<p_nidx>& nodes = f.get_nodes();

        int comp = f.get_zone();
        face_idx f_id1 = add_face(nodes[0], nodes[1], nodes[2], comp);
        face_idx f_id2 = add_face(nodes[2], nodes[3], nodes[0], comp);

        if (!has_components()) {
            initialize_components();
            set_component(f_id1, comp);
            set_component(f_id2, comp);
        }
    }
    
    generate_kd_tree();

    // Perform intersection
    if (intersect) {
        std::cout << "Finding overlapping points..." << std::endl;
        unsigned int initial_size = size();
        int overlap = intersect_grid();
        unsigned int final_size = size();
        std::cout << "Found " << (initial_size - final_size) << " non-unique points";
        std::cout << std::endl;
        std::cout << "Found " << overlap << " unique overlapping points" << std::endl;
        std::cout << std::endl;
    }
}

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::TriModel_(const UnstructuredGrid<FP>& ugrid, bool intersect/*=false*/):
        root_(nullptr) {
    load_grid(ugrid, intersect);
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::load_grid(const UnstructuredGrid<FP>& ugrid, 
        bool intersect/*=false*/) {

    // Will completely clear out any existing data
    x_ = ugrid.x;
    y_ = ugrid.y;
    z_ = ugrid.z;

    tris_ = ugrid.tris;
    n_tris_ = ugrid.tris.size();

    f2c_ = ugrid.comps;
    valid_tri_.assign(n_tris_, true);
    n2t_.resize(x_.size()); 

    // populate n2t_
    for (unsigned int i=0; i < tris_.size(); ++i) {
        n2t_[tris_[i][0]].insert(i);
        n2t_[tris_[i][1]].insert(i);
        n2t_[tris_[i][2]].insert(i);
    }

    // populate comps_
    for (unsigned int i=0; i < f2c_.size(); ++i) {
        comps_[f2c_[i]].insert(i);
    }

    generate_kd_tree();

    // handle intersection
    if (intersect) {
        std::cout << "Finding overlapping points..." << std::endl;
        unsigned int initial_size = size();
        int overlap = intersect_grid();
        unsigned int final_size = size();
        std::cout << "Found " << (initial_size - final_size) << " non-unique points";
        std::cout << std::endl;
        std::cout << "Found " << overlap << " unique overlapping points" << std::endl;
        std::cout << std::endl;
    }

}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::load_grid(std::string model_file, bool intersect/*=true*/) {

    // open the file
    std::ifstream ifs(model_file, std::ios::in | std::ios::binary);
    if (!ifs) {
        throw(std::invalid_argument("Cannot open tri grid file '" + model_file + "'"));
    }

    // setup Fortran record-length variables
    int32_t sz, sz2;
    
    // read header
    int32_t n_node, n_tri;
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (2*sizeof(int32_t))) {
        throw(std::invalid_argument("Unable to read tri grid file '" + model_file + "'"));
    }
    ifs.read((char*) &n_node, sizeof(int32_t));
    ifs.read((char*) &n_tri, sizeof(int32_t));
    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read tri grid file"));
    }

    assert(n_tri >= 0);
    n_tris_ = n_tri;

    // allocate storage
    x_.resize(n_node);
    y_.resize(n_node);
    z_.resize(n_node);
    tris_.resize(n_tri);

    // read nodes
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (sizeof(float)*3*n_node)) {
        throw(std::invalid_argument("Unable to read tri grid file," 
                                    " inconsistent number of nodes"));
    }

    float x,y,z;
    for (unsigned int i=0; i < n_node; ++i) {
        ifs.read((char*) &x, sizeof(float));
        ifs.read((char*) &y, sizeof(float));
        ifs.read((char*) &z, sizeof(float));

        x_[i] = static_cast<FP>(x);
        y_[i] = static_cast<FP>(y);
        z_[i] = static_cast<FP>(z);
    }

    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read tri grid file,"
                                    " inconsistent number of nodes"));
    }

    // read faces and load tris_ and n2t_
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (sizeof(int32_t)*3*n_tri)) {
        std::cout << sz << " " << n_tri << " " << (sizeof(int32_t)*3*n_tri) << std::endl;
        throw(std::invalid_argument("Unable to read tri grid file,"
                                    " inconsistent number of faces"));
    }

    int32_t n1, n2, n3;
    node_idx nodes[3];
    n2t_.resize(n_node);
    for (unsigned int i=0; i < n_tri; ++i) {
        ifs.read((char*) &n1, sizeof(int32_t));
        ifs.read((char*) &n2, sizeof(int32_t));
        ifs.read((char*) &n3, sizeof(int32_t));

        tris_[i][0] = static_cast<node_idx>(n1-1);
        tris_[i][1] = static_cast<node_idx>(n2-1);
        tris_[i][2] = static_cast<node_idx>(n3-1);
        
        for (unsigned int j=0; j < 3; ++j) {
            n2t_[tris_[i][j]].insert(i);
        }
    }

    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        std::cout << sz << " " << sz2 << " " << n_tri << std::endl;
        throw(std::invalid_argument("Unable to read tri grid file," 
                                    " inconsistent number of faces"));
    }

    // Initially all triangles are valid
    valid_tri_.assign(n_tri, true);

    // Check if this file has multiple components, if so read them
    if (!ifs.eof()) {
        ifs.read((char*) &sz, sizeof(int32_t));
        if (sz != (sizeof(int32_t)*n_tri)) {
            throw(std::invalid_argument("Unable to read tri grid file,"
                                        " inconsistent number of face components"));
        }
        std::vector<int32_t> comp_ids(n_tri);
        ifs.read((char*) &comp_ids[0], sizeof(int32_t)*n_tri);
        ifs.read((char*) &sz2, sizeof(int32_t));
        if (sz != sz2) {
            throw(std::invalid_argument("Unable to read tri grid file,"
                                        " inconsistent number of face components"));
        }

        // copy over to f2c_
        f2c_ = std::vector<int>( comp_ids.begin(), comp_ids.end() );

        // Put component IDs into separate maps for each ID
        //if (comp_ids[n_tri-1] > 1)
        for (unsigned int i=0; i < f2c_.size(); ++i) {
            if (comps_.find(f2c_[i]) == comps_.end()) {
                comps_[f2c_[i]] = std::unordered_set<face_idx>();
            }
            comps_[f2c_[i]].insert(i);
        }
    }

    ifs.close();

    // Describe Grid
    std::cout << "Read " << n_node << " nodes and " << n_tri << " faces";
    std::cout << std::endl;

    generate_kd_tree();

    // Perform intersection
    if (intersect) {
        std::cout << "Finding overlapping points..." << std::endl;
        unsigned int initial_size = size();
        int overlap = intersect_grid();
        unsigned int final_size = size();
        std::cout << "Found " << (initial_size - final_size) << " non-unique points";
        std::cout << std::endl;
        std::cout << "Found " << overlap << " unique overlapping points" << std::endl;
        std::cout << std::endl;
    }

}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::extract_tris(
        std::vector<float>& tris,
        std::vector<int>& triNodes) const {
    typedef cv::Point3_<float> v3f;

    const int nnodes = size();
    std::vector<float> nodepos(nnodes * 3);
    for (int ii = 0; ii < nnodes; ++ii){
        nodepos[ii*3 + 0] = x_[ii];
        nodepos[ii*3 + 1] = y_[ii];
        nodepos[ii*3 + 2] = z_[ii];
    }

    // extract tris
    triNodes.resize(n_tris_ * 3, -1);
    tris.resize(n_tris_ * 9, 0);
    int idx = 0;
    for (auto it = cface_begin(); it != cface_end(); ++it) {
        const auto nodes = (*it).nodes();
        const int i0 = nodes[0];
        const int i1 = nodes[1];
        const int i2 = nodes[2];
        triNodes[idx * 3 + 0] = i0;
        triNodes[idx * 3 + 1] = i1;
        triNodes[idx * 3 + 2] = i2;
        tris[idx * 9 + 0] = nodepos[i0 * 3 + 0];
        tris[idx * 9 + 1] = nodepos[i0 * 3 + 1];
        tris[idx * 9 + 2] = nodepos[i0 * 3 + 2];
        tris[idx * 9 + 3] = nodepos[i1 * 3 + 0];
        tris[idx * 9 + 4] = nodepos[i1 * 3 + 1];
        tris[idx * 9 + 5] = nodepos[i1 * 3 + 2];
        tris[idx * 9 + 6] = nodepos[i2 * 3 + 0];
        tris[idx * 9 + 7] = nodepos[i2 * 3 + 1];
        tris[idx * 9 + 8] = nodepos[i2 * 3 + 2];
        idx++;
    }
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::write_grid(std::string model_file) const {

    // open the file
    std::ofstream ofs(model_file, std::ios::out | std::ios::binary);
    if (!ofs) {
        throw(std::invalid_argument("Cannot open tri grid file '" + model_file + "' for writing"));
    }

    // setup Fortran record-length variables
    int32_t sz;
    
    // write header
    sz = sizeof(int32_t)*2;
    int32_t n_node = static_cast<int32_t>(number_of_nodes());
    int32_t n_tri = static_cast<int32_t>(number_of_faces());

    ofs.write((char*) &sz, sizeof(int32_t));
    ofs.write((char*) &n_node, sizeof(int32_t));
    ofs.write((char*) &n_tri, sizeof(int32_t));
    ofs.write((char*) &sz, sizeof(int32_t));

    // write nodes
    sz = sizeof(float)*n_node*3;
    ofs.write((char*) &sz, sizeof(int32_t));
    float x,y,z;
    for (unsigned int i=0; i < n_node; ++i) {
        x = static_cast<float>(x_[i]);
        y = static_cast<float>(y_[i]);
        z = static_cast<float>(z_[i]);

        ofs.write((char*) &x, sizeof(float));
        ofs.write((char*) &y, sizeof(float));
        ofs.write((char*) &z, sizeof(float));
    }
    ofs.write((char*) &sz, sizeof(int32_t));

    // write faces and compile an ordered list of component IDs
    sz = sizeof(int32_t)*3*n_tri;
    ofs.write((char*) &sz, sizeof(int32_t));
    int32_t n1, n2, n3;
    for (unsigned int i=0; i < valid_tri_.size(); ++i) {
        if (!valid_tri_[i]) {
            continue;
        }

        n1 = static_cast<int32_t>(tris_[i][0]) + 1;
        n2 = static_cast<int32_t>(tris_[i][1]) + 1;
        n3 = static_cast<int32_t>(tris_[i][2]) + 1;

        ofs.write((char*) &n1, sizeof(int32_t));
        ofs.write((char*) &n2, sizeof(int32_t));
        ofs.write((char*) &n3, sizeof(int32_t));
    }
    ofs.write((char*) &sz, sizeof(int32_t));

    // write components
    if (has_components()) {
        sz = sizeof(int32_t)*n_tri;
        ofs.write((char*) &sz, sizeof(int32_t));

        std::vector<int32_t> ord_comps(n_tri);
        unsigned int count = 0;
        for (unsigned int i=0; i < valid_tri_.size(); ++i) {
            if (!valid_tri_[i]) {
                continue;
            }
            ord_comps[count] = f2c_[i];
            ++count;
        }

        ofs.write((char*) &ord_comps[0], sizeof(int32_t)*n_tri);
        ofs.write((char*) &sz, sizeof(int32_t));
    }

    ofs.close();

}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::node_idx TriModel_<FP>::add_node(FP x, FP y, FP z) {
    x_.push_back(x);
    y_.push_back(y);
    z_.push_back(z);

    n2t_.resize(x_.size());
    
    return x_.size() - 1;
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::node_idx TriModel_<FP>::add_node(const cv::Point3_<FP>& pt) {
    return add_node(pt.x,pt.y,pt.z);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::face_idx TriModel_<FP>::add_face(node_idx n1, 
        node_idx n2, node_idx n3, component c_id/*=-1*/) {

    assert(n1 < number_of_nodes());
    assert(n2 < number_of_nodes());
    assert(n3 < number_of_nodes());

    CircularArray<node_idx,3> ca;
    ca[0] = n1;
    ca[1] = n2;
    ca[2] = n3;
    tris_.push_back(ca);
    valid_tri_.push_back(true);
    ++n_tris_;

    face_idx f_idx = tris_.size() - 1;

    if (has_components()) {
        f2c_.push_back(c_id);
        if (comps_.find(c_id) != comps_.end()) {
            comps_[c_id].insert(f_idx);
        } else {
            comps_[c_id] = std::unordered_set<face_idx>({f_idx});
        }
    }

    n2t_[n1].insert(f_idx);
    n2t_[n2].insert(f_idx);
    n2t_[n3].insert(f_idx);

    return f_idx;
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::remove_face(face_idx f) {

    assert(f < valid_tri_.size());

    // face_idx are universal, so just remove flagsand n2t mapping
    --n_tris_;
    valid_tri_[f] = false;
   
    for (unsigned int i=0; i < 3; ++i) {
        n2t_[tris_[f][i]].erase(f);
    }

    // remove from components and remove component if empty
    if ( has_components() ) {
        component comp = f2c_[f];
        comps_[comp].erase(f);
        if ( comps_[comp].empty() ) {
            comps_.erase(comp);
        }
    }
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::initialize_components(component comp_id/*=0*/) {
    boost::counting_iterator<face_idx> cit(0);
    f2c_.assign(tris_.size(), comp_id);
    comps_.clear();
    comps_[comp_id] = std::unordered_set<face_idx>(cit, cit+valid_tri_.size());
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::set_component(face_idx f, component comp_id) {

    if (!is_face(f)) {
        return;
    }

    component old_id = f2c_[f];

    f2c_[f] = comp_id;

    comps_[old_id].erase(f);
    if (comps_.find(comp_id) != comps_.end()) {
        comps_[comp_id].insert(f);
    } else {
        comps_[comp_id] = std::unordered_set<face_idx>({f});
    }

    // check if the old component is now empty
    if (comps_[old_id].empty()) {
        comps_.erase(old_id);
    }

}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Node TriModel_<FP>::node(node_idx n) const {
    return Node(this, n);
}

/*****************************************************************************/
//template<typename FP>
//typename TriModel_<FP>::Node TriModel_<FP>::cnode(node_idx n) const {
//    return Node(this, n);
//}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Edge TriModel_<FP>::edge(node_idx n1, node_idx n2) {
    return Edge(this, n1, n2);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Edge TriModel_<FP>::edge(simple_edge e) const {
    return Edge(this, e);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::face(face_idx t) {
    return Face(this, t);
}

/*****************************************************************************/
template<typename FP>
const typename TriModel_<FP>::Face TriModel_<FP>::cface(face_idx t) const {
    return Face(this, t);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NodeIterator TriModel_<FP>::node_begin() {
    return NodeIterator(this, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NodeIterator TriModel_<FP>::node_end() {
    return NodeIterator(this, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NodeIterator TriModel_<FP>::cnode_begin() const {
    return NodeIterator(this, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NodeIterator TriModel_<FP>::cnode_end() const {
    return NodeIterator(this, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::FaceIterator TriModel_<FP>::face_begin() {
    return FaceIterator(this, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::FaceIterator TriModel_<FP>::face_end() {
    return FaceIterator(this, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::FaceIterator TriModel_<FP>::cface_begin() const {
    return FaceIterator(this, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::FaceIterator TriModel_<FP>::cface_end() const {
    return FaceIterator(this, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompFaceIterator TriModel_<FP>::comp_face_begin(component comp) {
    return CompFaceIterator(this, comp, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompFaceIterator TriModel_<FP>::comp_face_end(component comp) {
    return CompFaceIterator(this, comp, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompFaceIterator 
        TriModel_<FP>::ccomp_face_begin(component comp) const {
    return CompFaceIterator(this, comp, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompFaceIterator 
        TriModel_<FP>::ccomp_face_end(component comp) const {
    return CompFaceIterator(this, comp, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjNodeIterator TriModel_<FP>::adj_node_begin(node_idx n) {
    return AdjNodeIterator(this, n, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjNodeIterator TriModel_<FP>::adj_node_end(node_idx n) {
    return AdjNodeIterator(this, n, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjNodeIterator TriModel_<FP>::cadj_node_begin(node_idx n) const {
    return AdjNodeIterator(this, n, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjNodeIterator TriModel_<FP>::cadj_node_end(node_idx n) const {
    return AdjNodeIterator(this, n, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NAdjFaceIterator TriModel_<FP>::nadj_face_begin(node_idx n) {
    return NAdjFaceIterator(this, n, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NAdjFaceIterator TriModel_<FP>::nadj_face_end(node_idx n) {
    return NAdjFaceIterator(this, n, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NAdjFaceIterator TriModel_<FP>::cnadj_face_begin(node_idx n) const {
    return NAdjFaceIterator(this, n, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NAdjFaceIterator TriModel_<FP>::cnadj_face_end(node_idx n) const {
    return NAdjFaceIterator(this, n, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjFaceIterator TriModel_<FP>::adj_face_begin(face_idx t) {
    return AdjFaceIterator(this, t, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjFaceIterator TriModel_<FP>::adj_face_end(face_idx t) {
    return AdjFaceIterator(this, t, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjFaceIterator TriModel_<FP>::cadj_face_begin(face_idx t) const {
    return AdjFaceIterator(this, t, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjFaceIterator TriModel_<FP>::cadj_face_end(face_idx t) const {
    return AdjFaceIterator(this, t, false);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompAdjFaceIterator TriModel_<FP>::compadj_face_begin(face_idx t) {
    return CompAdjFaceIterator(this, t, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompAdjFaceIterator TriModel_<FP>::compadj_face_end(face_idx t) {
    return CompAdjFaceIterator(this, t, false);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompAdjFaceIterator TriModel_<FP>::ccompadj_face_begin(face_idx t) const {
    return CompAdjFaceIterator(this, t, true);
}
/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompAdjFaceIterator TriModel_<FP>::ccompadj_face_end(face_idx t) const {
    return CompAdjFaceIterator(this, t, false);
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::is_face(node_idx n1, node_idx n2, node_idx n3) {

    // generate the circular array
    CircularArray<node_idx,3> ca;
    ca[0] = n1;
    ca[1] = n2;
    ca[2] = n3;

    // compare this circular array with all faces of v1
    for (auto it=n2t_[n1].begin(); it != n2t_[n1].end(); ++it) {
        if (tris_[*it] == ca) {
            return true;
        }
    }
    return false;
}

/*****************************************************************************/
template<typename FP>
unsigned int TriModel_<FP>::number_of_faces(component comp_id) const {

    if (!has_components()) {
        return 0;
    }

    if (comps_.find(comp_id) != comps_.end()) {
        return comps_.at(comp_id).size();
    }
}

/*****************************************************************************/
template<typename FP>
unsigned int TriModel_<FP>::number_of_adj_faces(face_idx t) const {

    unsigned int count = 0; 
    for (unsigned int i=0; i < 3; ++i) {
        // loop through all faces attached to node i
        for (auto it=n2t_[tris_[t][i]].begin(); it != n2t_[tris_[t][i]].end(); ++it) {
            // skip if it is the central face t
            if (*it == t) {
                continue;
            }

            // check if node i and i+1 share another face
            if (n2t_[tris_[t][i+1]].count(*it) == 1) {
                ++count;
                break;
            }
        }
    }

    return count;
}

/*****************************************************************************/
template<typename FP>
unsigned int TriModel_<FP>::number_of_adj_faces_comp(face_idx t) const {

    if (!has_components()) {
        return number_of_adj_faces(t);
    }

    component c_id = f2c_[t];

    unsigned int count = 0; 
    for (unsigned int i=0; i < 3; ++i) {
        // loop through all faces attached to node i
        for (auto it=n2t_[tris_[t][i]].begin(); it != n2t_[tris_[t][i]].end(); ++it) {
            // skip if it is the central face t
            if (*it == t) {
                continue;
            }

            // skip if part of a different component
            if (f2c_[*it] != c_id) {
                continue;
            }

            // check if node i and i+1 share another face
            if (n2t_[tris_[t][i+1]].count(*it) == 1) {
                ++count;
                break;
            }
        }
    }

    return count;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::is_valid() const {

    if ( ( x_.size() != y_.size() ) || ( x_.size() != z_.size() ) ) {
        std::cerr << "Node position arrays are inconsistent" << std::endl;
        return false;
    }

    if ( ( f2c_.size() > 0 ) && (f2c_.size() != tris_.size() ) ) {
        std::cerr << "Component ID vector is inconsistent" << std::endl;
        return false;
    }

    if ( n2t_.size() != x_.size() ) { 
        std::cerr << "Node2Triangle mapping has wrong number of nodes" << std::endl;
        return false;
    }

    unsigned int valid_t = std::count(valid_tri_.begin(), valid_tri_.end(), true);
    if ( n_tris_ != valid_t ) {
        std::cerr << "Incorrect tracking of number of triangles" << std::endl;
        std::cerr << "n_tris_ = " << n_tris_ << ", but valid_t = " << valid_t << std::endl;
        return false;
    }

    // Check for consistency between tris_ and n2t_
    for (node_idx n=0; n < size(); ++n) {
        for (auto f_it=n2t_[n].begin(); f_it != n2t_[n].end(); ++f_it) {
            face_idx fidx = *f_it;

            if ( (!valid_tri_[fidx]) || (fidx >= tris_.size()) ) {
                std::cerr << "Invalid face in Node2Triangle" << std::endl;
                return false;
            }

            bool found_node = false;
            for (unsigned int i = 0; i < 3; ++i) {
                if (tris_[fidx][i] == n) {
                    found_node = true;
                    break;
                }
            }
            if (!found_node) {
                std::cerr << "Inconsistency between Triangles and Node2Triangle";
                std::cerr << std::endl;
                //std::cerr << "Triangle " << fidx << " Node " << n << std::endl;
                return false;
            }
        }
    }

    if (has_components()) {
        unsigned int total_comps = 0;
        for (auto it = comps_.begin(); it != comps_.end(); ++it) {
            //component c_id = it->first;
            total_comps += (it->second).size();
        }
        if (total_comps != n_tris_) {
            std::cerr << "Component ID mapping inconsistent with number of triangles";
            std::cerr << std::endl;
            std::cerr << "n_tris_ = " << n_tris_ << ", comps_tris = " << total_comps;
            std::cerr << std::endl;
            return false;
        }

        for (auto it = comps_.begin(); it != comps_.end(); ++it) {
            component c_id = it->first;
            for (auto it2 = (it->second).begin(); it2 != (it->second).end(); ++it2) {
                if ( f2c_[*it2] != c_id) {
                    std::cerr << "Inconsistency between Components and Face2Component";
                    std::cerr << std::endl;
                    return false;
                }
            }
        }
    }

    return true;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::has_duplicate_faces() const {

    bool has_dups = false;
    for (unsigned int i=0; i < tris_.size(); ++i) {
        if (!valid_tri_[i]) {
            continue;
        }
        for (unsigned int j=i+1; j < tris_.size(); ++j) {
            if (!valid_tri_[j]) {
                continue;
            }
            if (tris_[i] == tris_[j]) {
                has_dups = true;
                std::cout << "Duplicate Triangles " << i << " and " << j << std::endl;
            }
        }
    }
    return has_dups;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::operator==(const TriModel_<FP>& tri_m) const {
    
    if ( size() != tri_m.size() ) { 
        return false;
    }

    if ( number_of_faces() != tri_m.number_of_faces() ) {
        return false;
    }

    if ( number_of_components() != tri_m.number_of_components() ) {
        return false;
    }

    //FP tol = 1e-8;
    for (unsigned int i=0; i < size(); ++i) {
        if ( (x_[i] != tri_m.x_[i]) || (y_[i] != tri_m.y_[i]) || (z_[i] != tri_m.z_[i]) ) {
            return false;
        }
    }

    for (unsigned int i=0; i < tris_.size(); ++i) {
        if (valid_tri_[i]) {
            if ( tris_[i] != tri_m.tris_[i]) {
                return false;
            }
            if ( f2c_[i] != tri_m.f2c_[i]) {
                return false;
            }
        }
    }

    return true;
}

/*****************************************************************************/
template<typename FP>
void TriModel_<FP>::generate_kd_tree() {

    // so we can store/retrieve node_idx to/from the kdtree which
    // expects void* pointers
    union UserData {
	    void* ptr;
	    uint64_t val;
    };

    UserData udata;
    
    // TODO: possible memory leak if root_ isn't free'd, but
    // kd_clear() seems to crash once in a while. Need to look into that.
    // In general, generate_kd_tree() is not called many times in a given
    // application.
    this->root_ = kd_create(3);

    const auto number_nodes = size();
    for (node_idx nidx = 0; nidx < number_nodes; nidx++) {
	    cv::Point3_<float> pos = get_position(nidx); // "run" coords
	    float x = pos.x;
	    float y = pos.y;
	    float z = pos.z;
	    // send gidx as 'userdata', no need to allocate
	    udata.val = static_cast<uint64_t>(nidx);
	    // add all nodes to main kdtree
	    kd_insert3(this->root_, x, y, z, udata.ptr);
    }
}

/*****************************************************************************/
template<typename FP>
int TriModel_<FP>::intersect_grid() {

    // Create Octree containing all nodes
    Octree<TriModel_<FP>,Node> tree(this, node_begin(), node_end());
    typedef typename Octree<TriModel_<FP>,Node>::Node tree_node;

    // Order all of the points in each leaf in the tree
    tree.sort_points();

    // Search over each leaf for duplicate nodes
    typedef std::pair<node_idx,node_idx> dupl_pair;
    std::vector<std::pair<node_idx,node_idx>> dupls; 
    for (auto it = tree.df_begin(); it != tree.df_end(); ++it) {
        tree_node* tn = *it;

        if (tn->empty()) {
            continue;
        }

        for (unsigned int i=0; i < (tn->elems).size(); ++i) {
            node_idx n1 = (tn->elems)[i];
            for (unsigned int j=i+1; j < (tn->elems).size(); ++j) {
                node_idx n2 = (tn->elems)[j];
                
                if ( (x_[n1] == x_[n2]) && (y_[n1] == y_[n2]) && (z_[n1] == z_[n2]) ) {
                    dupls.push_back(dupl_pair(n1,n2));
                }
            }
        }
    }

    // Define operator< to sort by second in pair, then first
    struct duplpairgreater {
        bool operator()(dupl_pair p1, dupl_pair p2) const {
            if (p1.second == p2.second ) {
                return p1.first < p2.first;
            } else { 
                return p1.second < p2.second;
            }
        }
    } dp_greater;

    // Sort duplicates by second (nodes that will be eliminated)
    std::sort(dupls.begin(), dupls.end(), dp_greater);

    // Remove any duplicates
    unsigned int unique_overlap = dupls.size();
    auto dp_it = dupls.begin();
    while (dp_it != dupls.end()) {
        auto dp_it2 = dp_it + 1;
        unsigned int sz_dup = 0;
        while (dp_it2 != dupls.end()) {
            if (dp_it->second == dp_it2->second) {
                dp_it2 = dupls.erase(dp_it2);
                sz_dup += 1;
            } else {
                break;
            }
        }
        if (sz_dup >= 1) {
            unique_overlap -= sz_dup + 1;
        }
        ++dp_it;
    }

    // Consolidate connectivity information with the new nodes
    // Remove old mapping to triangles
    for (unsigned int i=0; i < dupls.size(); ++i) {
        node_idx n1 = dupls[i].first;
        node_idx n2 = dupls[i].second;

        // move the faces to the new node
        for (auto it2 = n2t_[n2].begin(); it2 != n2t_[n2].end(); ++it2) {
            face_idx f = *it2;
            for (unsigned int i=0; i < 3; ++i) {
                if (tris_[f][i] == n2) {
                    tris_[f][i] = n1;
                }
            }
            n2t_[n1].insert(f);
        }

        n2t_[n2].clear();

        // Determine if there are any collapsed triangles
        // only need to do this after all the nodes that will be 
        // collapsed onto n1 have been
        bool check_collapsed_face = false;
        if (i < (dupls.size()-1)) {
            if (n1 != dupls[i+1].first) {
                check_collapsed_face = true;
            }
        } else {
            check_collapsed_face = true;
        }
        if (check_collapsed_face) {
            std::vector<face_idx> del_faces;
            for (auto it=n2t_[n1].begin(); it != n2t_[n1].end(); ++it) {
                if ( (tris_[*it][0] == tris_[*it][1]) || (tris_[*it][0] == tris_[*it][2]) ||
                     (tris_[*it][1] == tris_[*it][2]) ) {
                    del_faces.push_back(*it);
                }
            }
            for (unsigned int j=0; j < del_faces.size(); ++j) {
                remove_face(del_faces[j]);
            }
        }

    }

    // Add nodes without triangles to the list of duplicates
    // (just link to node 0, shouldn't cause issues since 
    // dupls.first isn't used after this)
    // by collapsing triangles, could introduce orphan nodes,
    // so need to check this after the collapsing
    unsigned int curr_dupl_size = dupls.size();
    for (unsigned int i=0; i < curr_dupl_size; ++i) {
        node_idx start = dupls[i].second + 1;
        node_idx end = 0;
        if (i < (dupls.size() - 1) ) {
            end = dupls[i+1].second;
        } else {
            end = curr_dupl_size;
        }

        for (node_idx n = start; n < end; ++n) {
            if (n2t_[n].size() == 0) {
                dupls.push_back({0,n});
                ++unique_overlap;
            }
        }
    }

    // Re-sort duplicates
    std::sort(dupls.begin(), dupls.end(), dp_greater);

    // Adjust indices for all nodes
    unsigned int adjust = 0;
    for (unsigned int i=0; i < dupls.size(); ++i) {
        node_idx start = dupls[i].second + 1;
        node_idx end = 0;
        if (i < (dupls.size() - 1) ) {
            end = dupls[i+1].second + 1;
        } else {
            end = size();
        }
        ++adjust;
        for (node_idx n=start; n < end; ++n) {
            node_idx new_n = n - adjust;

            for (auto it2 = n2t_[n].begin(); it2 != n2t_[n].end(); ++it2) {
                face_idx f = *it2;
                for (unsigned int j=0; j < 3; ++j) {
                    if (tris_[f][j] == n) {
                        tris_[f][j] = new_n;
                    }
                }
            }
        }
    }

    if (dupls.size() == 0) {
        return unique_overlap;
    }

    // remove nodes
    std::vector<unsigned int> idcs(dupls.size(), 0);
    for (unsigned int i=0; i < dupls.size(); ++i) {
        idcs[i] = dupls[i].second;
    }
    x_   = erase_indices(x_, idcs);
    y_   = erase_indices(y_, idcs);
    z_   = erase_indices(z_, idcs);
    n2t_ = erase_indices(n2t_, idcs);

    if (!is_valid()) {
        std::cout << "Producing invalid tri " << std::endl;
    }

    // Rebuild kdtree after deleting some vertices
    generate_kd_tree();

    return unique_overlap;
}

/*****************************************************************************/
template<typename FP>
FaceMap<typename TriModel_<FP>::face_idx> TriModel_<FP>::split(face_idx t, 
        const Plane<FP>& pl, bool split_adj/*=true*/) {

    assert(is_face(t));

    //using Model<FP>::set_node_nondata;
    //using Model<FP>::is_datanode;

    FaceMap<face_idx> new_faces;

    // Gather the nodes
    node_idx n1 = tris_[t][0];
    node_idx n2 = tris_[t][1];
    node_idx n3 = tris_[t][2];

    //std::cout << "Attempting to split Triangle (" << n1 << "," << n2;
    //std::cout << "," << n3 << ")" << std::endl;

    // Get the current dataface status
    bool is_data = true;
    if (!using_nodes()) {
        is_data = is_dataface(t);
    }

    // Gather the positions
    cv::Point3_<FP> pt1 = get_position(n1);
    cv::Point3_<FP> pt2 = get_position(n2);
    cv::Point3_<FP> pt3 = get_position(n3);

    // Form a simple triangle and perform the intersection
    Triangle<FP> tri(pt1,pt2,pt3);

    std::vector<SplitTri<FP>> new_tris = ::upsp::split(tri, pl);

    // Handle case where the plane does not split the face
    if (new_tris.size() == 0) {
        return new_faces;
    }

    // Get component
    component comp;
    if (has_components()) {
        comp = f2c_[t];
    }

    // Form new nodes
    std::array<int,3> map({-1,-1,-1}); // split (0-1),(1-2),(2-0)
    std::vector<std::array<node_idx,3>> tri_nidx(new_tris.size());
    for (unsigned int i=0; i < new_tris.size(); ++i) {
        for (unsigned int new_n=0; new_n < 3; ++new_n) {
            if (new_tris[i].mapping(new_n,0) > 0.0) {
                if (new_tris[i].mapping(new_n,1) > 0.0) {
                    if (map[0] < 0) {
                        map[0] = add_node(new_tris[i].nodes[new_n]);
                        // Carry on the datanode status
                        if (using_nodes()) {
                            if ((!is_datanode(n1)) || (!is_datanode(n2))) {
                                set_node_nondata(map[0]);
                            }
                        }
                    } 
                    tri_nidx[i][new_n] = map[0];
                } else if (new_tris[i].mapping(new_n,2) > 0.0) {
                    if (map[2] < 0) {    
                        map[2] = add_node(new_tris[i].nodes[new_n]);
                        // Carry on the datanode status
                        if (using_nodes()) {
                            if (!is_datanode(n1) || !is_datanode(n3)) {
                                set_node_nondata(map[2]);
                            }
                        }
                    }
                    tri_nidx[i][new_n] = map[2];
                } else {
                    tri_nidx[i][new_n] = n1;
                }
            } else if (new_tris[i].mapping(new_n,1) > 0.0) {
                if (new_tris[i].mapping(new_n,2) > 0.0) {
                    if (map[1] < 0) {
                        map[1] = add_node(new_tris[i].nodes[new_n]);
                        // Carry on the datanode status
                        if (using_nodes()) {
                            if (!is_datanode(n2) || !is_datanode(n3)) {
                                set_node_nondata(map[1]);
                            }
                        }
                    }
                    tri_nidx[i][new_n] = map[1];
                } else {
                    tri_nidx[i][new_n] = n2;
                }
            } else {
                tri_nidx[i][new_n] = n3;
            }
        }
        //std::cout << "New Tri " << i << " mapping = " << new_tris[i].mapping << std::endl;
    }

    //std::cout << "Split into " << (new_tris.size()) << " new triangles ";
    //std::cout << " with map = " << (map[0]) << ", " << (map[1]) << ", ";
    //std::cout << (map[2]) << std::endl;

    // For any edge that is split, propagate that split to the adjacent
    // triangle if it exists
    if (split_adj) {
        for (unsigned int i=0; i < 3; ++i) {
            if (map[i] < 0) {
                continue;
            }
            //std::cout << "looking for adj here " << i << std::endl;
            //std::cout << "new node = " << (map[i]) << " at : " << get_position(map[i]);
            //std::cout << std::endl;
            // look for an adjacent face
            std::vector<face_idx> edge_matches;
            std::set_intersection(
                    n2t_[tris_[t][i]].begin(), n2t_[tris_[t][i]].end(),
                    n2t_[tris_[t][i+1]].begin(), n2t_[tris_[t][i+1]].end(),
                    std::back_inserter(edge_matches));

            assert(edge_matches.size() > 0);
            //std::cout << "Found " << (edge_matches.size()) << " edge matches" << std::endl;
            //if (edge_matches.size() > 2) {
            //    for (unsigned int j=0; j < edge_matches.size(); ++j) {
            //        std::cout << "   " << (edge_matches[j]);
            //    }
            //    std::cout << std::endl;
            //}
            assert(edge_matches.size() <= 2); // including the current face
            if (edge_matches.size() > 1) {
                face_idx adj_fidx = edge_matches[0];
                if (adj_fidx == t) {
                    adj_fidx = edge_matches[1];
                }                

                // need to split the adj_face into 2 new triangles
                // first find the third node
                // note that for an adjacent triangle, this edge will point
                // in the opposite direction for consistent normals
                // but we are not going to make that assumption here
                simple_tri adj_t = tris_[adj_fidx];
                
                unsigned int ref = 0;
                for (unsigned int j=0; j < 3; ++j) {
                    if ( (adj_t[j] != tris_[t][i]) && (adj_t[j] != tris_[t][i+1])) {
                        ref = j;
                    }
                }

                // create the new triangles and remove the old
                // don't add to the list of new_faces
                // that is reserved for the new faces of ti
                face_idx new_f1 = add_face(adj_t[ref],adj_t[ref+1],map[i]);
                face_idx new_f2 = add_face(adj_t[ref],map[i],adj_t[ref+2]);
                //std::cout << " added adjacent faces " << new_f1 << " and ";
                //std::cout << new_f2 << std::endl;
                new_faces.add(adj_fidx, new_f1);
                new_faces.add(adj_fidx, new_f2);
                if (has_components()) {
                    component adj_comp = get_component(adj_fidx);
                    set_component(new_f1, adj_comp); 
                    set_component(new_f2, adj_comp); 
                }

                // propogate dataface status
                if (!using_nodes()) {
                    bool is_data2 = is_dataface(adj_fidx);
                    if (!is_data2) {
                        set_face_nondata(new_f1);
                        set_face_nondata(new_f2);
                    }
                }

                remove_face(adj_fidx);
                //std::cout << "Removed adjacent face " << adj_fidx << std::endl;
            }
        }
    }

    if (tri_nidx.size() > 0) {
        // Form new triangles
        for (unsigned int i=0; i < tri_nidx.size(); ++i) {
            face_idx new_f = add_face(tri_nidx[i][0],tri_nidx[i][1],tri_nidx[i][2]);
            new_faces.add(t, new_f);
            //std::cout << " added face " << new_f << std::endl;

            if (has_components()) {
                set_component(new_f, comp);
            }

            // propogate dataface status
            if ( (!using_nodes()) && (!is_data) ) {
                set_face_nondata(new_f);
            }
        }
    }

    // Remove the old face
    remove_face(t);

    return new_faces;
}

/*****************************************************************************/
template<typename FP>
template<size_t S>
FaceMap<typename TriModel_<FP>::face_idx> TriModel_<FP>::split(face_idx t, 
        const Polyhedron<S,FP>& poly, bool split_adj/*=true*/) {

    assert(is_face(t));

    FaceMap<face_idx> new_faces;

    // Gather the nodes
    node_idx n1 = tris_[t][0];
    node_idx n2 = tris_[t][1];
    node_idx n3 = tris_[t][2];

    // Gather the positions
    cv::Point3_<FP> pt1 = get_position(n1);
    cv::Point3_<FP> pt2 = get_position(n2);
    cv::Point3_<FP> pt3 = get_position(n3);

    // Form a simple triangle and test for intersection
    Triangle<FP> tri(pt1,pt2,pt3);

    if (!intersects(tri, poly)) {
        //std::cout << "no intersection" << std::endl;
        return new_faces;
    }

    // Use split by plane to perform repeated intersection
    std::set<face_idx> subfaces;
    subfaces.insert(t); // will hold all faces (some not valid), for reference

    std::vector<face_idx> new_tris(1,t); // will hold only valid faces
    for (unsigned int i=0; i < S; ++i) {

        // carry a list of all triangles that have been checked
        std::set<face_idx> checked_tris;

        // throw all of the triangles to consider into a vector
        std::vector<face_idx> check_tris(subfaces.begin(), subfaces.end());    
        
        for (unsigned int c_t=0; c_t < check_tris.size(); ++c_t) {
            face_idx curr_tri = check_tris[c_t];

            // face could have been cut by a previous step, if so just skip
            if (!is_face(curr_tri)) {
                assert(subfaces.count(curr_tri) == 0);
                continue;
            }

            // Perform the split
            FaceMap<face_idx> tmp_faces = split(curr_tri, poly.planes[i], split_adj);

            // if a splitting occurred, handle it
            if (tmp_faces.size() > 0) {
                for (auto f_it=tmp_faces.begin(); f_it != tmp_faces.end(); ++f_it) {
                    face_idx parent = *f_it;

                    // if this is part of the original triangle, 
                    // remove the old and add the new children
                    // also, if not checked, add it to check_tris
                    if (subfaces.count(parent) > 0) {
                        subfaces.erase(parent);
                        const std::set<face_idx>& childs = tmp_faces.children(parent);
                        subfaces.insert(childs.begin(), childs.end());
                        if (parent != curr_tri) {
                            if (checked_tris.count(parent) == 0) {
                                std::copy(childs.begin(), childs.end(), 
                                        std::back_inserter(check_tris));
                            }
                        } else {
                            checked_tris.insert(childs.begin(), childs.end());
                        }
                    }
                }

                // update the overall facemap
                new_faces.merge(tmp_faces);
            }
        }

    }

    return new_faces;
}

/*****************************************************************************/
template<typename FP>
std::set<typename TriModel_<FP>::face_idx> TriModel_<FP>::gather_faces(
        const std::set<node_idx>& nodes) const {

    std::set<face_idx> faces;
    for (auto it=nodes.begin(); it != nodes.end(); ++it) {
        faces.insert(n2t_[*it].begin(), n2t_[*it].end());
    }
    return faces;
}

template<typename FP>
void TriModel_<FP>::calcNormals()
{
  const std::vector<FP>& xpos = this->get_x();
  const std::vector<FP>& ypos = this->get_y();
  const std::vector<FP>& zpos = this->get_z();

  // returns the area of a triangle: 0.5 * || u X v ||
  auto triArea = [this,xpos,ypos,zpos](node_idx n0, node_idx n1, node_idx n2) -> FP {

      FP ux = xpos[ n2 ] - xpos[ n1 ];
      FP uy = ypos[ n2 ] - ypos[ n1 ];
      FP uz = zpos[ n2 ] - zpos[ n1 ];

      FP vx = xpos[ n0 ] - xpos[ n1 ];
      FP vy = ypos[ n0 ] - ypos[ n1 ];
      FP vz = zpos[ n0 ] - zpos[ n1 ];

      FP nx = uy * vz - vy * uz;
      FP ny = vx * uz - ux * vz;
      FP nz = ux * vy - vx * uy;
      return 0.5 * sqrt( nx*nx + ny*ny + nz*nz ); // area = 0.5 || u X v ||

  }; // triArea()

  // calcs the normal in model coords, xforms to "run" coords
  //
  //    n0
  //     |
  //    n1 - n2
  //
  //
  auto faceNormal = [xpos,ypos,zpos](
          node_idx n0, node_idx n1, node_idx n2) -> cv::Point3_<FP> {

      FP ux = xpos[ n2 ] - xpos[ n1 ];
      FP uy = ypos[ n2 ] - ypos[ n1 ];
      FP uz = zpos[ n2 ] - zpos[ n1 ];

      FP vx = xpos[ n0 ] - xpos[ n1 ];
      FP vy = ypos[ n0 ] - ypos[ n1 ];
      FP vz = zpos[ n0 ] - zpos[ n1 ];

      FP nx = uy * vz - vy * uz;
      FP ny = vx * uz - ux * vz;
      FP nz = ux * vy - vx * uy;

      cv::Point3_<FP> modelNorm = cv::Point3_<FP>(nx,ny,nz);
      FP nmag = cv::norm(modelNorm);
      cv::Point3_<FP> res = ( nmag == 0.f ) ? modelNorm : modelNorm/nmag;
      return res;

  }; // faceNormal()

  auto calcNormal = [this,xpos,ypos,zpos,faceNormal](int _nidx_) -> cv::Point3_<FP> {
      cv::Point3_<FP> normal(0.0,0.0,0.0);
      // add all the available faces for this node
      for (const auto face_index: n2t_[_nidx_]) {
          if (!this->valid_tri_[face_index]) continue;
          const auto& tri = this->tris_[face_index];
          normal += faceNormal(tri[0], tri[1], tri[2]);
      }
      FP nmag = cv::norm(normal);
      cv::Point3_<FP> res = ( nmag == 0.f ) ? normal : normal/nmag;
      return res;
  }; // calcNormal

  int nnodes = size(); // model::size()

  normals_.resize( nnodes, cv::Point3_<FP>(0.0,0.0,0.0) );

  // loop over nodes
#pragma omp parallel for
  for( int nidx=0 ; nidx<nnodes ; ++nidx )
  {
      normals_[nidx] = calcNormal(nidx);
  } // loop over nodes

} // TriModel_<FP>::calcNormals


/*********************************************************************
* Node Class
*********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::Node::Node(const TriModel_* model, node_idx n) :
        model_(const_cast<TriModel_*>(model)), nidx_(n) {

    assert(model != nullptr);
    assert(n < model->number_of_nodes());
}

/*****************************************************************************/
template<typename FP>
unsigned int TriModel_<FP>::Node::number_of_adj_faces() const { 
    return model_->n2t_[nidx_].size();
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::Node::has_primary_component() const {
    if (number_of_adj_faces() == 0) {
        return false;
    }
    if (!model_->has_components()) {
        return false;
    }

    auto it = (model_->n2t_[nidx_]).begin();
    component comp1 = model_->get_component(*it);
    for (++it; it != (model_->n2t_[nidx_]).end(); ++it) {
        if (model_->get_component(*it) != comp1) {
            return false;
        }
    }
    return true;
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::component TriModel_<FP>::Node::get_primary_component() const {
    assert(has_primary_component());

    return model_->get_component( *((model_->n2t_[nidx_]).begin()) );
}

/*****************************************************************************/
template<typename FP>
std::set<typename TriModel_<FP>::component> TriModel_<FP>::Node::get_components() const {

    std::set<component> comps;

    for (auto it = (model_->n2t_[nidx_]).begin(); it != (model_->n2t_[nidx_]).end(); ++it) {
        comps.insert(model_->get_component(*it));
    }

    return comps;
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> TriModel_<FP>::Node::get_normal() const {
    
    cv::Point3_<FP> normal(0.0,0.0,0.0);
    unsigned int count = 0;
    for (auto it=adj_face_begin(); it != adj_face_end(); ++it) {
        normal += (*it).get_normal() * (*it).get_area();
        ++count;
    }
    /*
    if (cv::norm(normal) == 0.0) {
        std::cout << "Failed to get normal for node " << nidx_ << std::endl;
        std::cout << " found " << count << " adjacent faces" << std::endl;
    }
    */
    if (cv::norm(normal) == 0.0) {
        return normal;
    } else {
        return normal / cv::norm(normal);
    }
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::Node::is_contained(const BoundingBox_<cv::Point3_<FP>>& bb) const {
    return contains(bb, get_position());
}

/********************************************************************
* Edge Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::Edge::Edge(const TriModel_* model, node_idx n1, node_idx n2) : 
        model_(const_cast<TriModel_*>(model)), ns_({n1,n2}) {

    assert(model_ != nullptr);
    assert(n1 < model_->number_of_nodes());
    assert(n2 < model_->number_of_nodes());

    // ensure that this edge exists
    std::vector<face_idx> inter;
    std::set_intersection(model_->n2t_[n1].begin(), model_->n2t_[n1].end(), 
        model_->n2t_[n2].begin(), model_->n2t_[n2].end(), std::back_inserter(inter));
    assert(inter.size() > 0);
}

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::Edge::Edge(const TriModel_* model, simple_edge& e) : 
        model_(const_cast<TriModel_*>(model)), ns_(e) {

    assert(model_ != nullptr);
    assert(ns_[0] < model_->number_of_nodes());
    assert(ns_[1] < model_->number_of_nodes());

    // ensure that this edge exists
    std::vector<face_idx> inter;
    std::set_intersection(model_->n2t_[ns_[0]].begin(), model_->n2t_[ns_[0]].end(), 
        model_->n2t_[ns_[1]].begin(), model_->n2t_[ns_[1]].end(), std::back_inserter(inter));
    assert(inter.size() > 0);
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> TriModel_<FP>::Edge::get_direction() const {
    cv::Point3_<FP> dir = model_->get_position(ns_[1]) - model_->get_position(ns_[0]);
    if (dir.norm() != 0) {
        return dir / dir.norm();
    } else {
        return dir;
    }
}

/********************************************************************
* Face Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::Face::Face(const TriModel_* model, face_idx t) : 
        model_(const_cast<TriModel_*>(model)), tri_(t) {

    assert(model_ != nullptr);
    assert(model_->is_face(t));
}

/*****************************************************************************/
template<typename FP>
std::array<typename TriModel_<FP>::simple_edge,3> TriModel_<FP>::Face::simple_edges() const {
    std::array<simple_edge,3> edges;
    for (unsigned int i=0; i < 3; ++i) {
        edges[i] = {model_->tris_[tri_][i], model_->tris_[tri_][i+1]};
    }
    return edges;
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> TriModel_<FP>::Face::get_normal() const {
    simple_tri s_tri = nodes();
    upsp::Triangle<FP> tri(model_->get_position(s_tri[0]), model_->get_position(s_tri[1]),
                           model_->get_position(s_tri[2]));
    return upsp::normal(tri);
}

/*****************************************************************************/
template<typename FP>
FP TriModel_<FP>::Face::get_area() const {
    simple_tri s_tri = nodes();
    upsp::Triangle<FP> tri(model_->get_position(s_tri[0]), model_->get_position(s_tri[1]),
                           model_->get_position(s_tri[2]));
    return upsp::area(tri);
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::Face::is_contained(const BoundingBox_<cv::Point3_<FP>>& bb) const {
    return ::upsp::contains(bb, get_triangle());
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::Face::is_intersected(const BoundingBox_<cv::Point3_<FP>>& bb) const {
    return ::upsp::intersects(get_triangle(), bb);
}

/*****************************************************************************/
template<typename FP>
double TriModel_<FP>::Face::intersects(const Ray<FP>& r) const {
    simple_tri s_tri = nodes();
    upsp::Triangle<FP> tri(model_->get_position(s_tri[0]), model_->get_position(s_tri[1]),
                           model_->get_position(s_tri[2]));
    return ::upsp::intersects(tri, r);
}

/********************************************************************
* NodeIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::NodeIterator::NodeIterator(const TriModel_* model, bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)) {

    assert(model != nullptr);
    
    if (begin) {
        nidx_ = 0;
    } else {
        nidx_ = model->number_of_nodes();
    }
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Node TriModel_<FP>::NodeIterator::operator*() const {
    return model_->node(nidx_);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NodeIterator& TriModel_<FP>::NodeIterator::operator++() {
    if (nidx_ < model_->number_of_nodes()) {
        ++nidx_;
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::NodeIterator::operator==(const NodeIterator& n_it) const {
    if (model_ != n_it.model_) {
        return false;
    }

    return nidx_ == n_it.nidx_;
}

/********************************************************************
* FaceIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::FaceIterator::FaceIterator(const TriModel_* model, bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)) {

    assert(model != nullptr);
    
    if (begin) {
        idx_ = 0;
        if (!model_->is_face(idx_)) {
            while (idx_ < model_->tris_.size()) {
                ++idx_;
                if (model_->is_face(idx_)) {
                    break;
                }
            }
        }
    } else {
        idx_ = model_->tris_.size();
    }
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::FaceIterator::operator*() const {
    return model_->face(idx_);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::FaceIterator& TriModel_<FP>::FaceIterator::operator++() {
    while (idx_ < model_->tris_.size()) {
        ++idx_;
        if (model_->is_face(idx_)) {
            break;
        }
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::FaceIterator::operator==(const FaceIterator& tri_it) const {
    if (model_ != tri_it.model_) {
        return false;
    }

    return idx_ == tri_it.idx_;
}

/********************************************************************
* CompFaceIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::CompFaceIterator::CompFaceIterator(const TriModel_* model, 
        component comp, bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)), comp_(comp) {

    assert(model != nullptr);
    assert(model_->comps_.find(comp) != model_->comps_.end()); 

    if (begin) {
        it_ = model_->comps_[comp].begin();
    } else {
        it_ = model_->comps_[comp].end();
    }
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::CompFaceIterator::operator*() const {
    return model_->face(*it_);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompFaceIterator& TriModel_<FP>::CompFaceIterator::operator++() {
    ++it_;
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::CompFaceIterator::operator==(const CompFaceIterator& cf_it) const {
    if (model_ != cf_it.model_) {
        return false;
    }

    return it_ == cf_it.it_;
}

/********************************************************************
* AdjNodeIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::AdjNodeIterator::AdjNodeIterator(const TriModel_* model, node_idx n,
        bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)), n_(n) {

    assert(model_ != nullptr);
    assert(n < model_->number_of_nodes());

    if (!begin) {
        idx_ = -1;
        return;
    }
    
    // find and store all of the adjacent nodes
    std::set<node_idx> tmp_adj;
    for (auto it=model_->nadj_face_begin(n); it != model_->nadj_face_end(n); ++it) {
        CircularArray<node_idx,3> nodes = (*it).nodes();
        for (unsigned int i=0; i < 3; ++i) {
            if (nodes[i] != n) {
                tmp_adj.insert(nodes[i]);
            }
        }
    }
    adj_ = std::vector<node_idx>(tmp_adj.begin(), tmp_adj.end());

    idx_ = 0;
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Node TriModel_<FP>::AdjNodeIterator::operator*() const {
    return model_->node(adj_[idx_]);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjNodeIterator& TriModel_<FP>::AdjNodeIterator::operator++() {
    if ( (idx_ >= 0) && (idx_ < adj_.size()) ) {
        ++idx_;
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::AdjNodeIterator::operator==(const AdjNodeIterator& adj_it) const {
    if (model_ != adj_it.model_) {
        return false;
    }
    if (n_ != adj_it.n_) {
        return false;
    }
    
    // check for end iterator
    if ( (idx_ < 0) || (idx_ == adj_.size()) ) {
        if ( (adj_it.idx_ < 0) || (adj_it.idx_ == adj_it.adj_.size()) ) {
            return true;
        } else {
            return false;
        }
    } else {
        return idx_ == adj_it.idx_;
    }
}

/********************************************************************
* NAdjFaceIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::NAdjFaceIterator::NAdjFaceIterator(const TriModel_* model, node_idx n,
        bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)), n_(n) {

    assert(model_ != nullptr);
    assert(n < model_->number_of_nodes());

    // find the number of adjacent faces
    count_ = ((model_->n2t_)[n]).size();

    // handle begin and end cases
    if (begin) {
        idx_ = 0;
        if (count_ != 0) {
            it_ = (model_->n2t_)[n].begin();
        }
    } else {
        idx_ = count_;
        if (count_ != 0) {
            it_ = (model_->n2t_)[n].end();
        }
    }

}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::NAdjFaceIterator::operator*() const {
    return model_->face(*it_);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::NAdjFaceIterator& TriModel_<FP>::NAdjFaceIterator::operator++() {
    if (idx_ < count_) {
        ++idx_;
        ++it_;
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::NAdjFaceIterator::operator==(const NAdjFaceIterator& adj_it) const {
    if (model_ != adj_it.model_) {
        return false;
    }
    if (n_ != adj_it.n_) {
        return false;
    }
    return idx_ == adj_it.idx_;
}

/********************************************************************
* AdjFaceIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::AdjFaceIterator::AdjFaceIterator(const TriModel_* model, face_idx t,
        bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)), t_(t) {

    assert(model_ != nullptr);
    assert(model_->is_face(t));

    // find the adjacent faces using set intersection
    if (begin) {
        count_ = 0; 
        for (unsigned int i=0; i < 3; ++i) {
            for (auto it=model_->n2t_[model_->tris_[t][i]].begin(); 
                    it != model_->n2t_[model_->tris_[t][i]].end(); ++it) {
                // skip if it is the central face t
                if (*it == t) {
                    continue;
                }

                // check if node i and i+1 share another face
                if (model_->n2t_[model_->tris_[t][i+1]].count(*it) == 1) {
                    adj_[count_++] = *it;
                    break;
                }
            }
        }
    }

    // handle begin and end cases
    if (begin) {
        idx_ = 0;
    } else {
        idx_ = 3;  // there cannot be more than 3 adjacent faces
        count_ = 3;
    }

}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::AdjFaceIterator::operator*() const {
    return model_->face(adj_[idx_]);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::AdjFaceIterator& TriModel_<FP>::AdjFaceIterator::operator++() {
    ++idx_;
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::AdjFaceIterator::operator==(const AdjFaceIterator& adj_it) const {
    if (model_ != adj_it.model_) {
        return false;
    }
    if (t_ != adj_it.t_) {
        return false;
    }

    // handle end iterator
    if (idx_ >= count_) {
        if (adj_it.idx_ >= adj_it.count_) {
            return true;
        } else {
            return false;
        }
    } else {
        // handle non-end iterator
        return idx_ == adj_it.idx_;
    }   
}

/********************************************************************
* CompAdjFaceIterator Class
********************************************************************/

/*****************************************************************************/
template<typename FP>
TriModel_<FP>::CompAdjFaceIterator::CompAdjFaceIterator(const TriModel_* model, face_idx t,
        bool begin/*=false*/) :
        model_(const_cast<TriModel_*>(model)), t_(t) {

    assert(model_ != nullptr);
    assert(model_->is_face(t));

    // find the adjacent faces using set intersection
    if (begin) {
        count_ = 0; 
        for (unsigned int i=0; i < 3; ++i) {
            for (auto it=model_->n2t_[model_->tris_[t][i]].begin(); 
                    it != model_->n2t_[model_->tris_[t][i]].end(); ++it) {
                // skip if it is the central face t
                if (*it == t) {
                    continue;
                }

                // skip if it is in a different component
                if (model_->has_components()) {
                    if (model_->f2c_[t] != model_->f2c_[*it]) {
                        continue;
                    }
                }

                // check if node i and i+1 share another face
                if (model_->n2t_[model_->tris_[t][i+1]].count(*it) == 1) {
                    adj_[count_++] = *it;
                    break;
                }
            }
        }
    }

    // handle begin and end cases
    if (begin) {
        idx_ = 0;
    } else {
        idx_ = 3;  // there cannot be more than 3 adjacent faces
        count_ = 3;
    }

    //std::cout << "Found " << count_ << " adjacent faces" << std::endl;
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::Face TriModel_<FP>::CompAdjFaceIterator::operator*() const {
    return model_->face(adj_[idx_]);
}

/*****************************************************************************/
template<typename FP>
typename TriModel_<FP>::CompAdjFaceIterator& TriModel_<FP>::CompAdjFaceIterator::operator++() {
    ++idx_;
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool TriModel_<FP>::CompAdjFaceIterator::operator==(const CompAdjFaceIterator& adj_it) const {
    if (model_ != adj_it.model_) {
        return false;
    }
    if (t_ != adj_it.t_) {
        return false;
    }

    // handle end iterator
    if (idx_ >= count_) {
        if (adj_it.idx_ >= adj_it.count_) {
            return true;
        } else {
            return false;
        }
    } else {
        // handle non-end iterator
        return idx_ == adj_it.idx_;
    }   
}

} /* end namespace upsp */
