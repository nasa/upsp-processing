/** @file
 *  @brief  Plot3D Grid Model
 *  @date   Nov 15, 2018
 *  @author jmpowel2
 */
#include "logging.h"
#include "plot3d.h"
#include "utils/pspError.h"

namespace upsp {

/*********************************************************************  
 * P3DModel_ 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::grid_size P3DModel_<FP>::size(
        unsigned int zone) const { 
    if (zone >= zones_) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    return sz_[zone];
}

/*****************************************************************************/
template<typename FP>
unsigned int P3DModel_<FP>::zone_size(unsigned int zone) const {
    if (zone >= zones_) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    return sz_[zone].pts();
}

/*****************************************************************************/
template<typename FP>
unsigned int P3DModel_<FP>::zone_size(unsigned int zone, unsigned int idx) const {
    if (zone >= zones_) {
        throw(std::invalid_argument("Exceeded number of zones"));
    }
    if (idx > 2) {  
        throw(std::invalid_argument("Exceeded number of dimensions"));
    }
    grid_size gsz = sz_[zone];
    switch (idx) {
        case (0): return gsz.j; break;
        case (1): return gsz.k; break;
        case (2): return gsz.l; break;
    }
    return -1;
}

/*****************************************************************************/
template<typename FP>
unsigned int P3DModel_<FP>::zone_start_idx(unsigned int zone) const {
    if (zone >= zones_) {
        throw std::invalid_argument("Exceeded number of zones");
    }
    unsigned int idx=0;
    for (unsigned int i=0; i < zone; ++i) {
        idx += zone_size(i);
    }
    return idx;
}

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::P3DModel_(std::string filename, FP tol) : overlap_tol_(tol) {

    // Load in the p3d grid
    read_p3d_file(filename);

    // Require grid be a surface grid in jk
    for (int i=0; i < zones_; ++i) {
        assert( is_surface(sz_[i]) );
        assert(sz_[i].l == 1);
    }

    std::pair<int,int> res = identifyOverlap(overlap_tol_);
    int nonuniq = res.first;
    int uniq    = res.second;
    n_vert_ = x_.size() - nonuniq + uniq;
    n_face_ = 0;
    for (int i=0; i < sz_.size(); ++i) {
        n_face_ += (sz_[i].j - 1) * (sz_[i].k - 1);
    }
    calcNormals();
}

/*****************************************************************************/
template<typename FP>
template<typename FPin>
P3DModel_<FP>::P3DModel_(const upsp::StructuredGrid<FPin>& grid, 
        FP tol/*=0.0*/) {
    load_grid(grid, tol);
}

/*****************************************************************************/
template<typename FP>
template<typename FPin>
void P3DModel_<FP>::load_grid(const upsp::StructuredGrid<FPin>& grid, 
        FP tol/*=0.0*/) {

    overlap_tol_ = tol;

    // Copy grid
    zones_ = grid.num_zones();

    if constexpr (std::is_same<FP,FPin>::value) {
        x_ = grid.x;
        y_ = grid.y;
        z_ = grid.z;
    } else {
        x_.assign(grid.x.begin(), grid.x.end());
        y_.assign(grid.y.begin(), grid.y.end());
        z_.assign(grid.z.begin(), grid.z.end());
    }

    sz_.resize(zones_);
    for (unsigned int i=0; i < grid.grid_size.size(); ++i) {
        sz_[i] = GridIndex(i, grid.grid_size[i][0], grid.grid_size[i][1], 
                grid.grid_size[i][2]);
    }

    // Require grid be a surface grid in jk
    for (int i=0; i < zones_; ++i) {
        assert( is_surface(sz_[i]) );
        assert(sz_[i].l == 1);
    }
    int overlap = 0;
    std::pair<int,int> res = identifyOverlap(overlap_tol_);
    int nonuniq = res.first;
    int uniq    = res.second;
    n_vert_ = x_.size() - nonuniq + uniq;
    n_face_ = 0;
    for (int i=0; i < sz_.size(); ++i) {
        n_face_ += (sz_[i].j - 1) * (sz_[i].k - 1);
    }
    calcNormals();
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::adjust_solution(std::vector<FP>& sol) const {

    node_idx curr, alt;
    for (auto it=overlap_pts_.begin(); it != overlap_pts_.end(); ++it) {
        curr = (*it).first;
        for (auto vit=(*it).second.begin(); vit != (*it).second.end(); ++vit) {
            alt = *vit;
            if (curr < alt) {
                sol[alt] = sol[curr];
            }
        }
    }

}

/*****************************************************************************/
template<typename FP>   
void P3DModel_<FP>::set_normals(node_idx nidx, cv::Point3_<FP> dir) {
    normals_[nidx] = dir;
}

/*****************************************************************************/
template<typename FP>
UnstructuredGrid<FP> P3DModel_<FP>::triangulate() const {

    UnstructuredGrid<FP> grid;
    
    // Maintain node indexing
    grid.x = x_;
    grid.y = y_;
    grid.z = z_;

    // Create triangles and carry over zone ids as components
    unsigned int num_tris = number_of_faces() * 2;

    grid.tris.resize(num_tris);
    grid.comps.resize(num_tris);

    unsigned int idx = 0;
    for (auto it = face_begin(); it != face_end(); ++it) {
        Face f = *it;
        
        std::vector<node_idx> nodes = f.get_nodes();
        grid.tris[idx] = CircularArray<node_idx,3>(nodes[0],nodes[1],nodes[2]);
        grid.comps[idx] = f.get_zone();

        grid.tris[idx+1] = CircularArray<node_idx,3>(nodes[0],nodes[2],nodes[3]);
        grid.comps[idx] = f.get_zone();

        idx += 2;
    }

    return grid;
}

/*****************************************************************************/
template<typename FP>
template<typename FPout>
P3DModel_<FPout> P3DModel_<FP>::cast() const {

    P3DModel_<FPout> p3d;

    p3d.zones_ = zones_;
    p3d.sz_ = sz_;
    p3d.overlap_tol_ = overlap_tol_;

    p3d.x_.assign(x_.begin(), x_.end());
    p3d.y_.assign(y_.begin(), y_.end());
    p3d.z_.assign(z_.begin(), z_.end());

    p3d.overlap_pts_ = overlap_pts_;

    p3d.n_vert_ = n_vert_;
    p3d.n_face_ = n_face_;

    p3d.normals_.clear();
    p3d.normals_.insert(p3d.normals_.begin(),
                        normals_.begin(), normals_.end());

    for (unsigned int i=0; i < x_.size(); ++i) {
        if (!is_datanode(i)) {
            p3d.set_node_nondata(i);
        }
    }
    
    return p3d;
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::extract_tris(
        std::vector<float>& tris,
        std::vector<int>& triNodes) const {
    typedef cv::Point3_<float> v3f;
    const size_t XYZ = 3;
    const size_t TRI = 3;

    int nnodes = size();
    int nzones = num_zones();
    const std::vector<FP>& xpos = get_x();
    const std::vector<FP>& ypos = get_y();
    const std::vector<FP>& zpos = get_z();

    std::vector<float> nodepos(nnodes*3);
    for (int i = 0; i < nnodes; ++i) {
        nodepos[i*3+0] = xpos[i];
        nodepos[i*3+1] = ypos[i];
        nodepos[i*3+2] = zpos[i];
    }

    // extract tris
    std::vector<int> srcoff(nzones, 0);
    std::vector<int> dstoff(nzones, 0);
    int soff = 0 , doff = 0;
    for(int zidx = 0; zidx < nzones; ++zidx) {
        upsp::GridIndex s = size( zidx );
        int nvrts  = s.j * s.k;
        int nquads = (s.j-1) * (s.k-1);

        srcoff[zidx] = soff;
        soff += nvrts; // index into x[],y[],z[]

        dstoff[zidx] = doff;
        doff += 2 * nquads * XYZ * TRI; // index into tris[]
    }

    size_t triNodeCnt = doff/3;
    tris.resize( doff, 0 );
    triNodes.resize( triNodeCnt, -1 );

    int tidx=0;
    for( int zidx=0 ; zidx<nzones ; ++zidx ){
        upsp::GridIndex s = size( zidx );
        int nvrts  = s.j * s.k;
        int nquads = (s.j-1) * (s.k-1);

        float* dstbase = tris.data() + dstoff[zidx];
        int srcbase = srcoff[zidx];

        for( int qidx=0 ; qidx<nquads ; ++qidx ){

            int klo = qidx / (s.j-1); // j = idx/I
            int khi = klo + 1;
            int jlo = qidx % (s.j-1); // i = idx%I
            int jhi = jlo + 1;

            // global indices of the face verts
            int i0 = srcbase + klo * s.j + jlo; // dj=0,dk=0
            int i1 = srcbase + klo * s.j + jhi; // dj=1,dk=0
            int i2 = srcbase + khi * s.j + jhi; // dj=1,dk=1
            int i3 = srcbase + khi * s.j + jlo; // dj=0,dk=1

            // lo tri
            std::copy(&nodepos[i0*3], &nodepos[i0*3]+XYZ, dstbase + 0*XYZ);
            std::copy(&nodepos[i1*3], &nodepos[i1*3]+XYZ, dstbase + 1*XYZ);
            std::copy(&nodepos[i2*3], &nodepos[i2*3]+XYZ, dstbase + 2*XYZ);

            triNodes[ tidx*3+0 ] = i0;
            triNodes[ tidx*3+1 ] = i1;
            triNodes[ tidx*3+2 ] = i2;
            ++tidx;

            // hi tri
            std::copy(&nodepos[i2*3], &nodepos[i2*3]+XYZ, dstbase + 3*XYZ);
            std::copy(&nodepos[i3*3], &nodepos[i3*3]+XYZ, dstbase + 4*XYZ);
            std::copy(&nodepos[i0*3], &nodepos[i0*3]+XYZ, dstbase + 5*XYZ);

            triNodes[ tidx*3+0 ] = i2;
            triNodes[ tidx*3+1 ] = i3;
            triNodes[ tidx*3+2 ] = i0;
            ++tidx;

            dstbase += 6*XYZ;
        }
    } // for zidx
}


/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::write_overlapped_pts(std::string filename) const {

    std::ofstream fs_out(filename);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write overlap points"));
    }

    fs_out.precision(8);    

    fs_out << "TITLE=\"Overlapped Points\"" << std::endl;
    fs_out << "VARIABLES=\"X\" \"Y\" \"Z\"" << std::endl;

    node_idx nidx;
    cv::Point3_<FP> pt;
    for (auto it = overlap_pts_.begin(); it != overlap_pts_.end(); ++it) {
        nidx = (*it).first;
        //for (int i=0; i < (*it).second.size(); ++i) {
            if ( (*it).second[0] < nidx ) {
                fs_out << x_[nidx] << " " << y_[nidx] << " ";
                fs_out << z_[nidx] << std::endl;
            }
        //}
    } 

    fs_out.close();

}

/*****************************************************************************/
template<typename FP>
template<typename T>
void P3DModel_<FP>::mark_overlapped_pts(std::vector<T>& marks) const {

    marks.assign(size(), 0);

    for (auto it = overlap_pts_.begin(); it != overlap_pts_.end(); ++it) {
        marks[(*it).first] = 1;
    }
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::write_grid(std::string filename) const {

    // Generate RefStructuredGrid
    RefStructuredGrid<FP> ref_grid(x_,y_,z_);

    ref_grid.grid_size.resize(zones_);
    for (unsigned int i=0; i < zones_; ++i) {
        ref_grid.grid_size[i].resize(3);
        ref_grid.grid_size[i][0] = sz_[i].j;
        ref_grid.grid_size[i][1] = sz_[i].k;
        ref_grid.grid_size[i][2] = sz_[i].l;
    }

    // Write out the grid
    write_plot3d_grid_file(filename, ref_grid);
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::write_tri_grid(std::string filename) const {

    std::ofstream fs_out(filename, std::ios::out | std::ios::binary);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write tri grid"));
    }

    int32_t sz = sizeof(int32_t)*2;
    int32_t n_tri = n_face_ * 2;

    fs_out.write((char*) &sz, sizeof(int32_t));
    fs_out.write((char*) &n_vert_, sizeof(int32_t));
    fs_out.write((char*) &n_tri, sizeof(int32_t));
    fs_out.write((char*) &sz, sizeof(int32_t));

    // Add the vertices to the file and log the conversion between node index
    // and the index of the printed vertex
    std::vector<int32_t> nidx2_idx(x_.size(),0);
    cv::Point3_<FP> pt;
    float x,y,z;
    int32_t count=1;
    sz = sizeof(float) * 3 * n_vert_;
    fs_out.write((char*) &sz, sizeof(int32_t));
    for (auto it=node_begin(); it != node_end(); ++it) {
        pt = (*it).get_position();
        x = (float) pt.x;
        y = (float) pt.y;
        z = (float) pt.z;
        fs_out.write((char*) &x, sizeof(float));
        fs_out.write((char*) &y, sizeof(float));
        fs_out.write((char*) &z, sizeof(float));
        nidx2_idx[(*it).get_nidx()] = count;
        ++count;
    }
    --count;
    fs_out.write((char*) &sz, sizeof(int32_t));
    std::cout << "Expected " << n_vert_ << " vertices" << std::endl;
    std::cout << "Wrote    " << count << " vertices" << std::endl;

    // Add the faces to the file and log the component (zone) id of each face
    // Triangles are nodes (0,1,2) and (2,3,0)
    std::vector<int32_t> face2_comp(2 * n_face_,0);
    std::vector<node_idx> nf;
    int32_t n1,n2,n3;
    count = 0;
    int32_t zone;
    sz = sizeof(int32_t) * 3 * n_face_ * 2;
    fs_out.write((char*) &sz, sizeof(int32_t));
    for (auto it = face_begin(); it != face_end(); ++it) {
        nf = (*it).get_nodes();
        zone = (int32_t) (*it).get_zone();
    
        n1 = nidx2_idx[nf[0]];
        n2 = nidx2_idx[nf[1]];
        n3 = nidx2_idx[nf[2]];
        fs_out.write((char*) &n1, sizeof(int32_t));
        fs_out.write((char*) &n2, sizeof(int32_t));
        fs_out.write((char*) &n3, sizeof(int32_t));

        // Check that all nidx were covered in node iteration
        if ((n1 == 0) || (n2 == 0) || (n3 == 0) ) {
            std::cerr << "invalid index found in face iteration" << std::endl;
            abort();
        }
    
        n1 = nidx2_idx[nf[2]];
        n2 = nidx2_idx[nf[3]];
        n3 = nidx2_idx[nf[0]];
        fs_out.write((char*) &n1, sizeof(int32_t));
        fs_out.write((char*) &n2, sizeof(int32_t));
        fs_out.write((char*) &n3, sizeof(int32_t));

        if (n2 == 0) {
            std::cerr << "invalid index found in face iteration" << std::endl;
            abort();
        }

        face2_comp[count] = zone + 1;
        ++count;
        face2_comp[count] = zone + 1;
        ++count;
    }
    fs_out.write((char*) &sz, sizeof(int32_t));
    std::cout << "Expected " << (2*n_face_) << " faces" << std::endl;
    std::cout << "Wrote    " << count << " faces" << std::endl;

    // Write out the component ids
    sz = sizeof(int32_t) * n_face_ * 2;
    fs_out.write((char*) &sz, sizeof(int32_t));
    fs_out.write((char*) &face2_comp[0], face2_comp.size()*sizeof(int32_t)); 
    fs_out.write((char*) &sz, sizeof(int32_t));

    fs_out.close();
}

/*****************************************************************************/
template<typename FP>
template<typename T>
void P3DModel_<FP>::write_triq_sol(std::string filename, const std::vector<T>& sol) const {

    assert(sol.size() == size());
    
    std::ofstream fs_out(filename, std::ios::out | std::ios::binary);
    if (!fs_out) {
        throw(std::invalid_argument("Cannot open file to write tri grid"));
    }

    int32_t sz = sizeof(int32_t)*3;
    int32_t n_tri = n_face_ * 2;
    int32_t n_scalar = 1;

    fs_out.write((char*) &sz, sizeof(int32_t));
    fs_out.write((char*) &n_vert_, sizeof(int32_t));
    fs_out.write((char*) &n_tri, sizeof(int32_t));
    fs_out.write((char*) &n_scalar, sizeof(int32_t));
    fs_out.write((char*) &sz, sizeof(int32_t));

    // Add the vertices to the file and log the conversion between node index
    // and the index of the printed vertex
    std::vector<int32_t> nidx2_idx(x_.size(),0);
    cv::Point3_<FP> pt;
    float x,y,z;
    int32_t count=1;
    sz = sizeof(float) * 3 * n_vert_;
    fs_out.write((char*) &sz, sizeof(int32_t));
    for (auto it=node_begin(); it != node_end(); ++it) {
        pt = (*it).get_position();
        x = (float) pt.x;
        y = (float) pt.y;
        z = (float) pt.z;
        fs_out.write((char*) &x, sizeof(float));
        fs_out.write((char*) &y, sizeof(float));
        fs_out.write((char*) &z, sizeof(float));
        nidx2_idx[(*it).get_nidx()] = count;
        ++count;
    }
    --count;
    fs_out.write((char*) &sz, sizeof(int32_t));
    std::cout << "Expected " << n_vert_ << " vertices" << std::endl;
    std::cout << "Wrote    " << count << " vertices" << std::endl;

    // Add the faces to the file and log the component (zone) id of each face
    // Triangles are nodes (0,1,2) and (2,3,0)
    std::vector<int32_t> face2_comp(2 * n_face_,0);
    std::vector<node_idx> nf;
    int32_t n1,n2,n3;
    count = 0;
    int32_t zone;
    sz = sizeof(int32_t) * 3 * n_face_ * 2;
    fs_out.write((char*) &sz, sizeof(int32_t));
    for (auto it = face_begin(); it != face_end(); ++it) {
        nf = (*it).get_nodes();
        zone = (int32_t) (*it).get_zone();
    
        n1 = nidx2_idx[nf[0]];
        n2 = nidx2_idx[nf[1]];
        n3 = nidx2_idx[nf[2]];
        fs_out.write((char*) &n1, sizeof(int32_t));
        fs_out.write((char*) &n2, sizeof(int32_t));
        fs_out.write((char*) &n3, sizeof(int32_t));

        // Check that all nidx were covered in node iteration
        if ((n1 == 0) || (n2 == 0) || (n3 == 0) ) {
            std::cerr << "invalid index found in face iteration" << std::endl;
            abort();
        }
    
        n1 = nidx2_idx[nf[2]];
        n2 = nidx2_idx[nf[3]];
        n3 = nidx2_idx[nf[0]];
        fs_out.write((char*) &n1, sizeof(int32_t));
        fs_out.write((char*) &n2, sizeof(int32_t));
        fs_out.write((char*) &n3, sizeof(int32_t));

        if (n2 == 0) {
            std::cerr << "invalid index found in face iteration" << std::endl;
            abort();
        }

        face2_comp[count] = zone + 1;
        ++count;
        face2_comp[count] = zone + 1;
        ++count;
    }
    fs_out.write((char*) &sz, sizeof(int32_t));
    std::cout << "Expected " << (2*n_face_) << " faces" << std::endl;
    std::cout << "Wrote    " << count << " faces" << std::endl;

    // Write out the component ids
    sz = sizeof(int32_t) * n_face_ * 2;
    fs_out.write((char*) &sz, sizeof(int32_t));
    fs_out.write((char*) &face2_comp[0], face2_comp.size()*sizeof(int32_t)); 
    fs_out.write((char*) &sz, sizeof(int32_t));

    // Write out the solution at each node
    float f_sol;
    sz = sizeof(float) * n_vert_;
    fs_out.write((char*) &sz, sizeof(int32_t));
    for (auto it = node_begin(); it != node_end(); ++it) {
        f_sol = (float) sol[(*it).get_nidx()];
        fs_out.write((char*) &f_sol, sizeof(float)); 
    }
    fs_out.write((char*) &sz, sizeof(int32_t));

    fs_out.close();

}

/* INCOMPLETE
 ** Write out a .plt single solution file for tecplot
 * 
 * @param[in] filename  file that will be written to
 * @param[in] sol       a vector containing the solution at each node point
 *
 * @pre @a sol is length this->size()
 * @post    if nidx1 < nidx2 and overlap_pts_[nidx2] includes nidx1, then nidx2 will
 *          use sol[nidx1]
void P3DModel_<FP>::write_tec_sol(std::string filename, const std::vector<double>& sol,
        double time) const {

    assert(sol.size() == size());

    // Prepare c-style filename
    char c_file[filename.length()+1];
    std::strcpy(c_file, filename.c_str());

    // Define TECIO parameters
    INTEGER4 I; // check return values

    INTEGER4 Debug      = 1;
    INTEGER4 VIsDouble  = 1;
    INTEGER4 FileType   = 1; // 0 == full, 1 == grid, 2 == solution
    INTEGER4 FileFormat = 0; // 0 == PLT, 1 = SZPLT

    // Title, Variables, filename, scratch directory, ...
    I = TECINI142((char*) "Plot3D style grid", (char*) "X Y Z",
            c_file, (char*) ".", &FileFormat, &FileType, &Debug, &VIsDouble);

    // Define Zone parameters
    INTEGER4 ZoneType           = 0; // ordered
    INTEGER4 IMax               = 0;
    INTEGER4 JMax               = 0; 
    INTEGER4 KMax               = 0;
    INTEGER4 ICellMax           = 0; // reserved for future use, set 0
    INTEGER4 JCellMax           = 0; // reserved for future use, set 0
    INTEGER4 KCellMax           = 0; // reserved for future use, set 0
    double SolutionTime         = time;
    INTEGER4 StrandID           = 0; // static zone
    INTEGER4 ParentZone         = 0;
    INTEGER4 IsBlock            = 1;
    INTEGER4 NumFaceConnections = 0;
    INTEGER4 FaceNeighborMode   = 0;
    INTEGER4 NumFaceNodes       = 0; // not needed for ordered
    INTEGER4 NumConnBoundFaces  = 0; // not needed for ordered
    INTEGER4 NumBoundConn       = 0; // not needed for ordered
    INTEGER4 PassiveVarList     = 0; // pass null for all variables active
    INTEGER4 ValueLocation      = 0; // pass null for all variables node-centered
    INTEGER4 SharVarFromZone    = 0; // pass null to indicate no variable sharing
    INTEGER4 SharConn           = 0; // no connectivity sharing

    INTEGER4 III;
    int count = 0;
    std::string s_title;
    for (int i=0; i < zones_; ++i) {

        // Define specific zone parameters
        IMax = sz_[i].j;
        JMax = sz_[i].k;
        KMax = sz_[i].l;
        III = sz_[i].pts();

        // Write header for zone
        s_title = "Zone " +  std::to_string(i+1);
        char title[s_title.length()+1]; 
        std::strcpy(title, s_title.c_str());
        I = TECZNE142(title, 
                &ZoneType, &IMax, &JMax, &KMax, &ICellMax, &JCellMax,
                &KCellMax, &SolutionTime, &StrandID, &ParentZone, &IsBlock,
                &NumFaceConnections, &FaceNeighborMode, &NumFaceNodes,
                &NumConnBoundFaces, &NumBoundConn, nullptr, nullptr, 
                nullptr, &SharConn);

        // Write grid points for zone
        I = TECDAT142(&III, &x_[count], &VIsDouble);

        count += sz_[i].pts();
    }

    I = TECEND142();

}
*/

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> P3DModel_<FP>::get_position(const GridIndex gidx) const {
    return get_position(gidx2_nidx(gidx));
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> P3DModel_<FP>::get_position(const node_idx nidx) const {
    return cv::Point3_<FP>(x_[nidx],y_[nidx],z_[nidx]);
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::is_overlapping(node_idx nidx) const {
    if (overlap_pts_.find(nidx) != overlap_pts_.end()) {
        return true;
    }
    return false;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::is_superceded(node_idx nidx) const {
    return get_low_nidx(nidx) != nidx;
}

/*****************************************************************************/
template<typename FP>
unsigned int P3DModel_<FP>::number_of_faces() const {
    unsigned int faces = 0;
    for (unsigned int i=0; i < num_zones(); ++i) {
        faces += (sz_[i].j - 1) * (sz_[i].k - 1);
    }
    return faces;
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::NodeIterator P3DModel_<FP>::node_begin() const {
    return NodeIterator(this,GridIndex(0,0,0,0));
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::NodeIterator P3DModel_<FP>::node_end() const {
    return NodeIterator(this,GridIndex(zones_,0,0,0));
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::FaceIterator P3DModel_<FP>::face_begin() const {
    return FaceIterator(this,0);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::FaceIterator P3DModel_<FP>::face_end() const {
    return FaceIterator(this,number_of_faces());
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::EdgeIterator P3DModel_<FP>::edge_begin(int zone) const {
    return EdgeIterator(this,zone);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::EdgeIterator P3DModel_<FP>::edge_end(int zone) const {
    return EdgeIterator(this,zone,true);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjNodeIterator P3DModel_<FP>::adj_node_begin(GridIndex gidx) const {
    return AdjNodeIterator(this,gidx);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjNodeIterator P3DModel_<FP>::adj_node_end(GridIndex gidx) const {
    return AdjNodeIterator(this,gidx,true);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjFaceIterator P3DModel_<FP>::adj_face_begin(GridIndex gidx) const {
    return AdjFaceIterator(this,gidx);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjFaceIterator P3DModel_<FP>::adj_face_end(GridIndex gidx) const {
    return AdjFaceIterator(this,gidx,true);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjFaceIterator P3DModel_<FP>::adj_face_begin(node_idx nidx) const {
    return AdjFaceIterator(this,nidx2_gidx(nidx));
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjFaceIterator P3DModel_<FP>::adj_face_end(node_idx nidx) const {
    return AdjFaceIterator(this,nidx2_gidx(nidx),true);
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::read_p3d_file(std::string filename) {

    // Generate RefStructuredGrid
    RefStructuredGrid<FP> ref_grid(x_,y_,z_);

    // Read in the data
    read_plot3d_grid_file(filename, ref_grid);

    // Copy over the size information
    zones_ = ref_grid.num_zones();
    sz_.resize(zones_);
    for (unsigned int i=0; i < zones_; ++i) {
        sz_[i].j = ref_grid.grid_size[i][0];
        sz_[i].k = ref_grid.grid_size[i][1];
        sz_[i].l = ref_grid.grid_size[i][2];
    }
}

/*****************************************************************************/
template<typename FP>
int P3DModel_<FP>::get_zone(node_idx nidx) const {
    int total_size = 0;
    for (int i=0; i < zones_; ++i) {
        total_size += sz_[i].pts();
        if (nidx < total_size) {
            return i;
        }
    }
    return -1;
}

/*****************************************************************************/
template<typename FP>
GridIndex P3DModel_<FP>::nidx2_gidx(node_idx nidx) const {

    if (nidx >= x_.size()) {
        return GridIndex();
    }

    // j is the fastest direction, then k, then zone
    int zone = -1, j_ind = -1, k_ind = -1;
    int total_size = 0;
    for (int i=0; i < zones_; ++i) {
        total_size += sz_[i].pts();
        if (nidx < total_size) {
            nidx -= (total_size - sz_[i].pts());
            zone = i;
            k_ind = nidx / sz_[i].j;
            j_ind = nidx % sz_[i].j;
            break;
        }
    }

    return GridIndex(zone, j_ind, k_ind);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::node_idx P3DModel_<FP>::gidx2_nidx(const GridIndex& gidx) const {

    assert(gidx.zone >= 0);
    assert(gidx.zone < zones_);

    int nidx = 0;
    for (int i=0; i < gidx.zone; ++i) {
        nidx += sz_[i].pts();
    }
    int idx_zone = gidx.k * sz_[gidx.zone].j + gidx.j;
    
    assert(idx_zone >= 0);
    assert(idx_zone < sz_[gidx.zone].pts());

    return nidx + idx_zone;
}

/*****************************************************************************/
template<typename FP>
GridIndex P3DModel_<FP>::fidx2_min_gidx(face_idx fidx) const {

    assert(fidx < number_of_faces());

    // Grab the nodes that make up this face
    int zone = 0;
    int count = 0;
    for (unsigned int i=0; i < zones_; ++i) {
        int n_faces = (sz_[i].j - 1) * (sz_[i].k - 1);
        if (fidx < (count + n_faces)) {
            zone = i;
            break;
        }
        count += n_faces;
    }

    int min_j = (fidx - count) % (sz_[zone].j-1);
    int min_k = (fidx - count) / (sz_[zone].j-1);
    
    return GridIndex(zone, min_j, min_k); 
}

/*****************************************************************************/

// SANDSTROM: 2019-Jul-10 ...
// store overlaps as they did before, lose the multimap
template<typename FP>
std::pair<int,int> P3DModel_<FP>::identifyOverlap(float tol)
{
    // TODO the KD tree nearest-neighbor implementation doesn't handle a
    // tolerance of exactly 0. So, inflate it to a small non-zero epsilon
    const float min_tol = 1e-12;
    tol = tol > min_tol ? tol : min_tol;

    // so we can store/retrieve node_idx to/from the kdtree which expects void* pointers
    union UserData {
	void* ptr;
	uint64_t val;
    };

    // SANDSTROM: 2019-Jul-10:
    // convert our multimap to their map: overlap_pts_

    auto build = [] (std::multimap<int,int>& src, std::map<node_idx, std::vector<node_idx> > & dst) {

	// set of all nodes with overlaps
	std::set<int> keys;
	for( auto ii : src ){
	    keys.insert(ii.first);
	}

	// build a vector for each node with overlaps
	for( auto key : keys ){
	    std::vector<node_idx>& vals = dst[key]; // calls vector::vector
	    auto ilo = src.lower_bound (key);
	    auto ihi = src.upper_bound (key);
	    std::set<int> seen;
	    for( auto iter=ilo ; iter != ihi ; ++iter ){
		node_idx overlapKey = (*iter).second;
		// belt and suspenders:
		// NOTE: a node CANNOT 'overlap' itself
		auto found = seen.find( overlapKey );
		if( found != seen.end() ) continue;
		seen.insert( overlapKey );
		if( overlapKey != key )
		    vals.push_back( overlapKey );
	    }

	    // these need to be sorted to maintain behaviours of iters, etc.
	    std::sort( vals.begin(), vals.end() );

	} // loop over nodes with overlaps

    }; // build()

    UserData udata;

    std::multimap<int,int> overlap;
    const int nzones = num_zones();

    // build KD of all boundary nodes across all zones
    this->overlapRoot = kd_create(3);

    // build KD of all nodes
    this->root = kd_create(3);

    int base = 0;
    for(uint32_t zidx=0 ; zidx<nzones ; ++zidx )
    {
	const auto sz = size(zidx);
	int znodes = sz.j * sz.k;

	for( int idx=0 ; idx<znodes ; ++idx ){
	    int r = idx / sz.j;
	    int c = idx % sz.j;

	    int nidx = base + idx;
	    cv::Point3_<FP> pos = get_position( nidx ); // "run" coords
	    float x = pos.x;
	    float y = pos.y;
	    float z = pos.z;

	    // send gidx as 'userdata', no need to allocate
	    udata.val = static_cast<uint64_t>(nidx);

	    // add all nodes to main kdtree
	    kd_insert3(this->root, x, y, z, udata.ptr); // NOTE: root

	    // boundary only add edge nodes to be tested for 'overlap'
	    if( r==0 or r==(sz.k-1) or c==0 or c==(sz.j-1) )
		kd_insert3(this->overlapRoot, x, y, z, udata.ptr); // NOTE: overlapRoot

	} // loop over zone nodes

	// advance past zone
	base += znodes;

    } // loop over zones

    // 1) loop over all boundary nodes across all zones
    // 2) find all the nodes within user-specified a tolerance
    // 3) make the lowest one the canonical one

    base=0; // reset this

    std::set<int> oset;
    std::set<int> seen,uniq;

    for( uint32_t zidx=0 ; zidx<nzones ; ++zidx )
    {
	const auto sz = size(zidx);
	int znodes = sz.j * sz.k;
	GridIndex dims = size( zidx );

	for( int idx=0 ; idx<znodes ; ++idx )
	{
	    int r = idx / sz.j;
	    int c = idx % sz.j;

	    // only test edge nodes, cheesy but simple
	    if( r==0 or r==(sz.k-1) or c==0 or c==(sz.j-1) )
	    {
		int nidx = base + idx;
		GridIndex gidx = nidx2_gidx(nidx);

		cv::Point3_<float> pos = get_position( nidx ); // "run" coords
		float x = pos.x;
		float y = pos.y;
		float z = pos.z;

		// find all zone boundary nodes within ['<='] the requested tolerance
		kdres *res = kd_nearest_range3(this->overlapRoot,x,y,z,tol);

		// process the query results
		std::set<int> others;
		do {
		    udata.ptr = kd_res_item_data(res);
		    int other = static_cast<int>(udata.val);

		    // NOTE: a node cannot 'overlap' itself
		    if( other==nidx ) continue;

		    // NOTE: overlaps must come from other grids
		    // UNLESS the grid WRAPS, in which case
		    //
		    // either: Js are equal, Ks 0 and K-1
		    //     or: Js are 0 and J-1 and Ks are equal

		    GridIndex other_gidx = nidx2_gidx(other);

		    bool samezone=false;
		    bool wrapped=false;
		    if( other_gidx.zone==gidx.zone )
		    {
			samezone=true;
			// wraps if Js are equal and Ks are 0 and K-1
			if( gidx.j == other_gidx.j )
			{
			    if( (gidx.k==0          and other_gidx.k==(dims.k-1) ) or
				(gidx.k==(dims.k-1) and other_gidx.k==0       )  )
				wrapped=true;
			}
			// wraps if Ks are equal and Js are 0 and J-1
			else if( gidx.k==other_gidx.k )
			{
			    if( (gidx.j==0          and other_gidx.j==(dims.j-1) ) or
				(gidx.j==(dims.j-1) and other_gidx.j==0        ) )
				wrapped=true;
			}
		    }

		    if( samezone and not wrapped ) continue;

		    others.insert( static_cast<int>(udata.val) );
		} while( kd_res_next(res) );

		// free the query results
		kd_res_free(res);

		if ( others.empty() ) continue;

		// ok, some overlap

		auto nseen = seen.find(nidx);
		auto nuniq = uniq.find(nidx);
		if( nseen==seen.end() and nuniq==uniq.end() )
		{
		    uniq.insert(nidx);
		    seen.insert(nidx);
		}

		for( auto other_nidx : others ){

		    seen.insert( other_nidx );

		    int lo_nidx = std::min(nidx, other_nidx);
		    int hi_nidx = std::max(nidx, other_nidx);
		    auto newentry = std::make_pair( lo_nidx, hi_nidx );
		    auto ilo = overlap.lower_bound (lo_nidx);
		    auto ihi = overlap.upper_bound (lo_nidx);
		    bool found=false;
		    for( auto iter=ilo ; iter != ihi and not found ; ++iter ){
			auto oldentry = *iter;
			found = (oldentry.first==newentry.first) and (oldentry.second==newentry.second);
		    }

		    if( not found ){
			oset.insert( lo_nidx );
			oset.insert( hi_nidx );

			overlap.insert( newentry );

			// ok, add reverse entry so we can search on either node
			auto reventry = std::make_pair( hi_nidx, lo_nidx );
			overlap.insert( reventry );
		    } // not found

		} // loop over others

	    } // only edges

	} // loop over zone nodes

	// advance past zone
	base += znodes;

    } // loop over zones

    // store into their map, preserving behavior of their iterators, etc
    build( overlap, overlap_pts_ );

    LOG_DEBUG(
        "P3DModel_::identifyOverlap(tol=%5.3f) %zu non-unique points",
        tol, oset.size()
    );
    LOG_DEBUG(
        "P3DModel_::identifyOverlap(tol=%5.3f) %zu unique, overlapping points",
        tol, uniq.size()
    );
    return std::make_pair( static_cast<int>(oset.size()),static_cast<int>(uniq.size()) );

} // identifyOverlap

// SANDSTROM: ... 2019-Jul-10

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::is_valid_gidx(const GridIndex& gidx) const {
    return ( (gidx.zone >= 0) && (gidx.zone < zones_) && 
            (gidx.j >= 0) && (gidx.j < sz_[gidx.zone].j) &&
            (gidx.k >= 0) && (gidx.k < sz_[gidx.zone].k) &&
            (gidx.l >= 0) && (gidx.l < sz_[gidx.zone].l) );
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::is_surface(const grid_size& sz) const {
    if (sz.j == 1) {
        if ( (sz.k != 1) && (sz.l != 1) ) {
            return true;
        } else { 
            return false;
        }
    } else if (sz.k == 1) {
        if ( (sz.j != 1) && (sz.l != 1) ) {
            return true;
        } else {
            return false;
        }
    } else if (sz.l == 1) {
        if ( (sz.j != 1) && (sz.k != 1) ) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::is_on_edge(const GridIndex& gidx) const {
    return ( (gidx.j == 0) || (gidx.j == sz_[gidx.zone].j - 1) ||
             (gidx.k == 0) || (gidx.k == sz_[gidx.zone].k - 1) );
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::are_adjacent(const GridIndex& gidx1, const GridIndex& gidx2) const {

    // check for simple adjacency
    if (are_simply_adj(gidx1, gidx2)) {
        return true;
    }

    // check if any overlapped points of gidx1 are simply adjacent to gidx2
    GridIndex over_gidx;
    auto oit = overlap_pts_.find(gidx2_nidx(gidx1));
    if (oit != overlap_pts_.end()) {
        for (int j=0; j < (*oit).second.size(); ++j) {
            over_gidx = nidx2_gidx((*oit).second[j]);
            if (are_simply_adj(over_gidx, gidx2)) {
                return true;
            }
        }
    }

    // check if any overlapped points of gidx2 are simply adjacent to gidx1
    oit = overlap_pts_.find(gidx2_nidx(gidx2));
    if (oit != overlap_pts_.end()) {
        for (int j=0; j < (*oit).second.size(); ++j) {
            over_gidx = nidx2_gidx((*oit).second[j]);
            if (are_simply_adj(over_gidx, gidx1)) {
                return true;
            }
        }
    }

    return false;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::are_simply_adj(const GridIndex& gidx1, const GridIndex& gidx2) const {
    if (gidx1.zone == gidx2.zone) {
        return ( std::abs(gidx1.j - gidx2.j) + std::abs(gidx1.k - gidx2.k) == 1 );
    }
    return false;
}

/*****************************************************************************/
template<typename FP>
GridIndex P3DModel_<FP>::get_adj_corner(const GridIndex& gidx, int corner) const {
    GridIndex gidx_out(gidx);

    switch (corner) {
        case 0:
            if (gidx.j > 0) {
                --gidx_out.j;
            }
            break;
        case 1:
            if (gidx.j < (sz_[gidx.zone].j-1)) {
                ++gidx_out.j;
            }
            break;  
        case 2:
            if (gidx.k > 0) {
                --gidx_out.k;
            } 
            break;
        case 3:
            if (gidx.k < (sz_[gidx.zone].k-1)) {
                ++gidx_out.k;
            }
            break;
    }
    return gidx_out;
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::find_adjacent(node_idx nidx, std::vector<node_idx>& adj) const {

    adj.clear();

    GridIndex gidx = nidx2_gidx(nidx);

    // First check all nodes that are simply adjacent
    GridIndex gidx_adj;
    for (int i=0; i < 4; ++i) {
        gidx_adj = get_adj_corner(gidx,i);
        if (gidx != gidx_adj) {
            adj.push_back(gidx2_nidx(gidx_adj));
        }
    }

    // Replace any adj with lower nidx
    for (int i=0; i < adj.size(); ++i) {
        adj[i] = get_low_nidx(adj[i]);
    }

    // For all nodes that overlap the node nidx, add all of their simply adjacent nodes
    auto it = overlap_pts_.find(nidx);
    if (it != overlap_pts_.end()) {
        for (int i=0; i < (*it).second.size(); ++i) {
            GridIndex gidx_over = nidx2_gidx( (*it).second[i] );
            for (int j=0; j < 4; ++j) {
                gidx_adj = get_adj_corner(gidx_over,j);
                if (gidx_over != gidx_adj) {
                    adj.push_back( gidx2_nidx(gidx_adj) );
                }
            }
        }
    }

    assert(adj.size() > 0);

    // Remove duplicate nodes
    auto ait1 = adj.begin();
    bool erase1;
    while (ait1 != adj.end()) {
        erase1 = false;
        auto ait2 = ait1+1;
        while (ait2 != adj.end()) {
            if ( cv::norm(get_position(*ait1) - get_position(*ait2)) <= overlap_tol_ ) {
                if ( (*ait1) < (*ait2) ) {
                    ait2 = adj.erase(ait2);
                    continue;
                } else {
                    erase1 = true;
                    ait1 = adj.erase(ait1);
                    break;
                }
            }
            ++ait2;
        } 
        if (!erase1) {
            ++ait1;
        }
    }

    std::sort(adj.begin(), adj.end());

}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::find_adjacent_faces(node_idx nidx, std::vector<Face>& adj) const {

    std::vector<node_idx> adj_nodes;
    find_adjacent(nidx, adj_nodes);

    // Attempt to create Faces with different combinations of adjacent nodes
    GridIndex gidx = nidx2_gidx(nidx);
    Face face;
    for (int i=0; i < adj_nodes.size(); ++i) {
        for (int j=i+1; j < adj_nodes.size(); ++j) {
            face = Face(this, nidx2_gidx(adj_nodes[i]), gidx, nidx2_gidx(adj_nodes[j]));
            if (face.is_valid()) {
                adj.push_back(face);
            }
        }
    }

}

/*****************************************************************************/
template<typename FP>
int P3DModel_<FP>::get_edge_id(const GridIndex& gidx) const {
    if (gidx.j == 0) return 0;
    if (gidx.j == sz_[gidx.zone].j-1) return 1;
    if (gidx.k == 0) return 2;
    if (gidx.k == sz_[gidx.zone].k-1) return 3;
    return -1;
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::node_idx P3DModel_<FP>::get_low_nidx(node_idx nidx) const {
    auto it = overlap_pts_.find(nidx);
    if (it != overlap_pts_.end()) {
        if ((*it).second[0] < nidx) {
            return (*it).second[0]; 
        }
    }
    return nidx;
}

template<typename FP>
void P3DModel_<FP>::calcNormals()
{
  enum{LL,LR,UR,UL,NADJ};

  const std::vector<FP>& xpos = this->get_x();
  const std::vector<FP>& ypos = this->get_y();
  const std::vector<FP>& zpos = this->get_z();
  cv::Point3_<FP> nullpos = cv::Point3_<FP>(0.0,0.0,0.0);

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

  // returns the area of a face: sum of it's two triangles
  //
  // ^  n3-n2
  // |  |  |
  // K  n0-n1
  //
  //     J->
  //
  auto faceArea = [this,triArea](node_idx n0, node_idx n1, node_idx n2, node_idx n3) -> FP {
      return triArea(n0,n1,n2) + triArea(n0,n2,n3) ;

  }; // faceArea()

  // calcs the normal in model coords, xforms to "run" coords
  //
  //    n0
  //     |
  //    n1 - n2
  //
  //

  auto faceNormal = [xpos,ypos,zpos,nullpos](node_idx n0, node_idx n1, node_idx n2) -> cv::Point3_<FP> {

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

  auto addFace = [this,faceNormal,faceArea,xpos,ypos,zpos](GridIndex dims, uint32_t base,
							   int nidx, int fidx, cv::Point3_<FP>& normal, bool dbg) {

      int idx = nidx - base;
      int row = idx / dims.j;
      int col = idx % dims.j;

      // +/- nodes

      int cm = base + row * dims.j + std::max(0,       col - 1); // -J
      int cp = base + row * dims.j + std::min(dims.j-1,col + 1); // +J
      int rm = base + std::max(0,       row - 1) * dims.j + col; // -K
      int rp = base + std::min(dims.k-1,row + 1) * dims.j + col; // +K

      int ll = base + std::max(0,       row - 1) * dims.j + std::max(0,       col - 1); // -J,-K
      int lr = base + std::max(0,       row - 1) * dims.j + std::min(dims.j-1,col + 1); // +J,-K
      int ur = base + std::min(dims.k-1,row + 1) * dims.j + std::min(dims.j-1,col + 1); // +J,+K
      int ul = base + std::min(dims.k-1,row + 1) * dims.j + std::max(0,       col - 1); // -J,+K

      int v[4]={0,0,0,0};

      switch( fidx ){
	  case LL:
	      if( cm==nidx or rm==nidx ) return; // not valid, no contribution
	      ASSERT( cm==nidx-1 and rm==nidx-dims.j );
	      v[0] = rm;   // cm -- nidx
	      v[1] = nidx; //  |    |
	      v[2] = cm;   // ll -- rm
	      v[3] = ll;
	  break;
	  case LR:
	      if( cp==nidx or rm==nidx ) return; // not valid, no contribution
	      ASSERT( cp==nidx+1 and rm==nidx-dims.j );
	      v[0] = cp;   // nidx -- cp
	      v[1] = nidx; //    |    |
	      v[2] = rm;   //   rm -- lr
	      v[3] = lr;
	  break;
	  case UR:
	      if( cp==nidx or rp==nidx ) return; // not valid, no contribution
	      ASSERT( cp==nidx+1 and rp==nidx+dims.j );
	      v[0] = rp;   //   rp -- ur
	      v[1] = nidx; //    |    |
	      v[2] = cp;   // nidx -- cp
	      v[3] = ur;
	  break;
	  case UL:
	      if( cm==nidx or rp==nidx ) return; // not valid, no contribution
	      ASSERT( cm==nidx-1 and rp==nidx+dims.j );
	      v[0] = cm;   // ul -- rp
	      v[1] = nidx; //  |    |
	      v[2] = rp;   // cm -- nidx
	      v[3] = ul;
	  break;
      }

      // only here if valid face

      FP area = faceArea( v[0],v[1],v[2],v[3] );
      //cv::Point3_<FP> fn = faceNormal(v[0],v[1],v[2],dbg) * area;
      cv::Point3_<FP> fn = faceNormal(v[0],v[1],v[2]);
      if( dbg ){
	  cv::Point3_<FP> v0 = this->get_position(v[0]);
	  cv::Point3_<FP> v1 = this->get_position(v[1]);
	  cv::Point3_<FP> v2 = this->get_position(v[2]);
	  std::cerr<<dtoa(v1.x)<<" "<<dtoa(v1.y)<<" "<<dtoa(v1.z)<<" P\n";
	  std::cerr<<dtoa(v1.x)<<" "<<dtoa(v1.y)<<" "<<dtoa(v1.z)<<" U\n";
	  std::cerr<<dtoa(v2.x)<<" "<<dtoa(v2.y)<<" "<<dtoa(v2.z)<<" U\n";
	  std::cerr<<dtoa(v1.x)<<" "<<dtoa(v1.y)<<" "<<dtoa(v1.z)<<" V\n";
	  std::cerr<<dtoa(v0.x)<<" "<<dtoa(v0.y)<<" "<<dtoa(v0.z)<<" V\n";
	  std::cerr<<dtoa(v1.x)<<" "<<dtoa(v1.y)<<" "<<dtoa(v1.z)<<" "<<dtoa(area)<<" N\n";
	  std::cerr<<dtoa(v1.x+fn.x)<<" "
		   <<dtoa(v1.y+fn.y)<<" "
		   <<dtoa(v1.z+fn.z)<<" "
		   <<dtoa(area)<<" N\n";
      }

      normal += fn;

  }; // addFace()

  auto calcNormal = [this,addFace,xpos,ypos,zpos](int _nidx_) -> cv::Point3_<FP> {

      // for (auto it=node.adj_face_begin(); it != node.adj_face_end(); ++it)

      //
      //  "simply" adjacent
      //  -------------------
      //  0 -> -j if possible
      //  1 -> +j if possible
      //  2 -> -k if possible
      //  3 -> +k if possible
      //
      // "overlap" adjacent
      //

      // loop over the faces adjacent to this node WITHOUT using an AdjacentFaceIterator
      //
      // - Node::adj_face_begin()
      //
      //   - Model::adj_face_begin(nidx)
      //
      //     - AdjacentFaceIterator::AdjacentFaceIterator(P3DModel*,GridIndex);
      //
      //       - Model::find_adjacent_faces(node_idx,std::vector<Face>&);
      //         - find all the faces of a node, including overlapped versions of that node
      //
      //         + gidx <= nidx
      //         + Model::find_adjacent(nidx,std::vector<node_idx>&)
      //         | - add simply adjacent nodes +/- J/K, store as lowest overlap
      //         | - add simply adjacent nodes of overlapping nodes 'corners'
      //         | - remove any nodes that overlap i.e. duplicates where dist<=overlap_tol_
      //         | - sort adj
      //         |
      //         + loop over adj nodes:
      //         |
      //         | - Face::Face
      //         |
      //         |   - NOTE Face::mode==null => INVALID FACE
      //         |   - gidx[s] must be valid [zone,i,j,k are ok]
      //         |
      //         |   - one of nodes must be adjacent [simple or overlap] to both others
      //         |     - 1:2 2:3
      //         |     - 1:2 1:3
      //         |     - 3:1 3:2
      //         |
      //         |   - gidx[s] must be ordered to preserve outwards normal
      //         |
      //         |     - Face::order_nodes(g1,g2,g3)
      //         |       - g's all from same zone
      //         |       - 1:2 2:3
      //         |       - abs(g1.jk-g2.jk) X (g3.jk-g2.jk) !=  1   =>   invalid face
      //         |       -    (g1.jk-g2.jk) X (g3.jk-g2.jk) == -1   =>   needs flip, std::swap(g1,g3)
      //         |
      //         |   - 4th node must be adj to 1 & 3
      //         |
      //         + add faces containing gidx
      //

      // add normals of all faces of this node
      // add normals of all overlapping versions of this node

      //
      //               ADJACENT FACES
      //
      //    N.j-1,N.k+1 --- N.j-1,N.k+1 --- N.j+1,N.k+1
      //         |               |               |
      //         |      UL       |       UR      |
      //         |               |               |
      //    N.j-1,N.k+0 --- N.j+0,N.k+0 --- N.j+1,N.k+0
      //         |               |               |
      //         |      LL       |       LR      |
      //         |               |               |
      //    N.j-1,N.k-1 --- N.j+0,N.k-1 --- N.j+1,N.k-1
      //
      cv::Point3_<FP> normal(0.0,0.0,0.0);

      bool dbg = false; // (_nidx_==17888);

      // add all the available faces for this node
      GridIndex gidx = this->nidx2_gidx(_nidx_);
      GridIndex dims = this->size(gidx.zone);
      uint32_t base = this->zone_start_idx(gidx.zone);
      for(int fidx = 0 ; fidx < NADJ ; ++ fidx ){
	  addFace( dims, base, _nidx_, fidx, normal,dbg );
      }

      // add all available faces for all overlapping nodes

      auto it = overlap_pts_.find(_nidx_);
      if( it != overlap_pts_.end() )
      {
	  std::vector<node_idx>& over = (*it).second;
	  for( auto oidx : over ){
	      DIE_IF( oidx==_nidx_ );
	      gidx = this->nidx2_gidx(oidx);
	      dims = this->size(gidx.zone);
	      base = this->zone_start_idx(gidx.zone);
	      if( dbg ){
		  char buf[PATH_MAX];
		  sprintf(buf,"T:%02d N:%6d Z:%6d O:%6d S:%6ld OVER\n",
			  omp_get_thread_num(),_nidx_,gidx.zone,oidx,over.size());
		  write(2,buf,strlen(buf));
	      }
	      for(int fidx = 0 ; fidx < NADJ ; ++ fidx ){
		  addFace( dims, base, oidx, fidx, normal,dbg );
	      }
	  }
      }

      FP nmag = cv::norm(normal);
      cv::Point3_<FP> res = ( nmag == 0.f ) ? normal : normal/nmag;

      if( dbg ){
	  cv::Point3_<FP> v0 = this->get_position(_nidx_);
	  std::cerr<<dtoa(v0.x)<<" "<<dtoa(v0.y)<<" "<<dtoa(v0.z)<<" F\n";
	  std::cerr<<dtoa(v0.x+res.x)<<" "<<dtoa(v0.y+res.y)<<" "<<dtoa(v0.z+res.z)<<" F\n";
      }
      return res;

  }; // calcNormal

  auto replicateNormal = [this](int nidx) -> cv::Point3_<FP> {
      cv::Point3_<FP> normal(0.0,0.0,0.0);
      Node node = this->node(nidx);
      for (auto it=node.adj_face_begin(); it != node.adj_face_end(); ++it) {
	  normal += (*it).get_normal() * (*it).get_area();
      }
      if (cv::norm(normal) == 0.0) {
	  return normal;
      } else {
	  return normal / cv::norm(normal);
      }
  }; // replicateNormal

  int nnodes = size(); // model::size()

  normals_.resize( nnodes, cv::Point3_<FP>(0.0,0.0,0.0) );

  // loop over nodes
#pragma omp parallel for
  for( int nidx=0 ; nidx<nnodes ; ++nidx )
  {
      //normals_[nidx] = replicateNormal(nidx);
      normals_[nidx] = calcNormal(nidx);
  } // loop over nodes

} // P3DModel_<FP>::calcNormals


/********************************************************************* 
 * Node Class
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::Node::Node(const P3DModel_* model, GridIndex gidx) :
        model_(const_cast<P3DModel_*>(model)) {

    assert(model != nullptr);

    // Check that the node is valid
    assert(model->is_valid_gidx(gidx));

    // Assign identifying information
    this->nidx_ = this->model_->gidx2_nidx(gidx);
    
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::Node::overlaps(node_idx n) const {
    return model_->get_low_nidx(nidx_) == model_->get_low_nidx(n);
}

/*****************************************************************************/

template<typename FP>
cv::Point3_<FP> P3DModel_<FP>::Node::get_normal() const {
    const auto& p2 = model_->normals_[get_idx()];
    if (cv::norm(p2) > 0.) return p2;

    cv::Point3_<FP> normal(0.0,0.0,0.0);
    GridIndex gidx = this->get_gidx();
    for (auto it=adj_face_begin(); it != adj_face_end(); ++it) {
        normal += (*it).get_normal() * (*it).get_area();
    }
    if (cv::norm(normal) == 0.0) {
        return normal;
    } else {
        return normal / cv::norm(normal);
    }
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::Node::operator==(const Node& node) const {
    if (this->model_ != node.model_) {
        return false;
    } else {
        return this->nidx_ == node.nidx_;
    }
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::Node::operator<(const Node& node) const {
    if (this->model_ != node.model_) {
        return this->model_ < node.model_;
    } else {
        return this->nidx_ < node.nidx_;
    }
}

/********************************************************************* 
 * Face Class 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::Face::Face(const P3DModel_* model, GridIndex gidx1, GridIndex gidx2, 
        GridIndex gidx3) : model_(const_cast<P3DModel_*>(model)) {

    assert(model != nullptr);

    // Check that all input nodes are valid
    if (!model->is_valid_gidx(gidx1)) {
        return;
    } 
    if (!model->is_valid_gidx(gidx2)) {
        return;
    } 
    if (!model->is_valid_gidx(gidx3)) {
        return;
    }

    // Check that one of the input points is adjacent to both others and
    // rearrange points so that 1 and 3 are adjacent to the 4th point
    GridIndex tmp;
    if (!(model->are_adjacent(gidx2,gidx1) && model->are_adjacent(gidx2,gidx3))) {
        if ( model->are_adjacent(gidx1,gidx2) && model->are_adjacent(gidx1,gidx3) ) {
            tmp = gidx1;
            gidx1 = gidx2;
            gidx2 = tmp;
        } else if ( model->are_adjacent(gidx3,gidx1) && model->are_adjacent(gidx3,gidx2) ) {
            tmp = gidx2;
            gidx2 = gidx3;
            gidx3 = tmp;
        } else {
            // create invalid face
            model_ = nullptr;
            return;
        }
    } 

    // Get the face index and move all gidx to the common zone
    // check if a face cannot be formed
    int fidx = move_to_common(gidx1, gidx2, gidx3);
    if (fidx < 0) {
        //std::cout << "Failed to find common zone: " << gidx1 << ", " << gidx2;
        //std::cout << ", " << gidx3 << std::endl;
        model_ = nullptr;
        return;
    } else {
        fidx_ = static_cast<face_idx>(fidx);
    }

    // Rearrange gidx1,gidx3 if needed for outward normal
    order_nodes(gidx1,gidx2,gidx3);

    // Ensure that the nodes are the lowest nidx of all overlapping points
    node_idx nidx1 = model->gidx2_nidx(gidx1);
    node_idx nidx2 = model->gidx2_nidx(gidx2);
    node_idx nidx3 = model->gidx2_nidx(gidx3);
    nidx1 = model->get_low_nidx(nidx1);
    nidx2 = model->get_low_nidx(nidx2);
    nidx3 = model->get_low_nidx(nidx3);

    // Identify the 4th node
    node_idx nidx4;
    std::vector<node_idx> adj_1;
    std::vector<node_idx> adj_3;
    model->find_adjacent(nidx1,adj_1);
    model->find_adjacent(nidx3,adj_3);
    for (int i=0; i < adj_1.size(); ++i) {
        if ( adj_1[i] != nidx2 ) {
            for (int j=0; j < adj_3.size(); ++j) {
                if ( adj_1[i] == adj_3[j] ) {
                    nidx4 = adj_1[i];
                    
                    // Assign identifying information
                    nidx_ = std::vector<node_idx>({nidx1,nidx2,nidx3,nidx4});

                    return;
                }
            }
        }
    }

    // Could not find a valid 4th node, Face is invalid
    model_ = nullptr;   
}

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::Face::Face(const P3DModel_* model, face_idx fidx) :
        model_(const_cast<P3DModel_*>(model)), fidx_(fidx) {
     
    assert(model != nullptr);
    assert(fidx_ < model_->number_of_faces());

    // Grab the nodes that make up this face
    // put in order, s.t. that the normal direction is correct
    GridIndex g1 = model_->fidx2_min_gidx(fidx_);
    GridIndex g2 = g1;
    g2.j += 1;
    GridIndex g3 = g2;
    g3.k += 1;
    GridIndex g4 = g1;
    g4.k += 1;

    // Get the lowest nidx of all overlapping points
    nidx_.resize(4);
    nidx_[0] = model_->get_low_nidx(model_->gidx2_nidx(g1));
    nidx_[1] = model_->get_low_nidx(model_->gidx2_nidx(g2));
    nidx_[2] = model_->get_low_nidx(model_->gidx2_nidx(g3));
    nidx_[3] = model_->get_low_nidx(model_->gidx2_nidx(g4));
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::Face::has_node(node_idx nidx) const {
    nidx = model_->get_low_nidx(nidx);
    for (int i=0; i < nidx_.size(); ++i) {
        if (nidx == nidx_[i]) {
            return true;
        }
    }
    return false;
}

/*****************************************************************************/
template<typename FP>
cv::Point3_<FP> P3DModel_<FP>::Face::get_normal() const {
    upsp::Triangle<FP> tri(model_->get_position(nidx_[0]),model_->get_position(nidx_[1]),
            model_->get_position(nidx_[2]));
    return upsp::normal(tri);
}

/*****************************************************************************/
template<typename FP>
double P3DModel_<FP>::Face::get_area() const {
    upsp::Triangle<FP> tri1(model_->get_position(nidx_[0]), model_->get_position(nidx_[1]),
            model_->get_position(nidx_[2]));
    upsp::Triangle<FP> tri2(model_->get_position(nidx_[0]), model_->get_position(nidx_[2]),
            model_->get_position(nidx_[3]));

    return upsp::area(tri1) + upsp::area(tri2);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Face P3DModel_<FP>::Face::get_adjacent(GridDir g_dir) const {
   
    // Get the minimum GridIndex in this face (in the zone of the face)
    GridIndex min_gidx = model_->fidx2_min_gidx(fidx_);

    int zone = min_gidx.zone;    
    int max_j = min_gidx.j + 1;
    int max_k = min_gidx.k + 1;

    // Store 3 Nodes from the next face 
    GridIndex adj_gidx1;
    GridIndex adj_gidx2;
    GridIndex adj_gidx3;

    // Get the next face, if simple (within the same zone) then just grab it
    // otherwise, use overlapping points to find it
    bool is_simple = true;
    switch (g_dir) {
        case GridDir::J_plus :
            // Define nodes on the edge where we want a new face
            adj_gidx1 = GridIndex(zone, max_j, max_k  , 0);
            adj_gidx2 = GridIndex(zone, max_j, max_k-1, 0);

            if ((max_j+1) < model_->sz_[zone].j) {
                // the adjacent face is within this zone, simply grab it
                adj_gidx3 = GridIndex(zone, max_j+1, max_k, 0);
            } else {
                is_simple = false;
            }
            break;
        case GridDir::J_minus :
            // Define nodes on the edge where we want a new face
            adj_gidx1 = GridIndex(zone, max_j-1, max_k  , 0);
            adj_gidx2 = GridIndex(zone, max_j-1, max_k-1, 0);

            if ((max_j-1) > 0) {
                // the adjacent face is within this zone, simply grab it
                adj_gidx3 = GridIndex(zone, max_j-2, max_k, 0);
            } else {
                is_simple = false;
            }
            break;
        case GridDir::K_plus : 
            // Define nodes on the edge where we want a new face
            adj_gidx1 = GridIndex(zone, max_j  , max_k, 0);
            adj_gidx2 = GridIndex(zone, max_j-1, max_k, 0);

            if ((max_k+1) < model_->sz_[zone].k) {
                // the adjacent face is within this zone, simply grab it
                adj_gidx3 = GridIndex(zone, max_j, max_k+1, 0);
            } else {
                is_simple = false;
            }
            break;
        case GridDir::K_minus : 
            // Define nodes on the edge where we want a new face
            adj_gidx1 = GridIndex(zone, max_j  , max_k-1, 0);
            adj_gidx2 = GridIndex(zone, max_j-1, max_k-1, 0);

            if ((max_k-1) > 0) {
                // the adjacent face is within this zone, simply grab it
                adj_gidx3 = GridIndex(zone, max_j, max_k-2, 0);
            } else {
                is_simple = false;
            }
            break;
    }

    // handle cases where overlapping points need to be investigated
    if (!is_simple) {                
        // define nodes on the edge where we want a new face
        node_idx n1 = model_->gidx2_nidx(adj_gidx1);
        node_idx n2 = model_->gidx2_nidx(adj_gidx2);
        GridIndex new_gidx1;
        GridIndex new_gidx2;
        bool found_match = false;
        if ( (model_->overlap_pts_.find(n1) != model_->overlap_pts_.end()) &&
             (model_->overlap_pts_.find(n2) != model_->overlap_pts_.end()) ) {
            for (auto it=model_->overlap_pts_.at(n1).begin(); 
                    it != model_->overlap_pts_.at(n1).end(); ++it) {
                new_gidx1 = model_->nidx2_gidx(*it);
                for (auto it2=model_->overlap_pts_.at(n2).begin(); 
                        it2 != model_->overlap_pts_.at(n2).end(); ++it2) {
                    new_gidx2 = model_->nidx2_gidx(*it2);
                    if (new_gidx1.zone == new_gidx2.zone) {
                        found_match = true;
                        break;
                    }
                }
                if (found_match) {
                    break;
                }   
            }
        }
        if (found_match) {
            // go ahead and convert the adj_gidx to the new_gidx since it
            // will make it easier to form the Face if they are all in the
            // same zone
            adj_gidx1 = new_gidx1;
            adj_gidx2 = new_gidx2;

            // determine if these are aligned in j or k
            if (new_gidx1.j == new_gidx2.j) {
                // determine if j=0 or j=-1
                if (new_gidx1.j == 0) {
                    adj_gidx3 = GridIndex(new_gidx1.zone, 1, new_gidx1.k, 0);
                } else {
                    adj_gidx3 = GridIndex(new_gidx1.zone, new_gidx1.j-1, new_gidx1.k, 0);
                }
            } else {
                // determine if k=0 or k=-1
                if (new_gidx1.k == 0) {
                    adj_gidx3 = GridIndex(new_gidx1.zone, new_gidx1.j, 1, 0);
                } else {
                    adj_gidx3 = GridIndex(new_gidx1.zone, new_gidx1.j, new_gidx1.k-1, 0);
                }
            }
        } else {
            GridIndex gidx(0,0,0,0);
            return model_->face(gidx,gidx,gidx); // creates an invalid Face
        }
    }

    return model_->face(adj_gidx1, adj_gidx2, adj_gidx3);
}

/*****************************************************************************/
template<typename FP>
std::array<typename P3DModel_<FP>::Node,2> P3DModel_<FP>::Face::get_edge_nodes(
        GridDir g_dir) const {

    GridIndex gidx_min = model_->fidx2_min_gidx(fidx_);
    GridIndex gidx1(gidx_min);
    GridIndex gidx2(gidx_min);

    switch (g_dir) {
        case GridDir::J_plus : 
            ++gidx1.j;
            ++gidx2.j;
            ++gidx2.k;
            break;
        case GridDir::J_minus :
            ++gidx2.k;
            break;
        case GridDir::K_plus :
            ++gidx1.k;
            ++gidx2.k;
            ++gidx2.j;
            break;
        case GridDir::K_minus :
            ++gidx2.j;
            break;
    }

    return {model_->node(gidx1), model_->node(gidx2)};
}

/*****************************************************************************/
template<typename FP>
double P3DModel_<FP>::Face::intersects(const upsp::Ray<FP>& r) const {

    // Split Face into 2 triangles
    upsp::Triangle<FP> tri1(model_->get_position(nidx_[0]),model_->get_position(nidx_[1]),
            model_->get_position(nidx_[2]));
    upsp::Triangle<FP> tri2(model_->get_position(nidx_[0]),model_->get_position(nidx_[2]),
            model_->get_position(nidx_[3]));

    // Try to intersect with each triangle
    FP dist1 = upsp::intersects(tri1,r);
    FP dist2 = upsp::intersects(tri2,r);

    if (dist1 > 0.0) {
        return dist1;
    } else {
        return dist2;
    }
}

/*****************************************************************************/
template<typename FP>
int P3DModel_<FP>::Face::get_zone() const {
    GridIndex min_gidx = model_->fidx2_min_gidx(fidx_);
    return min_gidx.zone;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::Face::operator==(const Face& face) const {
    if (this->model_ != face.model_) {
        return false;
    } else {
        if (this->nidx_.size() != face.nidx_.size() ) {
            return false;
        } else {
            int count = 0;
            for (int i=0; i < this->nidx_.size(); ++i) {
                for (int j=0; j < face.nidx_.size(); ++j) {
                    if (this->nidx_[i] == face.nidx_[j]) {
                        ++count;
                        break;
                    }
                }
            }
            return count >= 3;
        }
    }
}

/*****************************************************************************/
template<typename FP>
int P3DModel_<FP>::Face::move_to_common(GridIndex& gidx1, GridIndex& gidx2, 
                                        GridIndex& gidx3) const {

    typedef std::map<node_idx,std::vector<node_idx>>::iterator over_it;

    int common_zone = -1;
    std::array<GridIndex,3> gids({gidx1,gidx2,gidx3});

    // Put all of the gidx into a common zone and ensure they are simply adjacent
    if (!model_->are_simply_adj(gids[0],gids[1]) || 
        (!model_->are_simply_adj(gids[2],gids[1])) ) {

        // get the node indices for each grid index
        // and get iterators for the overlap vectors
        // if any node has no overlap, then it is in the common zone
        std::array<node_idx,3> nids;
        std::array<over_it,3> its;
        unsigned int anchor = 0;
        for (unsigned int i=0; i < 3; ++i) {
            nids[i] = model_->gidx2_nidx(gids[i]);
            its[i]  = model_->overlap_pts_.find(nids[i]);

            if (its[i] == model_->overlap_pts_.end()) {
                common_zone = gids[i].zone;
                anchor = i;
            }
        }

        // Change the gids over to the common zone if it was found in the first step
        // remember that some grids can be periodic
        // remember that the node may not be adjacent to the anchor
        if (common_zone != -1) {
            int anchorj = static_cast<int>(gids[anchor].j);
            int anchork = static_cast<int>(gids[anchor].k);
            for (unsigned int i=0; i < 3; ++i) {
                if (i == anchor) {
                    continue;
                }
                if (its[i] != model_->overlap_pts_.end()) {
                    if ( (abs(static_cast<int>(gids[i].j) - anchorj) > 1) ||
                            (abs(static_cast<int>(gids[i].k) - anchork) > 1) ) {
                        for (auto o_it=model_->overlap_pts_.at(nids[i]).begin(); 
                                o_it != model_->overlap_pts_.at(nids[i]).end(); ++o_it) {
                            gids[i] = model_->nidx2_gidx(*o_it);
                            if ( (abs(static_cast<int>(gids[i].j) - anchorj) <= 1) &&
                                    (abs(static_cast<int>(gids[i].k) - anchork) <= 1) ) {
                                break;
                            }
                        }
                    }
                }
            }
        } else {
            // if all nodes have overlap, then search for a common zone

            // gather all nodes that could be used for gidx2
            std::vector<GridIndex> all_gidx2(1,gidx2);
            if (its[1] != model_->overlap_pts_.end()) {
                for (auto it = model_->overlap_pts_.at(nids[1]).begin();
                        it != model_->overlap_pts_.at(nids[1]).end(); ++it) {
                    all_gidx2.push_back(model_->nidx2_gidx(*it));
                }
            }

            // outer loop changes gidx2 (since gidx1 and gidx3 are adjacent)
            for (auto o_it = all_gidx2.begin(); o_it != all_gidx2.end(); ++o_it) {
                bool found2 = false;
                bool found3 = false;
                gids[1] = *o_it;

                // search gidx1 for an overlap
                if (model_->are_simply_adj(gidx1, gids[1])) {
                    gids[0] = gidx1;
                    found2 = true;
                } else if (its[1] != model_->overlap_pts_.end()) {
                    for (auto i_it = model_->overlap_pts_.at(nids[0]).begin();
                            i_it != model_->overlap_pts_.at(nids[0]).end(); ++i_it) {
                        gids[0] = model_->nidx2_gidx(*i_it);
                        if (model_->are_simply_adj(gids[1],gids[0])) {
                            found2 = true;
                            break;
                        }
                    }
                }
                // search gidx3 for an overlap
                if (model_->are_simply_adj(gidx3, gids[1])) {
                    gids[2] = gidx3;
                    found2 = true;
                } else if (its[1] != model_->overlap_pts_.end()) {
                    for (auto i_it = model_->overlap_pts_.at(nids[2]).begin();
                            i_it != model_->overlap_pts_.at(nids[2]).end(); ++i_it) {
                        gids[2] = model_->nidx2_gidx(*i_it);
                        if (model_->are_simply_adj(gids[1],gids[2])) {
                            found3 = true;
                            break;
                        }
                    }
                }

                if (found2 && found3) {
                    break;
                }
            }

        }
    }

    // Check if successfully found a common zone with simple adjacency
    // if not, return an invalid index
    if (!model_->are_simply_adj(gids[0],gids[1]) || 
        (!model_->are_simply_adj(gids[2],gids[1])) ) {
        //std::cout << " not simply adjacent: " << gids[0] << ", " << gids[1] << ", ";
        //std::cout << gids[2] << std::endl;
        return -1;
    }

    // Also check for case where the gridpoints form a line (rather than an L)
    if ( (gids[0].j == gids[1].j) && (gids[0].j == gids[2].j) ) {
        return -1;
    }
    if ( (gids[0].k == gids[1].k) && (gids[0].k == gids[2].k) ) {
        return -1;
    }

    // Now should have all gids in the same zone and simply adjacent
    // find the face_idx
    int fidx = 0;
    for (unsigned int i=0; i < model_->sz_.size(); ++i) {
        if (gids[0].zone == i) {
            break;
        }
        fidx += (model_->sz_[i].j - 1) * (model_->sz_[i].k - 1);
    }
    int min_j = min(gids[0].j, gids[1].j, gids[2].j);
    int min_k = min(gids[0].k, gids[1].k, gids[2].k);

    fidx += min_k * (model_->sz_[gids[0].zone].j - 1) + min_j;

    // Set the new grid indices
    gidx1 = gids[0];
    gidx2 = gids[1];
    gidx3 = gids[2];

    return fidx;
}

/*****************************************************************************/
template<typename FP>
void P3DModel_<FP>::Face::order_nodes(GridIndex& gidx1, GridIndex& gidx2, GridIndex& gidx3) const {

    // Flip gidx1 and gidx3 if needed for outward normal 
    cv::Point2i vec1 = cv::Point2i(gidx1.j - gidx2.j, gidx1.k - gidx2.k);
    cv::Point2i vec2 = cv::Point2i(gidx3.j - gidx2.j, gidx3.k - gidx2.k);

    // Flip gidx1 and gidx3 if needed for outward normal 
    if ( vec2.cross(vec1) == -1) {
        GridIndex tmp(gidx1);
        gidx1 = gidx3;
        gidx3 = tmp;
    }
}

/********************************************************************* 
 * NodeIterator Class 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::NodeIterator::NodeIterator(const P3DModel_* model, GridIndex gidx) :
        model_(const_cast<P3DModel_*>(model)) {

    assert(model != nullptr);

    // Check that the zone is valid
    assert(gidx.zone >= 0);

    // Consider case of an end iterator
    if (gidx.zone >= model->zones_) {
        nidx_ = model->x_.size();
    } else {
        // Check that the indices are valid
        assert( (gidx.j >= 0) && (gidx.j < model->sz_[gidx.zone].j) );
        assert( (gidx.k >= 0) && (gidx.k < model->sz_[gidx.zone].k) );
        assert( (gidx.l >= 0) && (gidx.l < model->sz_[gidx.zone].l) );

        // Assign information to iterator
        nidx_ = model->gidx2_nidx(gidx);
    }
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Node P3DModel_<FP>::NodeIterator::operator*() const {
    return Node(this->model_,this->model_->nidx2_gidx(nidx_));
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::NodeIterator& P3DModel_<FP>::NodeIterator::operator++() {

    // Check if this point has an overlap with lower nidx
    while (nidx_ < this->model_->x_.size()) {   
        ++nidx_;    

        auto it = this->model_->overlap_pts_.find(nidx_);
        if (it != this->model_->overlap_pts_.end()) {
            if ( (*it).second[0] >= nidx_) {
                break;
            }
        } else {
            break;
        }
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::NodeIterator::operator==(const NodeIterator& ni) const {
    if (ni.model_ != this->model_) {
        return false;
    } else {
        if ( (ni.nidx_ >= ni.model_->x_.size()) && 
                (this->nidx_ >= this->model_->x_.size()) ) {
            return true;  // end iterator
        } else {
            return (ni.nidx_ == this->nidx_);
        }
    }
}

/********************************************************************* 
 * FaceIterator Class 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::FaceIterator::FaceIterator(const P3DModel_* model, face_idx fidx) : 
        model_(const_cast<P3DModel_*>(model)), fidx_(fidx) {

    assert(model != nullptr);

    // Consider case of an end iterator
    if (fidx >= model_->number_of_faces()) {
        fidx_ = model_->number_of_faces();
    }
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Face P3DModel_<FP>::FaceIterator::operator*() const {
    return Face(model_,fidx_);
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::FaceIterator& P3DModel_<FP>::FaceIterator::operator++() {
    ++fidx_;
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::FaceIterator::operator==(const FaceIterator& fi) const {
    if (fi.model_ != this->model_) {
        return false;
    } else {
        return (fi.fidx_ == this->fidx_);
    }
}


/********************************************************************* 
 * EdgeIterator Class 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::EdgeIterator::EdgeIterator(const P3DModel_* model, int zone, bool end) : 
        model_(const_cast<P3DModel_*>(model)) {                                          
                                                    
    assert(model != nullptr);                          
                                                    
    // Check that zone is valid                     
    assert(zone >= 0);                          
    assert(zone < model->zones_);             
                                                    
    // Assign identifying information
    nidx_ = this->model_->gidx2_nidx(GridIndex(zone,0,0,0));

    // Set starting path
    if (end) {
        path_ = 2;
    } else {
        path_ = 0;
    }
} 

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Node P3DModel_<FP>::EdgeIterator::operator*() const {
    return Node(this->model_,this->model_->nidx2_gidx(this->nidx_));
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::EdgeIterator& P3DModel_<FP>::EdgeIterator::operator++() {
    GridIndex gidx = this->model_->nidx2_gidx(this->nidx_);
    if (path_ == 0) {
        if (gidx.k == (model_->sz_[gidx.zone].k-1) ) {
            path_ = 3;
            ++gidx.j;
        } else {
            ++gidx.k;
        }
    } else if (path_ == 1) {
        if (gidx.k == 0) {
            path_ = 2;
            --gidx.j;
        } else {
            --gidx.k;
        }
    } else if (path_ == 2) {
        if (gidx.j == 0) {
            path_ = 0;
            ++gidx.k;
        } else {
            --gidx.j;
        }
    } else {
        if (gidx.j == (model_->sz_[gidx.zone].j-1) ) {
            path_ = 1;
            --gidx.k;
        } else {
            ++gidx.j;
        }
    }

    this->nidx_ = this->model_->gidx2_nidx(gidx);

    return *this;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::EdgeIterator::operator==(const EdgeIterator& ei) const {
    return ( (this->model_ == ei.model_) && (this->nidx_ == ei.nidx_) && 
            (this->path_ == ei.path_) );
}

/********************************************************************* 
 * AdjNodeIterator Class 
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::AdjNodeIterator::AdjNodeIterator(const P3DModel_* model, GridIndex gidx,
        bool end) : model_(const_cast<P3DModel_*>(model)) {

    assert(model != nullptr);

    // Check that the grid index is valid
    assert(model->is_valid_gidx(gidx));

    nidx_ = model->gidx2_nidx(gidx);

    // If not end, get find all adjacent nodes
    if (!end) {
        model->find_adjacent(nidx_,adj_);
    }
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Node P3DModel_<FP>::AdjNodeIterator::operator*() const {
    return Node(model_,model_->nidx2_gidx(adj_[adj_.size()-1]));    
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjNodeIterator& P3DModel_<FP>::AdjNodeIterator::operator++() {
    if (adj_.size() > 0) {
        adj_.pop_back();
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::AdjNodeIterator::operator==(const AdjNodeIterator& ani) const {
    if (this->model_ != ani.model_) {
        return false;
    } else {
        if (this->nidx_ != ani.nidx_) {
            return false;
        } else {
            if (this->adj_.size() != ani.adj_.size()) {
                return false;
            } else {    
                for (int i=0; i < this->adj_.size(); ++i) {
                    if (this->adj_[i] != ani.adj_[i]) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Node P3DModel_<FP>::AdjNodeIterator::get_center() const {
    return Node(this->model_,this->model_->nidx2_gidx(nidx_));
}

/********************************************************************* 
 * AdjFaceIterator Class
 ********************************************************************/ 

/*****************************************************************************/
template<typename FP>
P3DModel_<FP>::AdjFaceIterator::AdjFaceIterator(const P3DModel_* model, GridIndex gidx,
        bool end) : model_(const_cast<P3DModel_*>(model)) {

    assert(model != nullptr);

    // Check that the grid index is valid
    assert(model->is_valid_gidx(gidx));

    nidx_ = model->gidx2_nidx(gidx);

    // If not end, get find all adjacent faces
    if (!end) {
        model->find_adjacent_faces(nidx_,adj_);
    }
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::Face P3DModel_<FP>::AdjFaceIterator::operator*() const {
    return this->adj_[this->adj_.size()-1]; 
}

/*****************************************************************************/
template<typename FP>
typename P3DModel_<FP>::AdjFaceIterator& P3DModel_<FP>::AdjFaceIterator::operator++() {
    if (adj_.size() > 0) {
        adj_.pop_back();
    }
    return *this;
}

/*****************************************************************************/
template<typename FP>
bool P3DModel_<FP>::AdjFaceIterator::operator==(const AdjFaceIterator& ani) const {
    if (this->model_ != ani.model_) {
        return false;
    } else {
        if (this->nidx_ != ani.nidx_) {
            return false;
        } else {
            if (this->adj_.size() != ani.adj_.size()) {
                return false;
            } else {    
                for (int i=0; i < this->adj_.size(); ++i) {
                    if (this->adj_[i] != ani.adj_[i]) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

} /* end namespace upsp */
