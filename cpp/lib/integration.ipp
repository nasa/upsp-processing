/** @file
 *  @brief  Integration over panels
 *  @date   May 1, 2019
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename FP>
Panels<FP>::Panels(const StructuredGrid<FP>& grid, float height_sf/*=1.0*/) {

    assert(grid.num_zones() > 0);

    // Determine if the input grid is a surface or volume grid
    bool is_surface = false;

    if ( (grid.grid_size[0][0] == 1) || (grid.grid_size[0][1] == 1) || 
            (grid.grid_size[0][2] == 1) ) {
        is_surface = true;
    }

    // Create the surface or volume grid
    if (is_surface) {
        // If the grid is a surface grid, copy over the surfaces and 
        // build the hexes
        surfaces.load_grid(grid, 1e-3);

        for (auto p_it=surfaces.face_begin(); p_it != surfaces.face_end();
                ++p_it) {
            // Create a polyhedron that bounds all nodes/triangles
            // that could be a part of this panel
            hexes.push_back(build_polyhedron<FP>(*p_it, height_sf));
        }
    } else {
        // If the grid is a volume grid, convert the volume grid to 
        // hexes and build the surface mesh
        
        // Build the surface grid
        StructuredGrid<FP> surf_grid;
        surf_grid.grid_size.resize(grid.num_zones());

        surf_grid.x.resize(grid.size() / 2);
        surf_grid.y.resize(grid.size() / 2);
        surf_grid.z.resize(grid.size() / 2);
    
        unsigned int count = 0;
        for (unsigned int z=0; z < grid.num_zones(); ++z) {
            assert(grid.zone_size(z, 2) == 2);

            unsigned int j_len = grid.zone_size(z, 0);
            unsigned int k_len = grid.zone_size(z, 1);
            unsigned int z_sz = j_len * k_len;

            surf_grid.grid_size[z] = {j_len,k_len,1};

            unsigned int z_start = grid.zone_start_idx(z);             

            for (unsigned int k=0; k < k_len; ++k) {
                for (unsigned int j=0; j < j_len; ++j) {
                    FP new_x = 0.5*(grid.x[z_start + j + k*j_len] + 
                                    grid.x[z_start + j + k*j_len + z_sz]);
                    FP new_y = 0.5*(grid.y[z_start + j + k*j_len] + 
                                    grid.y[z_start + j + k*j_len + z_sz]);
                    FP new_z = 0.5*(grid.z[z_start + j + k*j_len] + 
                                    grid.z[z_start + j + k*j_len + z_sz]);

                    surf_grid.x[count + j + k*j_len] = new_x;
                    surf_grid.y[count + j + k*j_len] = new_y;
                    surf_grid.z[count + j + k*j_len] = new_z;
               }
            }
            count += z_sz;
        }

        surfaces.load_grid(surf_grid);

        // Build the hexahedrons
        hexes = build_polyhedrons(grid);
    }
}

/*****************************************************************************/
template<typename FP>
bool PanelIntegration<FP>::is_valid() const {

    // check that the size of the matrices are all the same
    if ( (fx.rows() != fy.rows()) || (fx.rows() != fz.rows()) || 
         (fx.rows() != mx.rows()) || (fx.rows() != my.rows()) ||
         (fx.rows() != mz.rows()) ) {
        std::cout << "Failed panel integration validity test 1" << std::endl;
        return false;
    }
    if ( (fx.cols() != fy.cols()) || (fx.cols() != fz.cols()) || 
         (fx.cols() != mx.cols()) || (fx.cols() != my.cols()) ||
         (fx.cols() != mz.cols()) ) {
        std::cout << "Failed panel integration validity test 2" << std::endl;
        return false;
    }

    // check that the size of the other vectors are equal to the right
    // matrix direction
    unsigned int vec_size; // number of panels
    if (transpose) {
        vec_size = fx.rows();
    } else {
        vec_size = fx.cols();
    }

    if ( (center_x.size() != vec_size) || (center_y.size() != vec_size) ||
         (center_z.size() != vec_size) || (area.size() != vec_size) ||
         (coverage.size() != vec_size) ) {
        std::cout << "Failed panel integration validity test 3" << std::endl;
        return false;
    }

    return true;
}

/*****************************************************************************/
template<typename FP>
PanelIntegration<FP> create_integration_matrices(const TriModel_<FP>& tri,
        const Panels<FP>& panels, bool nodal, 
        const std::vector<bool>& coverage) {

    typedef typename Eigen::SparseMatrix<FP> SpMat;
    typedef Eigen::Triplet<FP> Trip;
    typedef Polyhedron<6,FP> Hexa;
    typedef TriModel_<FP> Model;
    typedef typename TriModel_<FP>::node_idx node_idx;
    typedef typename TriModel_<FP>::face_idx face_idx;
    typedef typename TriModel_<FP>::simple_tri simple_tri;
    typedef typename TriModel_<FP>::Face Face;
    typedef typename TriModel_<FP>::Node Node;

    unsigned int n_panels = panels.size();
    unsigned int n_dp;
    if (nodal) {
        n_dp = tri.size();
    } else {
        n_dp = tri.number_of_faces();
    }

    assert(coverage.size() == n_dp);

    FP contains_tol = 0.0001;

    // Initialize the output data
    PanelIntegration<FP> p_integ;
    p_integ.fx = SpMat(n_panels, n_dp);
    p_integ.fy = SpMat(n_panels, n_dp);
    p_integ.fz = SpMat(n_panels, n_dp);
    p_integ.mx = SpMat(n_panels, n_dp);
    p_integ.my = SpMat(n_panels, n_dp);
    p_integ.mz = SpMat(n_panels, n_dp);
    p_integ.center_x = std::vector<FP>(n_panels, 0.0);
    p_integ.center_y = std::vector<FP>(n_panels, 0.0);
    p_integ.center_z = std::vector<FP>(n_panels, 0.0);
    p_integ.coverage = std::vector<FP>(n_panels, 0.0);
    p_integ.area     = std::vector<FP>(n_panels, 0.0);

    p_integ.transpose = true;

    // For faster building, use triplets
    std::vector<Trip> trip_fx;
    std::vector<Trip> trip_fy;
    std::vector<Trip> trip_fz;
    std::vector<Trip> trip_mx;
    std::vector<Trip> trip_my;
    std::vector<Trip> trip_mz;

    // Build an Octree for the TriModel
    BoundingBox_<cv::Point3_<FP>> bb = get_extent(
            boost::make_transform_iterator(tri.cnode_begin(), Node2_Point<Model>()),
            boost::make_transform_iterator(tri.cnode_end(), Node2_Point<Model>()));
    Octree<TriModel_<FP>,Face> tree(&tri, tri.cface_begin(), tri.cface_end(), bb, 30);

    //tree.write_tree_structure("test_tree.dat");

    // For each panel, get the triangles within that panel (splitting as needed)
    // then, add those results to the integration matrices
    // and log the amount of coverge provided by the data
    unsigned int p_count = 0;
    for (auto it = panels.surfaces.face_begin(); it != panels.surfaces.face_end(); 
            ++it) {
        //std::cout << std::endl << "Panel " << p_count << std::endl;
    
        // Set variables for coverage calculation
        // will use tri grid to get more accurate area calculation
        FP total_area = 0.0;
        FP cover_area = 0.0;

        // Compute the panel centroid
        std::vector<node_idx> nodes = (*it).get_nodes();
        for (unsigned int i=0; i < nodes.size(); ++i) {
            cv::Point3_<FP> pos = panels.surfaces.get_position(nodes[i]);
            p_integ.center_x[p_count] += pos.x / nodes.size();
            p_integ.center_y[p_count] += pos.y / nodes.size();
            p_integ.center_z[p_count] += pos.z / nodes.size();
        }
        cv::Point3_<FP> p_center(p_integ.center_x[p_count], p_integ.center_y[p_count],
                                 p_integ.center_z[p_count]);
    
        // Create a polyhedron that bounds all nodes/triangles
        // that could be a part of this panel
        Hexa hex = panels.hexes[p_count];

        // Find all of the triangles that may be intersected
        std::set<face_idx> faces = approx_faces_within(hex, tree);

        /*
        std::cout << "Found nodes: ";
        for (auto s_it = nodes.begin(); s_it != nodes.end(); ++s_it) {
            std::cout << (*s_it) << ", ";
        }
        std::cout << std::endl;
        std::cout << "Found faces: ";
        for (auto s_it = faces.begin(); s_it != faces.end(); ++s_it) {
            std::cout << (*s_it) << ", ";
        }
        std::cout << std::endl;
        */

        // Add each triangle to the integration matrices
        // if it is not entirely within the polyhedron, handle the partial triangle
        for (auto f_it = faces.begin(); f_it != faces.end(); ++f_it) {

            face_idx fidx = *f_it;
            Face f = tri.cface(fidx);

            // if the face is nondata, skip
            if (!tri.is_dataface(fidx)) {
                continue;
            }

            // build simple triangle structs
            Triangle<FP> tri_elem = f.get_triangle();
            simple_tri simp = f.nodes();

            //std::cout << "Face " << fidx << std::endl;

            // Get the negative face normal direction
            // this is the direction the pressure is recorded in
            cv::Point3_<FP> normal = -1.0* f.get_normal();

            // form a vector of all sub-faces that need to be included
            std::vector<SplitTri<FP>> stris;

            // Check if tri is within poly, to determine if splitting is needed
            if (contains(hex, tri_elem)) {
                stris.push_back(SplitTri<FP>(tri_elem));
            } else {
                // not all of the triangle is part of this panel, so need to find
                // out which part to include
                // triangle may be split by multiple planes in the polyhedron
                
                // split the triangle
                std::vector<SplitTri<FP>> tmp_stris = split(tri_elem, hex);

                // for each new triangle test if it is within the polyhedron
                for (unsigned int s=0; s < tmp_stris.size(); ++s) {
                    // to avoid possible numerical errors, just check the center
                    // of the triangle
                    cv::Point3_<FP> center = (tmp_stris[s].nodes[0] + 
                                              tmp_stris[s].nodes[1] + 
                                              tmp_stris[s].nodes[2]) / 3.0;
                    if (contains(hex, center)) {
                        stris.push_back(tmp_stris[s]);
                    }
                }
            }

            for (unsigned int s=0; s < stris.size(); ++s) {
                FP sub_area = area(static_cast<Triangle<FP>&>(stris[s]));
                total_area += sub_area;
                cover_area += sub_area;
                if (nodal) {
                    // get the weight of each original face node 
                    // (based on mapping to new nodes)
                    Eigen::Matrix<FP, 1, 3> n_weights = stris[s].mapping.colwise().sum();

                    //std::cout << "Weights = " << n_weights << std::endl;
                    //std::cout << "Nodes: " << simp[0] << ", " << simp[1];
                    //std::cout << ", " << simp[2] << std::endl;

                    // split weight equally among all nodes
                    // if coverage is missing for any node, just skip that piece
                    for (unsigned int i=0; i < 3; ++i) {
                        if (coverage[simp[i]]) {
                            trip_fx.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*normal.x)); 
                            trip_fy.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*normal.y)); 
                            trip_fz.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*normal.z));
                            // moment about the centroid (grid units)
                            cv::Point3_<FP> r = tri.get_position(simp[i]) - p_center;
                            cv::Point3_<FP> mom = r.cross(normal);
                            if (std::isnan(mom.x) || std::isnan(mom.y) 
                                || std::isnan(mom.z)) {
                                continue;
                            }
                            trip_mx.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*mom.x));
                            trip_my.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*mom.y));
                            trip_mz.push_back(Trip(p_count, simp[i], 
                                    n_weights(i)*1.0/3.0*sub_area*mom.z));
                        } else {
                            // no coverage,
                            // so just adjust the coverage area, not total
                            //cover_area -= n_weights(i)*1.0/3.0*sub_area;
                            cover_area -= n_weights(i)*1.0/3.0*sub_area;
                        }
                    }
                } else {
                    if (coverage[fidx]) {
                        trip_fx.push_back(Trip(p_count, fidx, sub_area*normal.x));
                        trip_fy.push_back(Trip(p_count, fidx, sub_area*normal.y));
                        trip_fz.push_back(Trip(p_count, fidx, sub_area*normal.z));
                        // moment about the centroid (grid units)
                        cv::Point3_<FP> r = tri.get_position(fidx) - p_center;
                        cv::Point3_<FP> mom = r.cross(normal);
                        if (std::isnan(mom.x) || std::isnan(mom.y) 
                            || std::isnan(mom.z)) {
                            continue;
                        }
                        trip_mx.push_back(Trip(p_count, fidx, sub_area*mom.x));
                        trip_my.push_back(Trip(p_count, fidx, sub_area*mom.y));
                        trip_mz.push_back(Trip(p_count, fidx, sub_area*mom.z));
                    } else {
                        cover_area -= sub_area;
                    }
                }
            }
        }

        // Update the coverage
        //assert(total_area > 0.0);
        if (total_area <= 0) {
            p_integ.coverage[p_count] = 1.0;
            p_integ.area[p_count] = 0.0;
        } else {
            p_integ.coverage[p_count] = cover_area / total_area;
            p_integ.area[p_count] = total_area;
        }

        ++p_count;
    } 

    // Build the sparse matrices from the triplets
    p_integ.fx.setFromTriplets(trip_fx.begin(), trip_fx.end());
    p_integ.fy.setFromTriplets(trip_fy.begin(), trip_fy.end());
    p_integ.fz.setFromTriplets(trip_fz.begin(), trip_fz.end());
    p_integ.mx.setFromTriplets(trip_mx.begin(), trip_mx.end());
    p_integ.my.setFromTriplets(trip_my.begin(), trip_my.end());
    p_integ.mz.setFromTriplets(trip_mz.begin(), trip_mz.end());

    return p_integ;
}

/*****************************************************************************/
template<typename FP>
FoMo<FP> integrate_panels(const PanelIntegration<FP>& pinteg, 
                                 const std::vector<FP>& values) {

    assert(pinteg.is_valid());

    FoMo<FP> fomo;

    if (pinteg.transpose) {
        // Operation:
        //  fx    * value = fx_out
        //  (PxN) * (NxF) = (PxF)
        // F is some number of frames

        assert(values.size() % pinteg.fx.cols() == 0);

        // Pull params
        unsigned int n_data_pts = pinteg.fx.cols();
        unsigned int n_panels   = pinteg.fx.rows();
        unsigned int n_frames   = values.size() / n_data_pts;

        // Initialize output data
        fomo.fx.assign(n_panels*n_frames, 0);
        fomo.fy.assign(n_panels*n_frames, 0);
        fomo.fz.assign(n_panels*n_frames, 0);
        fomo.mx.assign(n_panels*n_frames, 0);
        fomo.my.assign(n_panels*n_frames, 0);
        fomo.mz.assign(n_panels*n_frames, 0);
        
        // wrap the vectors with Eigen Maps
        typedef Eigen::Map<const Eigen::Matrix<FP,Eigen::Dynamic,Eigen::Dynamic,
                Eigen::RowMajor>> V_Map;
        typedef Eigen::Map<Eigen::Matrix<FP,Eigen::Dynamic,Eigen::Dynamic,
                Eigen::RowMajor>> M_Map;

        V_Map v_in(&values[0], n_data_pts, n_frames);
        
        M_Map fx_out(&fomo.fx[0], n_panels, n_frames);
        M_Map fy_out(&fomo.fy[0], n_panels, n_frames);
        M_Map fz_out(&fomo.fz[0], n_panels, n_frames);
        M_Map mx_out(&fomo.mx[0], n_panels, n_frames);
        M_Map my_out(&fomo.my[0], n_panels, n_frames);
        M_Map mz_out(&fomo.mz[0], n_panels, n_frames);

        // Perform matrix-matrix products
        fx_out = pinteg.fx * v_in;
        fy_out = pinteg.fy * v_in;
        fz_out = pinteg.fz * v_in;
        mx_out = pinteg.mx * v_in;
        my_out = pinteg.my * v_in;
        mz_out = pinteg.mz * v_in;

    } else {
        // Operation:
        //  values * fx    = fx_out
        //  (FxN)  * (NxP) = (FxP)
        // F is some number of frames

        assert(values.size() % pinteg.fx.rows() == 0);

        // Pull params
        unsigned int n_data_pts = pinteg.fx.rows();
        unsigned int n_panels   = pinteg.fx.cols();
        unsigned int n_frames   = values.size() / n_data_pts;

        // Initialize output data
        fomo.fx.assign(n_panels*n_frames, 0);
        fomo.fy.assign(n_panels*n_frames, 0);
        fomo.fz.assign(n_panels*n_frames, 0);
        fomo.mx.assign(n_panels*n_frames, 0);
        fomo.my.assign(n_panels*n_frames, 0);
        fomo.mz.assign(n_panels*n_frames, 0);

        // wrap the vectors with Eigen Maps
        typedef Eigen::Map<const Eigen::Matrix<FP,Eigen::Dynamic,Eigen::Dynamic,
                Eigen::RowMajor>> V_Map;
        typedef Eigen::Map<Eigen::Matrix<FP,Eigen::Dynamic,Eigen::Dynamic,
                Eigen::RowMajor>> M_Map;

        V_Map v_in(&values[0], n_frames, n_data_pts);
        
        M_Map fx_out(&fomo.fx[0], n_frames, n_panels);
        M_Map fy_out(&fomo.fy[0], n_frames, n_panels);
        M_Map fz_out(&fomo.fz[0], n_frames, n_panels);
        M_Map mx_out(&fomo.mx[0], n_frames, n_panels);
        M_Map my_out(&fomo.my[0], n_frames, n_panels);
        M_Map mz_out(&fomo.mz[0], n_frames, n_panels);

        // Perform matrix-matrix products
        fx_out = v_in * pinteg.fx;
        fy_out = v_in * pinteg.fy;
        fz_out = v_in * pinteg.fz;
        mx_out = v_in * pinteg.mx;
        my_out = v_in * pinteg.my;
        mz_out = v_in * pinteg.mz;

    }

    return fomo;
}

/*****************************************************************************/
template<typename FP>
Polyhedron<6,FP> build_polyhedron(const typename P3DModel_<FP>::Face& f, 
        FP height_sf/*=1.0*/) {

    typedef typename P3DModel_<FP>::Node Node;
    typedef typename P3DModel_<FP>::Face Face;
    typedef typename P3DModel_<FP>::face_idx face_idx;

    // Initialize the hexahedron
    Polyhedron<6,FP> poly;

    // Get the face index
    face_idx fidx = f.get_fidx();
    
    // Get the normal direction for f
    cv::Point3_<FP> normal = f.get_normal();

    // For each edge, add the plane to the polyhedron
    // and compute maximum edge length
    float min_edge = std::numeric_limits<FP>::infinity();
    std::array<GridDir,4> dirs({GridDir::J_plus, GridDir::K_plus, GridDir::J_minus, 
                               GridDir::K_minus});
    std::array<Node,2> edge;
    for (unsigned int i=0; i < 4; ++i) {
        GridDir g_dir = dirs[i];

        // Get the edge nodes
        edge = f.get_edge_nodes(g_dir);

        // Get the adjacent face
        Face f_adj = f.get_adjacent(g_dir);

        // Get the direction of the polyhedron's face on this edge
        // if there is an adjacent face, then average the normals
        // else just use the normal for f
        cv::Point3_<FP> full_normal = normal;
        if (f_adj.is_valid()) {
            cv::Point3_<FP> n2 = f_adj.get_normal();
            full_normal = 0.5*(full_normal + n2);
        }

        // Cross the surface normal with the edge direction
        // note that for each edge, edge[1].j > edge[0].j or edge[1].k > edge[0].k
        // want to go in a circle
        cv::Point3_<FP> edge_dir;
        switch (g_dir) {
            case GridDir::J_plus : 
                edge_dir = edge[1].get_position() - edge[0].get_position();
                break;
            case GridDir::K_plus : 
                edge_dir = edge[0].get_position() - edge[1].get_position();
                break;
            case GridDir::J_minus : 
                edge_dir = edge[0].get_position() - edge[1].get_position();
                break;
            case GridDir::K_minus : 
                edge_dir = edge[1].get_position() - edge[0].get_position();
                break;
        }

        cv::Point3_<FP> new_normal = edge_dir.cross(full_normal);

        // Define the plane
        poly.planes[i] = Plane<FP>(new_normal, edge[0].get_position());

        // Get the new maximum edge length
        FP edge_len = cv::norm(edge_dir);
        if (edge_len < min_edge) {
            min_edge = edge_len;
        }

        // add edge axis, could possibly reduce the number of axes
        // by checking these edges for duplication
        poly.edges.push_back(edge_dir);
    }

    // Add planes for the top and bottom
    // parallel to f, with full height defined by height*max_edge
    poly.planes[4] = Plane<FP>(     normal, edge[0].get_position() + 
                                            height_sf*min_edge*0.5*normal);
    poly.planes[5] = Plane<FP>(-1.0*normal, edge[0].get_position() - 
                                            height_sf*min_edge*0.5*normal);

    // Determine the vertices of the hexadron
    for (unsigned int i=0; i < 4; ++i) {
        Plane<FP>& next_pl = poly.planes[(i+1) % 4];
        assert(intersects(poly.planes[i], next_pl, poly.planes[4]));
        assert(intersects(poly.planes[i], next_pl, poly.planes[5]));
        
        poly.vertices.push_back(intersect(poly.planes[i], next_pl, poly.planes[4]));
        poly.vertices.push_back(intersect(poly.planes[i], next_pl, poly.planes[5]));

        // add vertical edge axis
        cv::Point3_<FP> axis_dir = poly.vertices[2*i+1] - poly.vertices[2*i];
        poly.edges.push_back(axis_dir);

    }

    return poly;
}

/*****************************************************************************/
template<typename FP>
std::vector<Polyhedron<6,FP>> build_polyhedrons(
        const StructuredGrid<FP>& grid) {

    std::vector<Polyhedron<6,FP>> hexes;

    for (unsigned int z=0; z < grid.num_zones(); ++z) {

        unsigned int k_len = grid.zone_size(z,1);
        unsigned int j_len = grid.zone_size(z,0);

        unsigned int z_sz = k_len * j_len;
        unsigned int z_start = grid.zone_start_idx(z);

        for (unsigned int k=0; k < k_len-1; ++k) {
            for (unsigned int j=0; j < j_len-1; ++j) {
                Polyhedron<6,FP> hex;

                /*******************************/
                /* Define all vertices         */
                /*******************************/
                for (unsigned int l2=0; l2 < 2; ++l2) {
                for (unsigned int k2=0; k2 < 2; ++k2) {
                for (unsigned int j2=0; j2 < 2; ++j2) {
                    unsigned int idx = z_start + l2*z_sz + (k+k2)*j_len + j+j2;
                    hex.vertices.push_back({grid.x[idx],grid.y[idx],grid.z[idx]});
                }}}

                /*******************************/
                /* Define all edges            */
                /*******************************/
                
                // edges in j
                hex.edges.push_back(hex.vertices[1] - hex.vertices[0]); // 0
                hex.edges.push_back(hex.vertices[3] - hex.vertices[2]); // 1
                hex.edges.push_back(hex.vertices[5] - hex.vertices[4]); // 2
                hex.edges.push_back(hex.vertices[7] - hex.vertices[6]); // 3
                
                // edges in k
                hex.edges.push_back(hex.vertices[2] - hex.vertices[0]); // 4
                hex.edges.push_back(hex.vertices[3] - hex.vertices[1]); // 5
                hex.edges.push_back(hex.vertices[6] - hex.vertices[4]); // 6
                hex.edges.push_back(hex.vertices[7] - hex.vertices[5]); // 7

                // edges in l
                hex.edges.push_back(hex.vertices[4] - hex.vertices[0]); // 8
                hex.edges.push_back(hex.vertices[5] - hex.vertices[1]); // 9
                hex.edges.push_back(hex.vertices[6] - hex.vertices[2]); // 10
                hex.edges.push_back(hex.vertices[7] - hex.vertices[3]); // 11

                /*******************************/
                /* Define all planes           */
                /*******************************/
                cv::Point3_<FP> normal;

                normal = hex.edges[0].cross(hex.edges[8]);
                hex.planes[0] = {normal, hex.vertices[0]};

                normal = hex.edges[5].cross(hex.edges[9]);
                hex.planes[1] = {normal, hex.vertices[1]};

                normal = hex.edges[3].cross(-hex.edges[10]);
                hex.planes[2] = {normal, hex.vertices[3]};

                normal = hex.edges[6].cross(-hex.edges[8]);
                hex.planes[3] = {normal, hex.vertices[2]};

                normal = hex.edges[2].cross(hex.edges[6]);
                hex.planes[4] = {normal, hex.vertices[4]};

                normal = hex.edges[4].cross(hex.edges[0]);
                hex.planes[5] = {normal, hex.vertices[0]};

                hexes.push_back(hex);
            }
        }
    }

    return hexes;
}

/*****************************************************************************/
template<typename FP>
void split_model(TriModel_<FP>& tri, const Panels<FP>& panels, 
        bool break_conn/*=false*/) {

    typedef Polyhedron<6,FP> Hexa;
    typedef TriModel_<FP> Model;
    typedef typename TriModel_<FP>::node_idx node_idx;
    typedef typename TriModel_<FP>::face_idx face_idx;
    typedef typename TriModel_<FP>::simple_tri simple_tri;
    typedef typename TriModel_<FP>::Face Face;
    typedef typename TriModel_<FP>::Node Node;
    typedef typename TriModel_<FP>::FaceIterator FaceIter;

    // Build an Octree for the TriModel over all triangle
    BoundingBox_<cv::Point3_<FP>> bb = get_extent(
            boost::make_transform_iterator(tri.node_begin(), Node2_Point<Model>()),
            boost::make_transform_iterator(tri.node_end(), Node2_Point<Model>()));
    Octree<TriModel_<FP>,Face> tree(&tri,
            tri.cface_begin(), tri.cface_end(),
            bb, 30);

    // Initialize TriModel components (to invalid face_idx -1)
    tri.initialize_components(-1);

    FP contains_tol = 0.0001;

    // Store a mapping between the original and the current split faces
    FaceMap<face_idx> parent_faces;

    // for each panel, find all triangles that are contained within that panel
    // and split any that lie across panel edges
    for (unsigned int i=0; i < panels.size(); ++i) {
        //std::cout << "Panel " << i << std::endl;

        // Create a polyhedron that bounds all nodes/triangles
        // that could be a part of this panel
        Hexa hex = panels.hexes[i];

        //if (i == 11) {
        //    break;
       // }

        // Find all of the triangles that may be intersected
        std::set<face_idx> all_faces = approx_faces_within(hex, tree);

        // Update with split faces (not updating tree during splitting)
        //std::cout << "Updating split faces" << std::endl;
        std::vector<face_idx> add_faces;
        std::vector<face_idx> remove_faces;
        for (auto af_it = all_faces.begin(); af_it != all_faces.end(); ++af_it) {
            face_idx curr_face = *af_it;

            if (parent_faces.has(curr_face)) {
                remove_faces.push_back(curr_face);

                std::queue<face_idx> check_faces;
                check_faces.push(curr_face);
                while (!check_faces.empty()) {
                    face_idx c_f = check_faces.front();
                    //std::cout << "check face " << c_f << std::endl;
                    check_faces.pop();
                    if (parent_faces.has(c_f)) {
                        const std::set<face_idx>& childs = parent_faces.children(c_f);
                        for (auto c_it = childs.begin(); c_it != childs.end(); ++c_it) {
                            assert(c_f < *c_it);
                            check_faces.push(*c_it);
                        }
                    } else {
                        add_faces.push_back(c_f);
                    }
                }
            }
        }
        for (unsigned int i=0; i < remove_faces.size(); ++i) {
            all_faces.erase(remove_faces[i]);
        }
        all_faces.insert(add_faces.begin(), add_faces.end());

        //std::cout << "Found faces that might be in panel " << i << std::endl;
        //for (auto it=all_faces.begin(); it != all_faces.end(); ++it) {
        //    std::cout << "  " << (*it);
        //}
        //std::cout << std::endl;

        // Get a list of all triangles that have been split
        // due to tolerance issues, may see repeated splitting of the same
        // face, want to stop this
        std::set<face_idx> already_split;
        
        // all the faces to check for splitting, or containment, may
        // grow as some of the original faces are split
        std::vector<face_idx> faces(all_faces.begin(), all_faces.end());

        for (unsigned int f_i = 0; f_i < faces.size(); ++f_i) {
            face_idx curr_tri = faces[f_i];

            //std::cout << "Testing triangle " << curr_tri << std::endl;

            // skip any faces that are non-data
            if (!tri.is_dataface(curr_tri)) {
                continue;
            }

            assert(already_split.count(curr_tri) == 0);

            // face could have been cut by a previous step, if so just skip
            if (!tri.is_face(curr_tri)) {
                assert(all_faces.count(curr_tri) == 0);
                continue;
            }

            // first just check if the triangle is already within the panel
            Face f = tri.face(curr_tri);
            if (contains(hex, f.get_triangle(), contains_tol)) {
                tri.set_component(curr_tri, i);
                already_split.insert(curr_tri);
                continue;
            }

            // Perform the split
            already_split.insert(curr_tri);
            FaceMap<face_idx> tmp_faces = tri.split(curr_tri, hex, !break_conn);

            // if found no splits, triangle is fully outside of panel
            if (tmp_faces.size() == 0) {

                // It may just be a tolerance issue, check if center of 
                // triangle is within hex
                 cv::Point3_<FP> center = f.get_center();
                 if (contains(hex, center)) {
                     tri.set_component(curr_tri, i);
                 }
                continue;
            }

            // update the overall mapping
            parent_faces.merge(tmp_faces);

            // splitting occured, so handle it
            for (auto f_it = tmp_faces.begin(); f_it != tmp_faces.end(); ++f_it) {
                face_idx parent = *f_it;

                // if this parent is part of the triangles that need to 
                // be checked, then add the new children
                if (all_faces.count(parent) == 0) {
                    continue;
                }

                all_faces.erase(parent);
                const std::set<face_idx>& childs = tmp_faces.children(parent);
                all_faces.insert(childs.begin(), childs.end());
                if (parent != curr_tri) {
                    if (already_split.count(parent) == 0) {
                        //std::cout << "Adding to faces: ";
                        //for (auto ot_c = childs.begin(); ot_c != childs.end(); ++ot_c) {
                        //    std::cout << (*ot_c) << " ";
                        //}
                        //std::cout << std::endl;
                        std::copy(childs.cbegin(), childs.cend(),
                                std::back_inserter(faces));
                    }
                    continue;
                }
              
                already_split.insert(childs.cbegin(), childs.cend());
                // check if any children belong to this panel
                for (auto c_it = childs.cbegin(); c_it != childs.cend(); ++c_it) {
                    Face c_f = tri.face(*c_it);
                    if (contains(hex, c_f.get_triangle(), contains_tol) &&
                            c_f.is_dataface()) {
                        tri.set_component(*c_it, i);
                    } else {
                        // already split, so could be some tolerance issues
                        // if the center of the triangle is inside, then
                        // it is inside
                        // could probably just do this test instead
                        cv::Point3_<FP> center = c_f.get_center();
                        if (contains(hex, center)) {
                            tri.set_component(*c_it, i);
                        }
                    }
                }

            }
        }
    }
}

/*****************************************************************************/
template<typename FP>
PanelIntegration<FP> transpose(const PanelIntegration<FP>& pinteg) {

    // copy over the unchanged vectors
    PanelIntegration<FP> out_pinteg;
    out_pinteg.center_x = std::vector<FP>(pinteg.center_x);
    out_pinteg.center_y = std::vector<FP>(pinteg.center_y);
    out_pinteg.center_z = std::vector<FP>(pinteg.center_z);
    out_pinteg.area = std::vector<FP>(pinteg.area);
    out_pinteg.coverage = std::vector<FP>(pinteg.coverage);

    // transpose the matrices
    out_pinteg.fx = pinteg.fx.transpose();
    out_pinteg.fy = pinteg.fy.transpose();
    out_pinteg.fz = pinteg.fz.transpose();
    out_pinteg.mx = pinteg.mx.transpose();
    out_pinteg.my = pinteg.my.transpose();
    out_pinteg.mz = pinteg.mz.transpose();

    assert(pinteg.fx.nonZeros() == out_pinteg.fx.nonZeros());
    
    out_pinteg.transpose = !pinteg.transpose;

    return out_pinteg;
}


} /* end namespace upsp */
