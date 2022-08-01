/** @file
 *  @brief  Test the TriModel class
 *  @date   March 11, 2019
 *  @author jmpowel2
 */
#include <algorithm>
#include <set>
#include <string>

#include "gtest/gtest.h"

#include "../include/TriModel.h"
#include "../include/P3DModel.h"

#include "test_grid_utils.h"

/** TODO
 * Node : is_contained
 * Face : is_contained
 */

/** Test that TriModel is able to read in an unformatted,
 *  single zone, intersected, *.i.tri file
 */
TEST(TriModelTest, LoadsUnfSingleITri) {
    const std::string input_file = "./inputs/sphere_unf_single.i.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_EQ(model.size(), 514);
    EXPECT_EQ(model.number_of_faces(), 1024);
    EXPECT_TRUE(model.has_components());
    EXPECT_EQ(model.number_of_components(), 1);
    EXPECT_TRUE(model.is_valid());

    // Perform intersection, should be no difference
    upsp::TriModel_<float> model2(input_file, true);
    EXPECT_EQ(model2.size(), 514);
    EXPECT_EQ(model2.number_of_faces(), 1024);
    EXPECT_TRUE(model2.has_components());
    EXPECT_EQ(model2.number_of_components(), 1);
    EXPECT_TRUE(model2.is_valid());
}

/** Test that TriModel is able to write out an unformatted
 * single zone, intersected, *.i.tri file
 */
TEST(TriModelTest, WritesUnfSingleITri) {
    const std::string input_file = "./inputs/sphere_unf_single.i.tri";
    const std::string output_file = "./outputs/out_sphere_unf_single.i.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_TRUE(model.is_valid());
    model.write_grid(output_file);

    upsp::TriModel_<float> model2(input_file, false);
    EXPECT_TRUE(model2.is_valid());
    EXPECT_TRUE(model == model2);
}

/** Test that TriModel is able to read in an unformatted,
 *  single zone, *.tri file
 */
TEST(TriModelTest, LoadsUnfSingleTri) {
    const std::string input_file = "./inputs/sphere_unf_single.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_EQ(model.size(), 594);
    EXPECT_EQ(model.number_of_faces(), 1024);
    EXPECT_TRUE(model.has_components());
    EXPECT_EQ(model.number_of_components(), 1);
    EXPECT_TRUE(model.is_valid());
    
    // Perform intersection
    upsp::TriModel_<float> model2(input_file, true);
    EXPECT_EQ(model2.size(), 514);
    EXPECT_EQ(model2.number_of_faces(), 1024);
    EXPECT_TRUE(model2.has_components());
    EXPECT_EQ(model2.number_of_components(), 1);
    EXPECT_TRUE(model2.is_valid());
}

/** Test that TriModel is able to read in an unformatted,
 *  multiple zone, intersected, *.i.tri file
 */
TEST(TriModelTest, LoadsUnfMultiITri) {
    const std::string input_file = "./inputs/sphere_unf_multi.i.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_EQ(model.size(), 514);
    EXPECT_EQ(model.number_of_faces(), 1024);
    EXPECT_TRUE(model.has_components());
    EXPECT_EQ(model.number_of_components(), 6);
    EXPECT_TRUE(model.is_valid());

    // Perform intersection, should be no difference
    upsp::TriModel_<float> model2(input_file, true);
    EXPECT_EQ(model2.size(), 514);
    EXPECT_EQ(model2.number_of_faces(), 1024);
    EXPECT_TRUE(model2.has_components());
    EXPECT_EQ(model2.number_of_components(), 6);
    EXPECT_TRUE(model2.is_valid());
}

/** Test that TriModel is able to write out an unformatted
 * multi zone, intersected, *.i.tri file
 */
TEST(TriModelTest, WritesUnfMultiITri) {
    const std::string input_file = "./inputs/sphere_unf_multi.i.tri";
    const std::string output_file = "./outputs/out_sphere_unf_multi.i.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_TRUE(model.is_valid());
    model.write_grid(output_file);

    upsp::TriModel_<float> model2(input_file, false);
    EXPECT_TRUE(model2.is_valid());
    EXPECT_TRUE(model == model2);
}

/** Test that TriModel is able to read in an unformatted,
 *  multiple zone, unintersected, *.tri file
 */
TEST(TriModelTest, LoadsUnfMultiTri) {
    const std::string input_file = "./inputs/sphere_unf_multi.tri";

    upsp::TriModel_<float> model(input_file, false);
    EXPECT_EQ(model.size(), 594);
    EXPECT_EQ(model.number_of_faces(), 1024);
    EXPECT_TRUE(model.has_components());
    EXPECT_EQ(model.number_of_components(), 6);
    EXPECT_TRUE(model.is_valid());

    // Perform intersection
    upsp::TriModel_<float> model2(input_file, true);
    EXPECT_EQ(model2.size(), 514);
    EXPECT_EQ(model2.number_of_faces(), 1024);
    EXPECT_TRUE(model2.has_components());
    EXPECT_EQ(model2.number_of_components(), 6);
    EXPECT_TRUE(model2.is_valid());
}

/** Test initialization from P3DModel */
TEST(TriModelTest, InitializeFromP3dSingle) {
    const std::string input_file = "./inputs/sphere_unf_single_integration_sp.x";

    upsp::P3DModel_<float> p3d(input_file);

    // Copy without intersection
    upsp::TriModel_<float> tri(p3d, false);

    EXPECT_EQ(p3d.size(), tri.size());
    EXPECT_EQ(p3d.num_zones(), tri.number_of_components());
    EXPECT_EQ(p3d.number_of_faces()*2, tri.number_of_faces());

    const auto vector_expect_float_eq = [](
            const std::vector<float>& a, const std::vector<float>& b) {
        EXPECT_EQ(a.size(), b.size());
        for (int ii = 0; ii < a.size(); ii++) EXPECT_FLOAT_EQ(a[ii], b[ii]);
    };

    vector_expect_float_eq(p3d.get_x(), tri.get_x());
    vector_expect_float_eq(p3d.get_y(), tri.get_y());
    vector_expect_float_eq(p3d.get_z(), tri.get_z());

    // Copy with intersection
    upsp::TriModel_<float> tri2(p3d, true);
    
    EXPECT_GE(p3d.size(), tri2.size()); // >=
    EXPECT_EQ(p3d.num_zones(), tri2.number_of_components());
    EXPECT_EQ(p3d.number_of_faces()*2, tri2.number_of_faces());
}

/** Test initialization from P3DModel */
TEST(TriModelTest, InitializeFromP3dMulti) {
    const std::string input_file = "./inputs/sphere_unf_multi_integration_sp.x";

    upsp::P3DModel_<float> p3d(input_file);

    // Copy without intersection
    upsp::TriModel_<float> tri(p3d, false);

    EXPECT_EQ(p3d.size(), tri.size());
    EXPECT_EQ(p3d.num_zones(), tri.number_of_components());
    EXPECT_EQ(p3d.number_of_faces()*2, tri.number_of_faces());

    // cannot use p3d node iterator directly here, because it automatically
    // skips over overlapping points
    bool matching_nodes = true;
    for (unsigned int i=0; i < tri.size(); ++i) {
        if (p3d.get_position(i) != tri.get_position(i)) {
            matching_nodes = false;
            break;
        }
    }
    EXPECT_TRUE(matching_nodes);

    // Copy with intersection
    upsp::TriModel_<float> tri2(p3d, true);
    
    EXPECT_GE(p3d.size(), tri2.size()); // >=
    EXPECT_EQ(p3d.num_zones(), tri2.number_of_components());
    EXPECT_EQ(p3d.number_of_faces()*2, tri2.number_of_faces());
}

/** Test NodeIterator */
TEST_F(TriModelTestClass, NodeIterator) {

    unsigned int count_sz=0;
    for (auto it = model.node_begin(); it != model.node_end(); ++it) {
        EXPECT_EQ( *(it) == model.node(count_sz), true);
        ++count_sz;
    }
    EXPECT_EQ(count_sz, model.size());

}

/** Test that SimpleTriModelTestClass is properly 
 *  initialized
 */
TEST_F(SimpleTriModelTestClass, Initialization) {
    EXPECT_TRUE(model.is_valid());

    EXPECT_EQ(model.size(), 9);
    EXPECT_EQ(model.number_of_faces(), 4);
}

/** Test Node */
TEST_F(SimpleTriModelTestClass, NodeClass) {

    Node n = model.node(1);
    EXPECT_EQ(n.get_nidx(), 1);

    // test number of adjacent faces
    EXPECT_EQ(n.number_of_adj_faces(), 3);

    // test position
    EXPECT_EQ(n.get_position(), cv::Point3f(-1,-1,0));

    // test normal
    cv::Point3f normal = n.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0);
    EXPECT_FLOAT_EQ(normal.y, 0.0);
    EXPECT_FLOAT_EQ(normal.z, 1.0);
}

/** Test Face */
TEST_F(SimpleTriModelTestClass, FaceClass) {

    Face f = model.face(0);
    EXPECT_EQ(f.get_fidx(), 0);
    
    // test simple triangle retrieval
    simple_tri s_tri = f.nodes();
    simple_tri s_ref({0,5,1});
    EXPECT_EQ(s_tri, s_ref);

    // test simple edge retrieval
    std::array<simple_edge,3> s_edge_ref;
    s_edge_ref[0] = simple_edge({0,5});
    s_edge_ref[1] = simple_edge({5,1});
    s_edge_ref[2] = simple_edge({1,0});
    std::array<simple_edge,3> s_edges = f.simple_edges();
    bool found_edges = true;
    for (unsigned int i=0; i < 3; ++i) {
        bool found_edgei = false;
        for (unsigned int j=0; j < 3; ++j) {
            if (s_edges[i] == s_edge_ref[j]) {
                found_edgei = true;
                break;
            }
        }
        found_edges &= found_edgei;
    }
    EXPECT_TRUE(found_edges);

    // test number of adjacent faces
    EXPECT_EQ(f.number_of_adj_faces(), 1);

    // test normal direction
    cv::Point3f normal = f.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0);
    EXPECT_FLOAT_EQ(normal.y, 0.0);
    EXPECT_FLOAT_EQ(normal.z, 1.0);

    // test area
    EXPECT_FLOAT_EQ(f.get_area(), 1.0);
}

/** Test Face iterators */
TEST_F(SimpleTriModelTestClass, FaceIterators) {

    model.initialize_components(0);
    model.set_component(1,1);
    model.set_component(2,1);
    model.remove_face(1);

    unsigned int count = 0;
    for (auto it = model.face_begin(); it != model.face_end(); ++it) {
        Face f = *it;
        EXPECT_TRUE(f.is_valid());
        ++count;
    }
    EXPECT_EQ(count, 3);

    count = 0;
    for (auto it = model.comp_face_begin(0); it != model.comp_face_end(0); ++it) {
        Face f = *it;
        EXPECT_TRUE(f.is_valid());
        ++count;
    }
    EXPECT_EQ(count, 2);

}

/** Test ability to assign and change components */
TEST_F(SimpleTriModelTestClass, AssignComponents) {

    model.initialize_components(0);
    EXPECT_TRUE(model.is_valid());
    EXPECT_EQ(model.number_of_components(), 1);

    model.set_component(1, 2);
    EXPECT_TRUE(model.is_valid());
    EXPECT_EQ(model.number_of_components(), 2);

    model.set_component(3, 2);
    EXPECT_TRUE(model.is_valid());
    EXPECT_EQ(model.number_of_components(), 2);

    // test that intialized component 0, is erased
    // once no faces use it
    model.set_component(0, 1);
    model.set_component(2, 1);
    EXPECT_EQ(model.number_of_components(), 2);
}

/** Test adjacent to node operations */
TEST_F(SimpleTriModelTestClass, AdjacentToNode) {

    /** Find all nodes adjacent to Node 2
     *  should be 0,1,3
     */
    std::vector<node_idx> adj;
    for (auto it=model.adj_node_begin(2); it != model.adj_node_end(2); ++it) {
        auto n = *it;
        adj.push_back(n.get_nidx());
    }
    std::sort(adj.begin(), adj.end());

    EXPECT_EQ(adj.size(), 3);
    EXPECT_EQ(adj[0], 0);
    EXPECT_EQ(adj[1], 1);
    EXPECT_EQ(adj[2], 3);

    /** Find all nodes adjacent to Node 4
     *  should be none
     */
    adj.clear();
    for (auto it=model.adj_node_begin(4); it != model.adj_node_end(4); ++it) {
        auto n = *it;
        adj.push_back(n.get_nidx());
    }

    EXPECT_EQ(adj.size(), 0);

    /** Find all faces adjacent to Node 2
     *  should be (0,1,2) = face 1 and (2,1,3) = face 2
     */
    std::vector<face_idx> adj_face;
    for (auto it=model.nadj_face_begin(2); it != model.nadj_face_end(2); ++it) {
        auto f = *it;
        adj_face.push_back(f.get_fidx());
    }
    std::sort(adj_face.begin(), adj_face.end());

    EXPECT_EQ(adj_face.size(), 2);
    EXPECT_EQ(adj_face[0], 1);
    EXPECT_EQ(adj_face[1], 2);

    /** Find all faces adjacent to Node 4
     *  should be none
     */
    adj_face.clear();
    for (auto it=model.nadj_face_begin(4); it != model.nadj_face_end(4); ++it) {
        auto f = *it;
        adj_face.push_back(f.get_fidx());
    }

    EXPECT_EQ(adj_face.size(), 0);

}

/** Test adjacent to face operations */
TEST_F(SimpleTriModelTestClass, AdjacentToFace) {

    EXPECT_EQ(model.number_of_adj_faces(0), 1);
    EXPECT_EQ(model.number_of_adj_faces(1), 2);
    EXPECT_EQ(model.number_of_adj_faces(2), 1);
    EXPECT_EQ(model.number_of_adj_faces(3), 0);

    /** Find all faces adjacent to Face 0
     *  should be face 1
     */
    std::vector<face_idx> adj;
    for (auto it=model.adj_face_begin(0); it != model.adj_face_end(0); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }
    
    EXPECT_EQ(adj.size(), 1);
    EXPECT_EQ(adj[0], 1);

    /** Find all faces adjacent to Face 1
     *  should face 0 and face 2
     *  testing const iterator
     */
    adj.clear();
    for (auto it=model.cadj_face_begin(1); it != model.cadj_face_end(1); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }
    std::sort(adj.begin(), adj.end());

    EXPECT_EQ(adj.size(), 2);
    EXPECT_EQ(adj[0], 0);
    EXPECT_EQ(adj[1], 2);

    /** Find all faces adjacent to Face 3
     *  should be none
     */
    adj.clear();
    for (auto it=model.adj_face_begin(3); it != model.adj_face_end(3); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }

    EXPECT_EQ(adj.size(), 0);

}

/** Test Face Adjacency within Components */
TEST_F(SimpleTriModelTestClass, CompAdjacentToFace) {

    model.initialize_components(1);
    model.set_component(0, 2);

    EXPECT_EQ(model.number_of_adj_faces_comp(0), 0);
    EXPECT_EQ(model.number_of_adj_faces_comp(1), 1);
    EXPECT_EQ(model.number_of_adj_faces_comp(2), 1);
    EXPECT_EQ(model.number_of_adj_faces_comp(3), 0);

    /** Find all faces adjacent to Face 0, within comp
     *  should be none
     */
    std::vector<face_idx> adj;
    for (auto it=model.compadj_face_begin(0); it != model.compadj_face_end(0); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }
    
    EXPECT_EQ(adj.size(), 0);
    
    /** Find all faces adjacent to Face 1, within comp
     *  should be face 2
     *  testing const iterator
     */
    adj.clear();
    for (auto it=model.ccompadj_face_begin(1); it != model.ccompadj_face_end(1); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }
    
    EXPECT_EQ(adj.size(), 1);
    EXPECT_EQ(adj[0], 2);

    /** Find all faces adjacent to Face 3, within comp
     *  should be none
     */
    adj.clear();
    for (auto it=model.compadj_face_begin(3); it != model.compadj_face_end(3); ++it) {
        auto f = *it;
        adj.push_back(f.get_fidx());
    }
    
    EXPECT_EQ(adj.size(), 0);

}

/** Test Ability to Remove Triangles */
TEST_F(SimpleTriModelTestClass, RemoveFace) {

    model.remove_face(1);

    EXPECT_TRUE(model.is_valid());
    EXPECT_EQ(model.number_of_nodes(), 9);
    EXPECT_EQ(model.number_of_faces(), 3);

    EXPECT_EQ(model.number_of_adj_faces(0), 0);
    EXPECT_EQ(model.number_of_adj_faces(2), 0);
    EXPECT_EQ(model.number_of_adj_faces(3), 0);

    /** Find all nodes adjacent to Node 2
     *  should be 1,3
     */
    std::vector<node_idx> adj;
    for (auto it=model.adj_node_begin(2); it != model.adj_node_end(2); ++it) {
        auto n = *it;
        adj.push_back(n.get_nidx());
    }
    std::sort(adj.begin(), adj.end());

    EXPECT_EQ(adj.size(), 2);
    EXPECT_EQ(adj[0], 1);
    EXPECT_EQ(adj[1], 3);

    /** All faces in iteration should be valid */
    for (auto it = model.face_begin(); it != model.face_end(); ++it) {
        EXPECT_TRUE((*it).is_valid());
    }
}

/** Test Ability to Remove Triangles with Components*/
TEST_F(SimpleTriModelTestClass, CompRemoveFace) {

    model.initialize_components();
    model.set_component(0, 1);
    model.set_component(1, 1);
    model.set_component(2, 2);
    model.set_component(3, 3);

    std::cout << "removing face" << std::endl;
    model.remove_face(2);
    std::cout << "finished removing face" << std::endl;
    
    EXPECT_TRUE(model.is_valid());
    EXPECT_EQ(model.number_of_nodes(), 9);
    EXPECT_EQ(model.number_of_faces(), 3);

    EXPECT_EQ(model.number_of_adj_faces(0), 1);
    EXPECT_EQ(model.number_of_adj_faces(1), 1);
    EXPECT_EQ(model.number_of_adj_faces(3), 0);

    EXPECT_EQ(model.number_of_components(), 2);

    /** Find all nodes adjacent to Node 2
     *  should be 0,1
     */
    std::vector<node_idx> adj;
    for (auto it=model.adj_node_begin(2); it != model.adj_node_end(2); ++it) {
        auto n = *it;
        adj.push_back(n.get_nidx());
    }
    std::sort(adj.begin(), adj.end());

    EXPECT_EQ(adj.size(), 2);
    EXPECT_EQ(adj[0], 0);
    EXPECT_EQ(adj[1], 1);
}

/** Test Ability to Split a Triangle by a Plane through 0 or 1 edge */
TEST_F(SimpleTriModelTestClass, SplitFace01Edge) {

    unsigned int curr_faces = model.number_of_faces();

    typedef upsp::FaceMap<face_idx> FaceMap;

    /*******************************************/
    /* No faces cut                            */
    /*******************************************/
    // Try a plane that does not cut any faces
    upsp::Plane<float> pl({1,0,0},{2,0,0});
    FaceMap new_faces = model.split(0, pl);
    EXPECT_EQ(new_faces.size(), 0);
    new_faces.clear();

    new_faces = model.split(1, pl);
    EXPECT_EQ(new_faces.size(), 0);
    new_faces.clear();

    /*******************************************/
    /* Cut 1st node position (0,1,2) -> 0      */
    /*******************************************/

    // check that normals are consistent
    // check that area is consistent
    upsp::Plane<float> pl2({1,0,0},{0,0,0});
    Face f1 = model.face(1);
    float area1 = f1.get_area();
    Face f2 = model.face(2);
    float area2 = f2.get_area();

    new_faces = model.split(1, pl2);

    ASSERT_TRUE(new_faces.has(1));
    EXPECT_EQ(new_faces.size(1), 2);
    curr_faces += 2;
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    float new_area = 0.0;
    const std::set<face_idx>& childs = new_faces.children(1);
    for (auto it = childs.begin(); it != childs.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area1, new_area);

    // this cut also impacts face 2
    EXPECT_FALSE(model.is_face(2));
    ASSERT_TRUE(new_faces.has(2));
    EXPECT_EQ(new_faces.size(2), 2);

    new_area = 0.0;
    const std::set<face_idx>& childs2 = new_faces.children(2);
    for (auto it = childs2.begin(); it != childs2.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area2, new_area);
    new_faces.clear();

    /*******************************************/
    /* Cut 2nd node position (6,8,7) -> 8      */
    /*******************************************/
    Face f = model.face(3);
    float area = f.get_area();

    new_faces = model.split(3, pl2);

    ASSERT_TRUE(new_faces.has(3));
    EXPECT_EQ(new_faces.size(3), 2);
    EXPECT_EQ(model.number_of_faces(), ++curr_faces);

    new_area = 0.0;
    const std::set<face_idx>& childs3 = new_faces.children(3);
    for (auto it = childs3.begin(); it != childs3.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area, new_area);
    new_faces.clear();

    /*******************************************/
    /* Cut 3rd node position (5,1,0) -> 0      */
    /*******************************************/
    upsp::Plane<float> pl3({-1,1.5,0},{0,0,0});
    f = model.face(0);
    area = f.get_area();

    new_faces = model.split(0, pl3);

    ASSERT_TRUE(new_faces.has(0));
    EXPECT_EQ(new_faces.size(0), 2);
    EXPECT_EQ(model.number_of_faces(), ++curr_faces);

    new_area = 0.0;
    const std::set<face_idx>& childs0 = new_faces.children(0);
    for (auto it = childs0.begin(); it != childs0.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area, new_area);
    new_faces.clear();
}

/** Test Ability to Split a Triangle by a Plane through 2 edges */
TEST_F(SimpleTriModelTestClass, SplitFace2Edges) {

    unsigned int curr_faces = model.number_of_faces();

    typedef upsp::FaceMap<face_idx> FaceMap;

    // Intersect through 2 edges of a face
    // try all node positions as the odd node out

    /*******************************************/
    /* Cut 1st node position (5,1,0) -> 5      */
    /*******************************************/
    upsp::Plane<float> pl({-1,1,0},{-1.0,0,0});
    Face f = model.face(0);
    float area = f.get_area();

    FaceMap new_faces = model.split(0, pl);

    EXPECT_EQ(new_faces.size(), 1);
    ASSERT_TRUE(new_faces.has(0));
    EXPECT_EQ(new_faces.size(0), 3);

    curr_faces += 2;
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    float new_area = 0.0;
    const std::set<face_idx>& childs0 = new_faces.children(0);
    for (auto it = childs0.begin(); it != childs0.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area, new_area);
    new_faces.clear();

    /*******************************************/
    /* Cut 2nd node position (0,1,2) -> 1      */
    /*******************************************/
    upsp::Plane<float> pl2({1,1,0},{0,-1,0});
    Face f1 = model.face(1);
    float area1 = f1.get_area();
    Face f2 = model.face(2);
    float area2 = f2.get_area();  

    new_faces = model.split(1, pl2);

    ASSERT_TRUE(new_faces.has(1));
    EXPECT_EQ(new_faces.size(1), 3);
    curr_faces += 4; // impacts part of old face 0 and face 2
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    new_area = 0.0;
    const std::set<face_idx>& childs1 = new_faces.children(1);
    for (auto it = childs1.begin(); it != childs1.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area1, new_area);

    // also impacts face 2
    EXPECT_FALSE(model.is_face(2));
    ASSERT_TRUE(new_faces.has(2));
    EXPECT_EQ(new_faces.size(2), 2);

    new_area = 0.0;
    const std::set<face_idx>& childs2 = new_faces.children(2);
    for (auto it = childs2.begin(); it != childs2.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area2, new_area);
    new_faces.clear();

    /*******************************************/
    /* Cut 3rd node position (6,8,7) -> 7      */
    /*******************************************/
    upsp::Plane<float> pl3({1,1,0},{0,-4.0,0});
    f = model.face(3);
    area = f.get_area();

    new_faces = model.split(3, pl3);

    ASSERT_TRUE(new_faces.has(3));
    EXPECT_EQ(new_faces.size(3), 3);
    curr_faces += 2;
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    new_area = 0.0;
    const std::set<face_idx>& childs3 = new_faces.children(3);
    for (auto it = childs3.begin(); it != childs3.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area, new_area);
    new_faces.clear();

}

/** Test Ability to Split a Triangle by a Plane (with components) */
TEST_F(SimpleTriModelTestClass, SplitFaceComp) {

    typedef upsp::FaceMap<face_idx> FaceMap;

    model.initialize_components(1);
    model.set_component(1, 2);

    // Test Single split
    upsp::Plane<float> pl1({1,0,0},{0,0,0});
    FaceMap new_faces = model.split(1, pl1); // first node position (0,1,2) -> through 0

    ASSERT_TRUE(new_faces.has(1));
    const std::set<face_idx>& childs1 = new_faces.children(1);
    for (auto it = childs1.begin(); it != childs1.end(); ++it) {
        EXPECT_EQ(model.get_component(*it), 2);
    }
    new_faces.clear();

    // Test Double split
    upsp::Plane<float> pl2({1,1,0},{0,-1.0,0});
    new_faces = model.split(0, pl2); // third node position (5,1,0) -> left 3

    ASSERT_TRUE(new_faces.has(0));
    const std::set<face_idx>& childs0 = new_faces.children(0);
    for (auto it = childs0.begin(); it != childs0.end(); ++it) {
        EXPECT_EQ(model.get_component(*it), 1);
    }
    new_faces.clear();

}

/** Test Ability to Split a Triangle by a Polyhedron */
TEST_F(SimpleTriModelTestClass, SplitFacePoly) {

    typedef upsp::FaceMap<face_idx> FaceMap;

    model.initialize_components(1);
    model.set_component(2,2);

    unsigned int curr_faces = model.number_of_faces();
    
    float epsilon = 0.00001;

    /*******************************************/
    /* No Cutting                              */
    /*******************************************/
    upsp::BoundingBox_<cv::Point3f> bb1({-2,-1,-1},{0,0,1});
    upsp::Polyhedron<6,float> poly = upsp::convert(bb1);

    FaceMap new_faces = model.split(0,poly);

    EXPECT_EQ(new_faces.size(), 0);
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    /*******************************************/
    /* Intersect with 1 plane                  */
    /*******************************************/
    std::cout << "Intersect by 1 plane" << std::endl;
    upsp::BoundingBox_<cv::Point3f> bb2({-2,-0.6,-1},{0,0,1});
    poly = upsp::convert(bb2);

    Face f0 = model.face(0);
    float area = f0.get_area();
    Face f1 = model.face(1);
    float area1 = f1.get_area();
    new_faces = model.split(0,poly);
   
    ASSERT_TRUE(new_faces.has(0));
    EXPECT_EQ(new_faces.size(0), 3);
    curr_faces += 3 + 1 - 1; // also splits face 1 into 2 new faces
    EXPECT_EQ(model.number_of_faces(), curr_faces);

    float new_area = 0.0;
    const std::set<face_idx>& childs0 = new_faces.children(0);
    for (auto it = childs0.begin(); it != childs0.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        EXPECT_EQ(model.get_component(*it), 1);
        new_area += f.get_area();
    }
    EXPECT_TRUE(std::abs(area - new_area) < epsilon);

    // modified face 1
    EXPECT_FALSE(model.is_face(1));    
    ASSERT_TRUE(new_faces.has(1));
    EXPECT_EQ(new_faces.size(1), 2);

    new_area = 0.0;
    const std::set<face_idx>& childs1 = new_faces.children(1);
    for (auto it = childs1.begin(); it != childs1.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += f.get_area();
    }
    EXPECT_FLOAT_EQ(area1, new_area);
    new_faces.clear();

    /*******************************************/
    /* Intersect with multiple planes          */
    /*******************************************/
    std::cout << "Intersect by multiple planes" << std::endl;
    upsp::BoundingBox_<cv::Point3f> bb3({0,-1.4,-1},{0.5,0,1});
    poly = upsp::convert(bb3);

    Face f2 = model.face(2);
    area = f2.get_area();
    new_faces = model.split(2,poly);
    
    ASSERT_TRUE(new_faces.has(2));
    EXPECT_TRUE(new_faces.size(2) >= 2);

    new_area = 0.0;
    const std::set<face_idx>& childs2 = new_faces.children(2);
    for (auto it = childs2.begin(); it != childs2.end(); ++it) {
        Face f = model.face(*it);
        cv::Point3f n = f.get_normal();
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        EXPECT_EQ(model.get_component(*it), 2);
        new_area += f.get_area();
    }
    EXPECT_TRUE(std::abs(area - new_area) < epsilon);
    
}

/** Test gathering faces from a list of nodes */
TEST_F(SimpleTriModelTestClass, GatherFaces) {

    // One Node from each triangle
    std::set<node_idx> nodes;
    nodes.insert(1);
    nodes.insert(7);

    std::set<face_idx> faces = model.gather_faces(nodes);
    EXPECT_EQ(faces.size(), 4);

    // No triangles   
    nodes.clear();
    nodes.insert(4);
    
    faces = model.gather_faces(nodes);
    EXPECT_EQ(faces.size(), 0);

    // All nodes
    nodes.insert(0);
    nodes.insert(1);
    nodes.insert(2);
    nodes.insert(3);
    nodes.insert(5);
    nodes.insert(6);
    nodes.insert(7);
    nodes.insert(8);

    faces = model.gather_faces(nodes);
    EXPECT_EQ(faces.size(), 4);
}
