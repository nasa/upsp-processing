/** @file
 *  @brief  Test the P3DModel class
 *  @date   May 2, 2019
 *  @author jmpowel2
 */
#include <algorithm>
#include "gtest/gtest.h"

#include "../include/grids.h"
#include "../include/P3DModel.h"

#include "test_grid_utils.h"

/* TODO: Still need Tests for the following:
 *
 * EdgeIterator
 * AdjNodeIterator
 * AdjFaceIterator
 * adjust_solution
 * transform 
 * set_normals
 * write_tri_grid
 * write_triq_sol
 * write_p3d_sol
 */

/** Test Basic Initialization of Simple Model */
TEST_F(SimpleP3DModelTestClass, InitializeSingle) {

    EXPECT_TRUE(model->is_structured());
    EXPECT_EQ(model->size(), 20);
    EXPECT_EQ(model->number_of_faces(), 12);
    EXPECT_EQ(model->num_zones(), 1);
    EXPECT_EQ(model->zone_size(0), 20);
    EXPECT_EQ(model->zone_size(0,0), 4);
    EXPECT_EQ(model->zone_size(0,1), 5);
    EXPECT_EQ(model->zone_start_idx(0), 0);
}

/** Test Basic Initialization of multi simple model */
TEST_F(SimpleP3DModelTestClass, InitializeMulti) {

    add_zones();

    EXPECT_TRUE(model->is_structured());
    EXPECT_EQ(model->size(), 52);
    EXPECT_EQ(model->number_of_faces(), 12 + 6 + 12);
    EXPECT_EQ(model->num_zones(), 3);

    EXPECT_EQ(model->zone_size(0), 20);
    EXPECT_EQ(model->zone_size(1), 12);
    EXPECT_EQ(model->zone_size(2), 20);

    EXPECT_EQ(model->zone_size(0,0), 4);
    EXPECT_EQ(model->zone_size(0,1), 5);
    EXPECT_EQ(model->zone_size(1,0), 3);
    EXPECT_EQ(model->zone_size(1,1), 4);
    EXPECT_EQ(model->zone_size(2,0), 5);
    EXPECT_EQ(model->zone_size(2,1), 4);

    EXPECT_EQ(model->zone_start_idx(0), 0);
    EXPECT_EQ(model->zone_start_idx(1), 20);
    EXPECT_EQ(model->zone_start_idx(2), 32);
}

/** Test Node Class */
TEST_F(SimpleP3DModelTestClass, NodeClass) {

    // Test invalid node
    Node n_invalid;
    EXPECT_FALSE(n_invalid.is_valid());

    // Define Nodes
    upsp::GridIndex gidx(0,0,0,0); // corner node
    Node n  = model->node(0);
    Node n_equal = model->node(gidx);

    upsp::GridIndex gidx2(0,1,1,0); // corner node
    Node n2 = model->node(5);
    Node n2_equal = model->node(gidx2);

    // Test validity
    EXPECT_TRUE(n.is_valid());
    EXPECT_TRUE(n2.is_valid());

    EXPECT_TRUE(n_equal.is_valid());
    EXPECT_TRUE(n2_equal.is_valid());

    // Test Equality
    EXPECT_TRUE(n == n_equal);
    EXPECT_TRUE(n2 == n2_equal);

    // Test Corner node
    EXPECT_EQ(n.get_zone(), 0);

    cv::Point3f pos = n.get_position();
    EXPECT_FLOAT_EQ(pos.x, 0.0);
    EXPECT_FLOAT_EQ(pos.y, 0.0);
    EXPECT_FLOAT_EQ(pos.z, 0.0);

    EXPECT_EQ(n.get_gidx(), gidx);
    EXPECT_EQ(n.get_nidx(), 0);
    EXPECT_TRUE(n.is_datanode());

    cv::Point3f normal = n.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0); 
    EXPECT_FLOAT_EQ(normal.y, 0.0); 
    EXPECT_FLOAT_EQ(normal.z, 1.0); 
    
    EXPECT_TRUE(n < n2);

    // Test Middle 
    EXPECT_EQ(n2.get_zone(), 0);

    cv::Point3f pos2 = n2.get_position();
    EXPECT_FLOAT_EQ(pos2.x, 1.0);
    EXPECT_FLOAT_EQ(pos2.y, 1.0);
    EXPECT_FLOAT_EQ(pos2.z, 0.0);

    EXPECT_EQ(n2.get_gidx(), gidx2);
    EXPECT_EQ(n2.get_nidx(), 5);
    EXPECT_TRUE(n2.is_datanode());

    cv::Point3f normal2 = n2.get_normal();
    EXPECT_FLOAT_EQ(normal2.x, 0.0); 
    EXPECT_FLOAT_EQ(normal2.y, 0.0); 
    EXPECT_FLOAT_EQ(normal2.z, 1.0); 

    // Create multi-zone model and test one of those nodes
    add_zones();

    gidx = upsp::GridIndex(1,0,0,0);
    n = model->node(20);
    n_equal = model->node(gidx);
    gidx2 = upsp::GridIndex(2,4,3,0);
    n2 = model->node(51);
    n2_equal = model->node(gidx2);

    EXPECT_TRUE(n.is_valid());
    EXPECT_TRUE(n2.is_valid());
    EXPECT_TRUE(n_equal.is_valid());
    EXPECT_TRUE(n2_equal.is_valid());

    EXPECT_TRUE(n == n_equal);
    EXPECT_TRUE(n2 == n2_equal);
    // these are not equal, because the Node class uses a simple
    // determination of equality based on the node index (not overlap)

    pos = n.get_position();
    EXPECT_FLOAT_EQ(pos.x, 3.0);
    EXPECT_FLOAT_EQ(pos.y, 0.0);
    EXPECT_FLOAT_EQ(pos.z, 0.0);

    EXPECT_EQ(n.get_gidx(), gidx);
    EXPECT_EQ(n.get_nidx(), 20);
    EXPECT_TRUE(n.is_datanode());

    normal = n.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0); 
    EXPECT_FLOAT_EQ(normal.y, 0.0); 
    EXPECT_FLOAT_EQ(normal.z, 1.0); 
    
    EXPECT_TRUE(n < n2);
}

/** Test Node Iterator */
TEST_F(SimpleP3DModelTestClass, NodeIterator) {

    // Single Zone
    int count = 0;
    for (auto it=model->node_begin(); it != model->node_end(); ++it) {
        Node n = *it;
        EXPECT_EQ(n.get_nidx(), count);
        ++count;
    }
    EXPECT_EQ(model->size(), count);

    // Multiple Zone
    // just check that it correctly skips superceded nodes
    // more overlap testing occurs in a different test
    add_zones();
    count = 0;
    for (auto it=model->node_begin(); it != model->node_end(); ++it) {
        Node n = *it;
        EXPECT_FALSE(model->is_superceded(n.get_nidx()));
        ++count;
    }
    EXPECT_EQ(count, 52 - 7);
}

/** Test Model overlap for tol=0.0 */
TEST_F(SimpleP3DModelTestClass, ExactNodeOverlap) {

    add_zones();

    // test superceded nodes
    EXPECT_TRUE(model->is_superceded(20));
    EXPECT_TRUE(model->is_superceded(23));
    EXPECT_TRUE(model->is_superceded(26));
    EXPECT_TRUE(model->is_superceded(29));
    EXPECT_TRUE(model->is_superceded(51));
    EXPECT_TRUE(model->is_superceded(46));
    EXPECT_TRUE(model->is_superceded(41));
}

/** Test Tolerance handling for simple model */
TEST_F(SimpleP3DModelTestClass, TolNodeOverlap) {

    // Test with tol too small
    float offset = 0.1;
    float tol = 0.09;

    add_zones(offset, tol);

    int count = 0;
    for (auto it=model->node_begin(); it != model->node_end(); ++it) {
        ++count;
    }
    EXPECT_EQ(model->size(), count);

    // Test with good tolerance
    offset = 0.1;
    tol = 0.100001;

    add_zones(offset, tol);

    count = 0;
    for (auto it=model->node_begin(); it != model->node_end(); ++it) {
        ++count;
    }
    EXPECT_EQ(model->size() - 7, count);
}

/** Test Face Class for single zone */
TEST_F(SimpleP3DModelTestClass, FaceClassSingleZone) {

    // Test Invalid Face
    Face f_invalid;
    EXPECT_FALSE(f_invalid.is_valid());    

    // Test most functions for 1 face
    Face f = model->face(4);
    
    EXPECT_TRUE(f.is_valid());
    EXPECT_EQ(f.get_fidx(), 4);
    EXPECT_TRUE(f.has_node(5));
    EXPECT_TRUE(f.has_node(6));
    EXPECT_TRUE(f.has_node(9));
    EXPECT_TRUE(f.has_node(10));
    
    cv::Point3f normal = f.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0);
    EXPECT_FLOAT_EQ(normal.y, 0.0);
    EXPECT_FLOAT_EQ(normal.z, 1.0);

    EXPECT_FLOAT_EQ(f.get_area(), 1.0);

    std::vector<node_idx> nodes = f.get_nodes();
    std::sort(nodes.begin(), nodes.end());
    EXPECT_EQ(nodes[0], 5);
    EXPECT_EQ(nodes[1], 6);
    EXPECT_EQ(nodes[2], 9);
    EXPECT_EQ(nodes[3], 10);

    EXPECT_EQ(f.get_zone(), 0);

    EXPECT_TRUE(f == f);

    // Test adjacent faces
    Face adj1 = f.get_adjacent(upsp::GridDir::J_plus); // +j
    EXPECT_EQ(adj1.get_fidx(), 5);
    adj1 = f.get_adjacent(upsp::GridDir::J_minus); // -j
    EXPECT_EQ(adj1.get_fidx(), 3);
    adj1 = f.get_adjacent(upsp::GridDir::K_plus); // +k
    EXPECT_EQ(adj1.get_fidx(), 7);
    adj1 = f.get_adjacent(upsp::GridDir::K_minus); // -k
    EXPECT_EQ(adj1.get_fidx(), 1);
   
    if (adj1.is_valid()) { 
        Face adj2 = adj1.get_adjacent(upsp::GridDir::K_minus);  // does not exist
        EXPECT_FALSE(adj2.is_valid());
    }

    // Test edge nodes
    std::array<Node,2> simple_edge = f.get_edge_nodes(upsp::GridDir::J_plus);
    EXPECT_TRUE(simple_edge[0].get_nidx() == 6);
    EXPECT_TRUE(simple_edge[1].get_nidx() == 10);
    simple_edge = f.get_edge_nodes(upsp::GridDir::J_minus);
    EXPECT_TRUE(simple_edge[0].get_nidx() == 5);
    EXPECT_TRUE(simple_edge[1].get_nidx() == 9);
    simple_edge = f.get_edge_nodes(upsp::GridDir::K_plus);
    EXPECT_TRUE(simple_edge[0].get_nidx() == 9);
    EXPECT_TRUE(simple_edge[1].get_nidx() == 10);
    simple_edge = f.get_edge_nodes(upsp::GridDir::K_minus);
    EXPECT_TRUE(simple_edge[0].get_nidx() == 5);
    EXPECT_TRUE(simple_edge[1].get_nidx() == 6);

    // Test initializing the face with grid indices
    // try every permutation of inputs
    std::array<upsp::GridIndex,4> gids;
    gids[0] = upsp::GridIndex(0,1,1,0);
    gids[1] = upsp::GridIndex(0,2,1,0);
    gids[2] = upsp::GridIndex(0,1,2,0);
    gids[3] = upsp::GridIndex(0,2,2,0);
    for (unsigned int i=0; i < 4; ++i ) {
        for (unsigned int j=0; j < 4; ++j) {
            if (i == j) {
                continue;
            }
            for (unsigned int k=0; k < 4; ++k) {
                if ( (i == k) || (j == k) ) {
                    continue;
                }
                Face f2 = model->face(gids[i],gids[j],gids[k]);

                EXPECT_TRUE(f == f2);
                if (f != f2) {
                    break;
                }

                normal = f2.get_normal();
                EXPECT_FLOAT_EQ(normal.x, 0.0);
                EXPECT_FLOAT_EQ(normal.y, 0.0);
                EXPECT_FLOAT_EQ(normal.z, 1.0);
            }
        }
    }
}   

/** Test Face Class for multiple zones */
TEST_F(SimpleP3DModelTestClass, FaceClassMultiZone) {

    add_zones();

    // Test most functions for 1 face (on the boundary)
    Face f = model->face(12);
    
    EXPECT_TRUE(f.is_valid());
    EXPECT_EQ(f.get_fidx(), 12);
    EXPECT_TRUE(f.has_node(3));
    EXPECT_TRUE(f.has_node(20));
    EXPECT_TRUE(f.has_node(51));
    EXPECT_TRUE(f.has_node(7));
    EXPECT_TRUE(f.has_node(23));
    EXPECT_TRUE(f.has_node(21));
    EXPECT_TRUE(f.has_node(46));
    EXPECT_TRUE(f.has_node(24));
    
    cv::Point3f normal = f.get_normal();
    EXPECT_FLOAT_EQ(normal.x, 0.0);
    EXPECT_FLOAT_EQ(normal.y, 0.0);
    EXPECT_FLOAT_EQ(normal.z, 1.0);

    EXPECT_FLOAT_EQ(f.get_area(), 1.0);

    std::vector<node_idx> nodes = f.get_nodes(); // will be lowest nidx
    std::sort(nodes.begin(), nodes.end());
    EXPECT_EQ(nodes[0], 3);
    EXPECT_EQ(nodes[1], 7);
    EXPECT_EQ(nodes[2], 21);
    EXPECT_EQ(nodes[3], 24);

    EXPECT_EQ(f.get_zone(), 1);

    EXPECT_TRUE(f == f);

    // Test initializing the face with grid indices
    // try every permutation of inputs
    std::array<upsp::GridIndex,4> gids;
    gids[0] = upsp::GridIndex(0,3,0,0);
    gids[1] = upsp::GridIndex(0,3,1,0);
    gids[2] = upsp::GridIndex(1,1,1,0);
    gids[3] = upsp::GridIndex(2,4,2,0);
    for (unsigned int i=0; i < 4; ++i ) {
        for (unsigned int j=0; j < 4; ++j) {
            if (i == j) {
                continue;
            }
            for (unsigned int k=0; k < 4; ++k) {
                if ( (i == k) || (j == k) ) {
                    continue;
                }
                Face f2 = model->face(gids[i],gids[j],gids[k]);

                EXPECT_TRUE(f == f2);
                if (f != f2) {
                    std::cout << " failed permutation: " << i << ", ";
                    std::cout << j << ", " << k << std::endl;
                    break;
                }

                normal = f2.get_normal();
                EXPECT_FLOAT_EQ(normal.x, 0.0);
                EXPECT_FLOAT_EQ(normal.y, 0.0);
                EXPECT_FLOAT_EQ(normal.z, 1.0);
            }
        }
    }

    // Test getting adjacent faces
    Face adj1 = f.get_adjacent(upsp::GridDir::J_plus); // +j
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 13);
        EXPECT_EQ(adj1.get_zone(), 1);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
    adj1 = f.get_adjacent(upsp::GridDir::J_minus); // -j
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 2);
        EXPECT_EQ(adj1.get_zone(), 0);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
    adj1 = f.get_adjacent(upsp::GridDir::K_plus); // +k
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 14);
        EXPECT_EQ(adj1.get_zone(), 1);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
    adj1 = f.get_adjacent(upsp::GridDir::K_minus); // -k
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 29);
        EXPECT_EQ(adj1.get_zone(), 2);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
   
    if (adj1.is_valid()) { 
        Face adj2 = adj1.get_adjacent(upsp::GridDir::K_plus);  // +k, does not exist
        EXPECT_FALSE(adj2.is_valid());
    }

    // Create another adjacent face test
    Face f2 = model->face(29);

    adj1 = f2.get_adjacent(upsp::GridDir::J_plus); //+j
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 12);
        EXPECT_EQ(adj1.get_zone(), 1);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
    adj1 = f2.get_adjacent(upsp::GridDir::J_minus); //-j
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 28);
        EXPECT_EQ(adj1.get_zone(), 2);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }
    adj1 = f2.get_adjacent(upsp::GridDir::K_minus); //-k
    if (adj1.is_valid()) {
        EXPECT_EQ(adj1.get_fidx(), 25);
        EXPECT_EQ(adj1.get_zone(), 2);
    } else {
        EXPECT_TRUE(adj1.is_valid());
    }

    // Test edge nodes
    std::array<Node,2> simple_edge = f.get_edge_nodes(upsp::GridDir::J_plus);
    EXPECT_TRUE(simple_edge[0].overlaps(21));
    EXPECT_TRUE(simple_edge[1].overlaps(24));
    simple_edge = f.get_edge_nodes(upsp::GridDir::J_minus);
    EXPECT_TRUE(simple_edge[0].overlaps(3));
    EXPECT_TRUE(simple_edge[1].overlaps(7));
    simple_edge = f.get_edge_nodes(upsp::GridDir::K_plus);
    EXPECT_TRUE(simple_edge[0].overlaps(7));
    EXPECT_TRUE(simple_edge[1].overlaps(24));
    simple_edge = f.get_edge_nodes(upsp::GridDir::K_minus);
    EXPECT_TRUE(simple_edge[0].overlaps(3));
    EXPECT_TRUE(simple_edge[1].overlaps(21));

}   

/** Test Face iterator */
TEST_F(SimpleP3DModelTestClass, FaceIterator) {

    // Single Zone
    int count = 0;
    for (auto it=model->face_begin(); it != model->face_end(); ++it) {
        ++count;
    }
    EXPECT_EQ(model->number_of_faces(), count);

    // Multiple Zone
    add_zones();
    count = 0;
    for (auto it=model->face_begin(); it != model->face_end(); ++it) {
        ++count;
    }
    EXPECT_EQ(model->number_of_faces(), count);
}
