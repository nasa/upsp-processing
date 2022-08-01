/** @file
 *  @brief  Test the grids.h structures
 *  @date   May 2, 2019
 *  @author jmpowel2
 */
#include "gtest/gtest.h"
#include "../include/grids.h"
#include "test_grid_utils.h"

/** Test StructuredGrid single zone struct */
TEST_F(SimpleSingleStructGridTestClass, InitializeSingleStruct) {

    EXPECT_TRUE(model.is_structured());
    EXPECT_EQ(model.size(), 20);
    EXPECT_EQ(model.num_zones(), 1);
    EXPECT_EQ(model.zone_size(0), 20);
    EXPECT_EQ(model.zone_size(0,0), 4);
    EXPECT_EQ(model.zone_size(0,1), 5);
    EXPECT_EQ(model.zone_start_idx(0), 0);
}

/** Test StructuredGrid multi zone struct */
TEST_F(SimpleSingleStructGridTestClass, InitializeMultiStruct) {

    add_zones();

    EXPECT_TRUE(model.is_structured());
    EXPECT_EQ(model.size(), 52);
    EXPECT_EQ(model.num_zones(), 3);

    EXPECT_EQ(model.zone_size(0), 20);
    EXPECT_EQ(model.zone_size(1), 12);
    EXPECT_EQ(model.zone_size(2), 20);

    EXPECT_EQ(model.zone_size(0,0), 4);
    EXPECT_EQ(model.zone_size(0,1), 5);
    EXPECT_EQ(model.zone_size(1,0), 3);
    EXPECT_EQ(model.zone_size(1,1), 4);
    EXPECT_EQ(model.zone_size(2,0), 5);
    EXPECT_EQ(model.zone_size(2,1), 4);

    EXPECT_EQ(model.zone_start_idx(0), 0);
    EXPECT_EQ(model.zone_start_idx(1), 20);
    EXPECT_EQ(model.zone_start_idx(2), 32);
}

/** Test UnstructuredGrid sorting */
TEST(UnstructuredGridTest, SortByComponent) {

    typedef upsp::UnstructuredGrid<float> Grid;
    typedef Grid::Triangle Triangle;

    // (0) - (1) - (2) - (3)
    //   \   / \   / \   / \
    //    (4) - (5) - (6) - (7)
    //    / \   / \   / \   /
    // (8) - (9) - (10)- (11)
    
    upsp::UnstructuredGrid<float> grid;
    
    // initialize node locations
    static const float arr_x[] = {0.0, 2.0, 4.0, 6.0, 1.0, 3.0, 5.0, 7.0, 0.0, 2.0, 4.0, 6.0};
    static const float arr_y[] = {0.0, 0.0, 0.0, 0.0,-1.0,-1.0,-1.0,-1.0,-2.0,-2.0,-2.0,-2.0};
    
    grid.x.assign(arr_x, arr_x + sizeof(arr_x) / sizeof(arr_x[0]));
    grid.y.assign(arr_y, arr_y + sizeof(arr_y) / sizeof(arr_y[0]));
    grid.z.assign(grid.x.size(), 0.0);

    // initialize triangles
    grid.tris.push_back({ 0, 4, 1});
    grid.tris.push_back({ 1, 4, 5});
    grid.tris.push_back({ 1, 5, 2});
    grid.tris.push_back({ 2, 5, 6});
    grid.tris.push_back({ 2, 6, 3});
    grid.tris.push_back({ 3, 6, 7});
    grid.tris.push_back({ 4, 8, 9});
    grid.tris.push_back({ 4, 9, 5});
    grid.tris.push_back({ 5, 9,10});
    grid.tris.push_back({ 5,10, 6});
    grid.tris.push_back({ 6,10,11});
    grid.tris.push_back({ 6,11, 7});

    std::vector<CircularArray<unsigned int,3>> tri_copy(grid.tris);

    // initialize components
    static const int arr_comps[] = {2, 2, 1, 2, 3, 1, 1, 0, 3, 4, 0, 2};
    grid.comps.assign(arr_comps, arr_comps + sizeof(arr_comps) / sizeof(arr_comps[0]));

    // sort
    grid.sort_by_components();

    // check that number of components has been set
    ASSERT_TRUE(grid.has_n_comps);
    ASSERT_EQ(grid.n_comps, 5);

    // brute force, check that triangles still have their same component
    // number
    for (unsigned int i=0; i < grid.tris.size(); ++i) {
        for (unsigned int j=0; j < grid.tris.size(); ++j) {
            if (grid.tris[i] == tri_copy[j]) {
                EXPECT_TRUE(grid.comps[i] == arr_comps[j]);
                break;
            }
        }
    }

} 
