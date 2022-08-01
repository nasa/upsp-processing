/** @file
 *  @brief  Test the models.h functions
 *  @date   April 26, 2019
 *  @author jmpowel2
 */
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "../include/models.h"

#include "test_grid_utils.h"

/* TODO: Still need Tests for:
 * 
 * get_extent
 * get_extent2D
 * normal
 * area
 * intersects (bb,bb) and (bb,pt)
 * intersects (poly,bb) -> needs more cases
 * distance (bb,pt)
 * combine_solutions
 * approx_nearest_node
 * nearest_neighbor_tol
 * nearest_neighbor
 * rotate2D
 * find_intersecting_planes
 * contains(Poly,Tri)
 */

/** Test shortest_distance */
TEST(ModelHTest, ShortestDistance) {

    // Define simple plane
    upsp::Plane<float> pl1;
    pl1.n  = {1,0,0};
    pl1.pt = {0,0,0};

    // Test on plane
    cv::Point3f pt1(0,0,0);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl1,pt1), 0.0);

    // Test in positive direction
    cv::Point3f pt2(2,0,0);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl1,pt2), 2.0);

    // Test in negative direction
    cv::Point3f pt3(-1,0,0);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl1,pt3), -1.0);

    // Define a plane with non-zero values in each direction
    upsp::Plane<float> pl2;
    pl2.n  = {1,3,1};
    pl2.pt = {1,2,4};

    // Test on plane
    cv::Point3f pt4(1,2,4);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl2,pt4), 0.0);

    // Test in positive direction
    cv::Point3f pt5(2,7,5);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl2,pt5), 5.1256929);

    // Test in negative direction
    cv::Point3f pt6(0,-1,-3);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl2,pt6), -5.1256929);

    // Test with negative plane normal direction
    upsp::Plane<float> pl3({-1,-1,-1},{-1,-1,-1});
    cv::Point3f pt7(1,1,1);
    EXPECT_FLOAT_EQ(upsp::shortest_distance(pl3,pt7), -2.0*sqrt(3));
}

/** Test contains(BoundingBox, Point) */ 
TEST(ModelHTest, BBContainsPt) {

    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});

    // Check on min edge
    cv::Point3f pt0(-2,-1,-1);
    EXPECT_TRUE(upsp::contains(bb, pt0));
    
    // Check on max edge
    cv::Point3f pt1(0,0,0);
    EXPECT_FALSE(upsp::contains(bb, pt1));

    // Check in the middle
    cv::Point3f pt2(-1,-0.5,-0.5);
    EXPECT_TRUE(upsp::contains(bb, pt2));

    // Check on other edges
    cv::Point3f pt3(-2,0,0);
    cv::Point3f pt4(-2,-1,0);
    EXPECT_FALSE(upsp::contains(bb, pt3));
    EXPECT_FALSE(upsp::contains(bb, pt4));
}

/** Test contains(Polyhedron, Point) */
TEST(ModelHTest, PolyhedronContainsPt) {

    // Define points (referencing SimpleTriModelTestClass)
    cv::Point3f pt0(0,0,0);
    cv::Point3f pt1(-1,-1,0);
    cv::Point3f pt2(1,-1,0);
    cv::Point3f pt3(0,-2,0);
    cv::Point3f pt5(-2,0,0);

    // define extra points (not in simpletri)
    cv::Point3f pt9(-1,-1,1);

    // Define a polyhedron
    upsp::Polyhedron<5, float> poly;
    poly.planes[0].pt = { 0, 0, 0}; // aligns with edge (0-1)
    poly.planes[0].n  = { 1,-1, 0}; 
    poly.planes[1].pt = {-2, 0, 0}; // aligns with edge (5-1)
    poly.planes[1].n  = {-1,-1, 0};
    poly.planes[2].pt = { 0, 1, 0}; // above all nodes, pointing up
    poly.planes[2].n  = { 0, 1, 0};
    poly.planes[3].pt = { 0, 0, 1};
    poly.planes[3].n  = { 0, 0, 1};
    poly.planes[4].pt = { 0, 0,-1};
    poly.planes[4].n  = { 0, 0,-1};

    // Test nodes
    EXPECT_TRUE(upsp::contains(poly, pt0));
    EXPECT_TRUE(upsp::contains(poly, pt1));
    EXPECT_FALSE(upsp::contains(poly, pt2));
    EXPECT_FALSE(upsp::contains(poly, pt3));
    EXPECT_TRUE(upsp::contains(poly, pt5));
    EXPECT_TRUE(upsp::contains(poly, pt9));
}

/** Test contains(Polyhedron, BoundingBox) */
TEST(ModelHTest, PolyhedronContainsBB) {

    // Define a bounding box
    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});

    // Convert bounding box to polyhedron
    upsp::Polyhedron<6,float> poly = upsp::convert(bb);

    // Test that it contains itself
    EXPECT_TRUE(upsp::contains(poly, bb));

    // Create a polyhedron that is partially within the bb
    poly.planes[0].pt = {0, 0, 0};
    poly.planes[0].n  = {1, 0, 0};
    poly.planes[1].pt = {0, 0, 0};
    poly.planes[1].n  = {0, 1, 0};
    poly.planes[2].pt = {0, 0, 0};
    poly.planes[2].n  = {0, 0, 1};
    poly.planes[3].pt = {-1, -1, -1};
    poly.planes[3].n  = {-1, 0, 0};
    poly.planes[4].pt = {-1, -1, -1};
    poly.planes[4].n  = {0, -1, 0};
    poly.planes[5].pt = {-1, -1, -1};
    poly.planes[5].n  = {0, 0, -1};

    EXPECT_FALSE(upsp::contains(poly, bb));
}

/** Test contains(BoundingBox, Polyhedron) */
TEST(ModelHTest, BBContainsPolyhedron) {

    // Define a bounding box
    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});

    // Convert bounding box to polyhedron
    upsp::Polyhedron<6,float> poly = upsp::convert(bb);

    // Test that it does not contains itself (because of bb [) )
    EXPECT_FALSE(upsp::contains(bb, poly));

    // Create a polyhedron that is partially within the bb on the +side
    poly.vertices[0] = {-2,-1,-1};
    poly.vertices[1] = {-2,-0.1,-1};
    poly.vertices[2] = {-2,-1,-0.1};
    poly.vertices[3] = {-2,-0.1,-0.1};
    poly.vertices[4] = {-0.1,-1,-1};
    poly.vertices[5] = {-0.1,-0.1,-1};
    poly.vertices[6] = {-0.1,-1,-0.1};
    poly.vertices[7] = {-0.1,-0.1,-0.1};

    // don't strictly need to modify planes, but done here for completeness
    poly.planes[0].pt = {-0.1, -0.1, -0.1};
    poly.planes[0].n  = {1, 0, 0};
    poly.planes[1].pt = {-0.1, -0.1, -0.1};
    poly.planes[1].n  = {0, 1, 0};
    poly.planes[2].pt = {-0.1, -0.1, -0.1};
    poly.planes[2].n  = {0, 0, 1};
    poly.planes[3].pt = {-2, -1, -1};
    poly.planes[3].n  = {-1, 0, 0};
    poly.planes[4].pt = {-2, -1, -1};
    poly.planes[4].n  = {0, -1, 0};
    poly.planes[5].pt = {-2, -1, -1};
    poly.planes[5].n  = {0, 0, -1};

    EXPECT_TRUE(upsp::contains(bb, poly));
}

/** Test contains(Polyhedron, Polyhedron) */
TEST(ModelHTest, PolyhedronContainsPolyhedron) {

    // Define a bounding box
    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});

    // Convert bounding box to polyhedron
    upsp::Polyhedron<6,float> poly_out = upsp::convert(bb);
    upsp::Polyhedron<6,float> poly_in  = upsp::convert(bb);

    // Test that it contains itself
    EXPECT_TRUE(upsp::contains(poly_out, poly_in));

    // Push poly_out in slightly
    poly_out.vertices[0] = {-1,-1,-1};
    poly_out.vertices[1] = {-1,0,-1};
    poly_out.vertices[2] = {-1,-1,0};
    poly_out.vertices[3] = {-1,0,0};
    poly_out.vertices[4] = {0,-1,-1};
    poly_out.vertices[5] = {0,0,-1};
    poly_out.vertices[6] = {0,-1,0};
    poly_out.vertices[7] = {0,0,0};

    // don't strictly need to modify planes, but done here for completeness
    poly_out.planes[0].pt = {0, 0, 0};
    poly_out.planes[0].n  = {1, 0, 0};
    poly_out.planes[1].pt = {0, 0, 0};
    poly_out.planes[1].n  = {0, 1, 0};
    poly_out.planes[2].pt = {0, 0, 0};
    poly_out.planes[2].n  = {0, 0, 1};
    poly_out.planes[3].pt = {-1, -1, -1};
    poly_out.planes[3].n  = {-1, 0, 0};
    poly_out.planes[4].pt = {-1, -1, -1};
    poly_out.planes[4].n  = {0, -1, 0};
    poly_out.planes[5].pt = {-1, -1, -1};
    poly_out.planes[5].n  = {0, 0, -1};

    EXPECT_FALSE(upsp::contains(poly_out, poly_in));
}

/** Test intersects(Polyhedron, BoundingBox) */
TEST(ModelHTest, PolyhedronIntersectsBB) {

    // Define a bounding box
    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});

    // Convert bounding box to polyhedron
    upsp::Polyhedron<6,float> poly  = upsp::convert(bb);

    // Test that it intersects itself
    EXPECT_TRUE(upsp::intersects(poly, bb));

    // Push poly in slightly
    poly.vertices[0] = {-1,-1,-1};
    poly.vertices[1] = {-1,0,-1};
    poly.vertices[2] = {-1,-1,0};
    poly.vertices[3] = {-1,0,0};
    poly.vertices[4] = {0,-1,-1};
    poly.vertices[5] = {0,0,-1};
    poly.vertices[6] = {0,-1,0};
    poly.vertices[7] = {0,0,0};

    poly.planes[0].pt = {0, 0, 0};
    poly.planes[0].n  = {1, 0, 0};
    poly.planes[1].pt = {0, 0, 0};
    poly.planes[1].n  = {0, 1, 0};
    poly.planes[2].pt = {0, 0, 0};
    poly.planes[2].n  = {0, 0, 1};
    poly.planes[3].pt = {-1, -1, -1};
    poly.planes[3].n  = {-1, 0, 0};
    poly.planes[4].pt = {-1, -1, -1};
    poly.planes[4].n  = {0, -1, 0};
    poly.planes[5].pt = {-1, -1, -1};
    poly.planes[5].n  = {0, 0, -1};

    EXPECT_TRUE(upsp::intersects(poly, bb));

    // Define a new bounding box, outside of poly (but on plus edge)
    upsp::BoundingBox_<cv::Point3f> bb2({-2,-2,-2},{-1,-1,-1});

    EXPECT_FALSE(upsp::intersects(poly, bb2));

    // Test case where neither contain the nodes of the other
    // but there is intersection
    upsp::BoundingBox_<cv::Point3f> bb3({-4,-1.6,-1.6},{0,-1.2,-1.2});
    upsp::Polyhedron<6,float> poly2 = upsp::convert(bb3);

    EXPECT_TRUE(upsp::intersects(poly2, bb2));
   
    // Could use many additional cases, should create a function
    // to rotate boundingbox and work on a loop from there 

    /*
    upsp::Polyhedron<5,float> poly2;

    poly2.vertices[0] = {0.0, 0.5, 1.0};
    poly2.vertices[1] = {0.5, 0.2, 1.0};
    poly2.vertices[2] = {0.5, 0.8, 1.0}; // top triangle
    poly2.vertices[3] = {3.0, 0.5, 0.0};
    poly2.vertices[4] = {3.5, 0.2, 0.0};
    poly2.vertices[5] = {3.5, 0.8, 0.0}; // bottom triangle

    cv::Point3f v1 = poly2.vertices[3] - poly2.vertices[0];
    cv::Point3f v2 = poly2.vertices[1] - poly2.vertices[0];
    cv::Point3f v3 = poly2.vertices[2] - poly2.vertices[0];
    
    cv::Point3f v4 = poly2.vertices[4] - poly2.vertices[1];
    cv::Point3f v5 = poly2.vertices[2] - poly2.vertices[1];

    poly2.planes[0].pt = poly2.vertices[0];
    poly2.planes[0].n = {0,0,1};
    poly2.planes[1].pt = poly2.vertices[3];
    poly2.planes[1].n = {0,0,-1};
    poly2.planes[2].pt = poly2.vertices[0];
    poly2.planes[2].n = v1.cross(v2);
    poly2.planes[3].pt = poly2.vertices[0];
    poly2.planes[3].n = v3.cross(v1);
    poly2.planes[4].pt = poly2.vertices[1];
    poly2.planes[4].n = v4.cross(v5);

    EXPECT_TRUE(upsp::intersects(poly2, bb));
    */
}

/** Test nodes_within(Polyhedron, Octree) */
TEST_F(SimpleTriModelTestClass, SimpleNodesWithin) {

    // put model into an octree
    upsp::Octree<Wrapped,Node> tree(&model, model.node_begin(), model.node_end());

    // Define a polyhedron from a bounding box
    upsp::BoundingBox_<cv::Point3f> bb({-2,-1,-1},{0,0,0});
    upsp::Polyhedron<6,float> poly  = upsp::convert(bb);

    // Get all nodes within the bounding box
    // should be 0,1,5
    std::set<node_idx> nodes = upsp::nodes_within(poly, tree);
    EXPECT_EQ(nodes.size(), 3);
    EXPECT_TRUE(nodes.find(0) != nodes.end());
    EXPECT_TRUE(nodes.find(1) != nodes.end());
    EXPECT_TRUE(nodes.find(5) != nodes.end());

    // Create a polyhedron fully outside of the octree
    upsp::BoundingBox_<cv::Point3f> bb2({2,2,2},{4,4,4});
    upsp::Polyhedron<6,float> poly2 = upsp::convert(bb2);

    nodes.clear();
    nodes = upsp::nodes_within(poly2, tree);
    EXPECT_EQ(nodes.size(), 0);

    // Create a polyhedron inside of Octree, but with 0 nodes
    upsp::BoundingBox_<cv::Point3f> bb3({-2,-3,-1},{-1.5,-2,1});
    upsp::Polyhedron<6,float> poly3 = upsp::convert(bb3);

    nodes.clear();
    nodes = upsp::nodes_within(poly3, tree);
    EXPECT_EQ(nodes.size(), 0);
}

/** Test nodes_within(Polyhedron, Octree) */
TEST_F(TriModelTestClass, NodesWithin) {

    typedef typename upsp::Octree<Wrapped,Node>::Node tree_node;

    // put model into an octree
    upsp::Octree<Wrapped,Node> tree(&model, model.node_begin(), model.node_end());

    // Define a polyhedron from a bounding box (contains entire sphere)
    upsp::BoundingBox_<cv::Point3f> bb({-10,-10,-10},{10,10,10});
    upsp::Polyhedron<6,float> poly  = upsp::convert(bb);

    // Should contain all nodes
    std::set<node_idx> nodes = upsp::nodes_within(poly, tree);
    EXPECT_EQ(nodes.size(), model.size());

    // Find boundary of octree and define polyhedron on one of those edges
    tree_node* root = tree.get_root();
    upsp::BoundingBox_<cv::Point3f> root_bb = root->get_bounding_box();
    upsp::BoundingBox_<cv::Point3f> bb1(root_bb);
    upsp::BoundingBox_<cv::Point3f> bb2(root_bb);
    float z = bb1.bounds[0].z + 0.5*(bb1.bounds[1].z - bb1.bounds[0].z);
    bb1.bounds[1].z = z;
    bb2.bounds[0].z = z;

    upsp::Polyhedron<6,float> poly1 = upsp::convert(bb1);
    upsp::Polyhedron<6,float> poly2 = upsp::convert(bb2);

    std::set<node_idx> nodes1 = upsp::nodes_within(poly1, tree);
    std::set<node_idx> nodes2 = upsp::nodes_within(poly2, tree);
    nodes1.insert(nodes2.begin(), nodes2.end());
    EXPECT_EQ(nodes1.size(), model.size());

    // Define polyhedron inside of the sphere
    upsp::BoundingBox_<cv::Point3f> bb3({-5,-5,-5},{5,5,5});
    upsp::Polyhedron<6,float> poly3 = upsp::convert(bb3);
    std::set<node_idx> nodes3 = upsp::nodes_within(poly3, tree);
    EXPECT_EQ(nodes3.size(), 0);

}

/** Test intersect(pl, pt1, pt2) */
TEST(ModelHTest, IntersectPlPtPt) {

    typedef upsp::Plane<float> Plane;

    // Case where it intersects at the first or last point
    cv::Point3f pt1(0,0,0);
    cv::Point3f pt2(4,0,0);
    Plane pl1({1,0,0},{0,0,0});
    Plane pl2({1,0,0},{4,0,0});

    cv::Point3f inter = upsp::intersect(pl1,pt1,pt2);
    EXPECT_FLOAT_EQ(inter.x, 0.0);
    EXPECT_FLOAT_EQ(inter.y, 0.0);
    EXPECT_FLOAT_EQ(inter.z, 0.0);

    inter = upsp::intersect(pl2,pt1,pt2);
    EXPECT_FLOAT_EQ(inter.x, 4.0);
    EXPECT_FLOAT_EQ(inter.y, 0.0);
    EXPECT_FLOAT_EQ(inter.z, 0.0);

    // General Case
    cv::Point3f pt3(1,1,1);
    Plane pl3({1,1,1},{0.5,0.5,0.5});

    inter = upsp::intersect(pl3,pt1,pt3);
    EXPECT_FLOAT_EQ(inter.x, 0.5);
    EXPECT_FLOAT_EQ(inter.y, 0.5);
    EXPECT_FLOAT_EQ(inter.z, 0.5);
    inter = upsp::intersect(pl3,pt3,pt1);
    EXPECT_FLOAT_EQ(inter.x, 0.5);
    EXPECT_FLOAT_EQ(inter.y, 0.5);
    EXPECT_FLOAT_EQ(inter.z, 0.5);

    Plane pl4({1,1,1},{0.2,0.3,0.4});

    inter = upsp::intersect(pl4,pt1,pt3);
    EXPECT_FLOAT_EQ(inter.x, 0.3);
    EXPECT_FLOAT_EQ(inter.y, 0.3);
    EXPECT_FLOAT_EQ(inter.z, 0.3);
}

/** Test intersect_dist(pl, pt1, pt2) */
TEST(ModelHTest, IntersectDistPl) {

    typedef upsp::Plane<float> Plane;

    // Case where it intersects at the first or last point
    cv::Point3f pt1(0,0,0);
    cv::Point3f pt2(4,0,0);
    Plane pl1({1,0,0},{0,0,0});
    Plane pl2({1,0,0},{4,0,0});

    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl1,pt1,pt2), 0.0);
    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl1,pt2,pt1), 1.0);

    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl2,pt1,pt2), 1.0);
    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl2,pt2,pt1), 0.0);

    // General Case
    cv::Point3f pt3(1,1,1);
    Plane pl3({1,1,1},{0.5,0.5,0.5});

    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl3,pt1,pt3), 0.5);
    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl3,pt3,pt1), 0.5);

    Plane pl4({1,1,1},{0.2,0.3,0.4});

    EXPECT_FLOAT_EQ(upsp::intersect_dist(pl4,pt1,pt3), 0.3);
}

/** Test intersects(Plane,Plane,Plane) */
TEST(ModelHTest, Intersects3Planes) {

    typedef upsp::Plane<float> Plane;

    // Case where planes intersect in point
    Plane pl1({1,0,0},{1,1,1});
    Plane pl2({0,1,0},{2,4,5});
    Plane pl3({0,0,1},{-1,-2,0});

    EXPECT_TRUE(upsp::intersects(pl1,pl2,pl3));

    // Case where 2 planes are parallel
    Plane pl4({1,0,0}, {0,0,0});

    EXPECT_FALSE(upsp::intersects(pl1,pl2,pl4));

    // Case where each plane intersects each other plane in a line
    // no central point
    Plane pl5({1,1,0}, {0,0,0});

    EXPECT_FALSE(upsp::intersects(pl1,pl2,pl5));

    // Case where 3 planes are parallel
    Plane pl6({1,0,0}, {-1,-1,-1});
    
    EXPECT_FALSE(upsp::intersects(pl1,pl4,pl6));

    // Case where 3 planes are conincident
    EXPECT_FALSE(upsp::intersects(pl1,pl1,pl1));

    // Case where 2 planes are coincident
    EXPECT_FALSE(upsp::intersects(pl1,pl1, pl2));

    // Case where 3 planes intersect in a line
    Plane pl7({0,1,0}, {0,0,0});

    EXPECT_FALSE(upsp::intersects(pl4,pl5,pl7));
}

/** Test intersect(Plane,Plane,Plane */
TEST(ModelHTest, Intersect3Planes) {

    typedef upsp::Plane<float> Plane;

    // Test 1
    Plane pl1({1,0,0},{0,0,0});
    Plane pl2({0,1,0},{0,0,0});
    Plane pl3({0,0,1},{0,0,0});

    ASSERT_TRUE(upsp::intersects(pl1,pl2,pl3));
    cv::Point3f inter = upsp::intersect(pl1,pl2,pl3);
    EXPECT_FLOAT_EQ(inter.x, 0.0);
    EXPECT_FLOAT_EQ(inter.y, 0.0);
    EXPECT_FLOAT_EQ(inter.z, 0.0);

    // Test 2
    Plane pl4({-1,-1,1},{0,0,0});

    ASSERT_TRUE(upsp::intersects(pl1,pl2,pl4));
    inter = upsp::intersect(pl1,pl2,pl4);
    EXPECT_FLOAT_EQ(inter.x, 0.0);
    EXPECT_FLOAT_EQ(inter.y, 0.0);
    EXPECT_FLOAT_EQ(inter.z, 0.0);

    // Test 3
    Plane pl5({1,0,1},{1,2,3});
    Plane pl6({1,1,0},{1,2,3});
    Plane pl7({0,1,1},{1,2,3});

    ASSERT_TRUE(upsp::intersects(pl5,pl6,pl7));
    inter = upsp::intersect(pl5,pl6,pl7);
    EXPECT_FLOAT_EQ(inter.x, 1.0);
    EXPECT_FLOAT_EQ(inter.y, 2.0);
    EXPECT_FLOAT_EQ(inter.z, 3.0);

    // Test 4
    Plane pl8({2,-3,4},{1,2,5.0/4.0});
    Plane pl9({1,-1,-1},{0,3,-2});
    Plane pl10({-1,2,-1},{5,-4,-11});

    ASSERT_TRUE(upsp::intersects(pl8,pl9,pl10));
    inter = upsp::intersect(pl8,pl9,pl10);
    EXPECT_FLOAT_EQ(inter.x, -4.0);
    EXPECT_FLOAT_EQ(inter.y, -3.0);
    EXPECT_FLOAT_EQ(inter.z, 0.0);
}

/** Test split(Triangle, Plane) , cut through 1 node*/
TEST_F(SimpleTriModelTestClass, ModelHSplitTriangle01Edge) {

    typedef upsp::Plane<float> Plane;
    typedef upsp::Triangle<float> Tri;
    typedef upsp::SplitTri<float> SplitTri;

    // Test case where triangle is not split by the plane
    Tri tri1 = get_triangle(1);
    Plane pl1({1,0,0},{1,0,0});

    std::vector<SplitTri> new_tris = upsp::split(tri1, pl1);

    EXPECT_EQ(new_tris.size(), 0);
    new_tris.clear();

    // Intersect through a node in the triangle
    // check that normals are consistent
    // check that area is consistent
    // check that node mapping information is correct
    // could loop these three cases, but it is easier to debug laid out
    // like this

    // Test plane through first position (0,1,2) -> through 0
    Plane pl2({1,0,0},{0,0,0});

    new_tris = upsp::split(tri1, pl2);

    EXPECT_EQ(new_tris.size(), 2);
    float area = upsp::area(static_cast<Tri&>(tri1));
    float new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,1) > 0.0) && (stri.mapping(i,2) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,1), 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].x, 0.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-1.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri1.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri1.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri1.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
        }
        EXPECT_TRUE(found_new_map);
    }
    EXPECT_FLOAT_EQ(new_area, area);
    
    // Test plane through second position (5,1,0) -> through 1
    Plane pl3({-1,0,0},{-1,0,0});
    Tri tri0 = get_triangle(0);

    new_tris.clear();
    new_tris = upsp::split(tri0, pl3);

    EXPECT_EQ(new_tris.size(), 2);
    area = upsp::area(static_cast<Tri&>(tri0));
    new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,2) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,0), 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].x,-1.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].y, 0.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri0.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri0.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri0.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
                
        }
        EXPECT_TRUE(found_new_map);
    }
    EXPECT_FLOAT_EQ(new_area, area);

    // Test plane through third position (2,1,3) -> through 3
    Tri tri2 = get_triangle(2);

    new_tris.clear();
    new_tris = upsp::split(tri2, pl2);

    EXPECT_EQ(new_tris.size(), 2);
    area = upsp::area(static_cast<Tri&>(tri2));
    new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,1) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,0), 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].x, 0.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-1.0);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri2.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri2.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri2.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
        }
        EXPECT_TRUE(found_new_map);
    }
    EXPECT_FLOAT_EQ(new_area, area);
}

/** Test split(Triangle, Plane) , cut through 2 edges */
TEST_F(SimpleTriModelTestClass, ModelHSplitTriangle2Edges) {

    typedef upsp::Plane<float> Plane;
    typedef upsp::Triangle<float> Tri;
    typedef upsp::SplitTri<float> SplitTri;

    // Define epsilon for area equality
    float epsilon = 0.00001;

    // Odd node out is first position (0,1,2) -> left 0
    Tri tri1 = get_triangle(1);
    Plane pl1({0,1,0},{0,-0.4,0});

    std::vector<SplitTri> new_tris = upsp::split(tri1, pl1);
    
    EXPECT_EQ(new_tris.size(), 3);
    float area = upsp::area(static_cast<Tri&>(tri1));
    float new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map1 = false;
        bool found_new_map2 = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,1) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,0), 0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].x,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map1 = true;
            } else if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,2) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,0), 0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].x, 0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map2 = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri1.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri1.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri1.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
        }
        EXPECT_TRUE(found_new_map1 || found_new_map2);
    }
    EXPECT_TRUE(abs(new_area - area) < epsilon);

    // Odd node out is second position (5,1,0) -> left 1
    Tri tri0 = get_triangle(0);

    new_tris.clear();
    new_tris = upsp::split(tri0, pl1);
    
    EXPECT_EQ(new_tris.size(), 3);
    area = upsp::area(static_cast<Tri&>(tri0));
    new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map1 = false;
        bool found_new_map2 = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,1) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,1), 0.6);
                EXPECT_FLOAT_EQ(stri.nodes[i].x,-1.6);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map1 = true;
            } else if ( (stri.mapping(i,1) > 0.0) && (stri.mapping(i,2) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,1), 0.6);
                EXPECT_FLOAT_EQ(stri.nodes[i].x,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-0.4);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map2 = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri0.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri0.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri0.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
        }
        EXPECT_TRUE(found_new_map1 || found_new_map2);
    }
    EXPECT_FLOAT_EQ(new_area, area);

    // Odd node out is third position (2,1,3) -> left 3
    Tri tri2 = get_triangle(2);
    Plane pl2({0,1,0},{0,-1.5,0});

    new_tris.clear();
    new_tris = upsp::split(tri2, pl2);
    
    EXPECT_EQ(new_tris.size(), 3);
    area = upsp::area(static_cast<Tri&>(tri2));
    new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));

        // test the mapping information
        bool found_new_map1 = false;
        bool found_new_map2 = false;
        for (unsigned int i=0; i < 3; ++i) {
            if ( (stri.mapping(i,2) > 0.0) && (stri.mapping(i,1) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,2), 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].x,-0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-1.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map1 = true;
            } else if ( (stri.mapping(i,0) > 0.0) && (stri.mapping(i,2) > 0.0) ) {
                EXPECT_FLOAT_EQ(stri.mapping(i,2), 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].x, 0.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].y,-1.5);
                EXPECT_FLOAT_EQ(stri.nodes[i].z, 0.0);
                found_new_map2 = true;
            } else {
                // it is a direct mapping of an old node to a new node
                bool found_direct_map = false;
                for (unsigned int j=0; j < 3; ++j) {
                    if (stri.mapping(i,j) == 1.0) {
                        EXPECT_FLOAT_EQ(stri.nodes[i].x, tri2.nodes[j].x);
                        EXPECT_FLOAT_EQ(stri.nodes[i].y, tri2.nodes[j].y);
                        EXPECT_FLOAT_EQ(stri.nodes[i].z, tri2.nodes[j].z);
                        found_direct_map = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_direct_map);
            }
        }
        EXPECT_TRUE(found_new_map1 || found_new_map2);
    }
    EXPECT_FLOAT_EQ(new_area, area);

}

/** Test shortest_distance(Polyhedron,pt) */
TEST(ModelHTest, ShortestDistancePolyPt) {

    upsp::BoundingBox_<cv::Point3f> bb({-1,-1,-1},{1,1,1});
    upsp::Polyhedron<6,float> poly = upsp::convert(bb);

    // Test on 3 planes
    float dist = upsp::shortest_distance(poly, cv::Point3f(-1,-1,-1));
    EXPECT_FLOAT_EQ(dist, 0.0);

    // Test on 2 planes
    dist = upsp::shortest_distance(poly, cv::Point3f(-1,-1,0));
    EXPECT_FLOAT_EQ(dist, 0.0);

    // Test on 1 plane
    dist = upsp::shortest_distance(poly, cv::Point3f(0,0,1));
    EXPECT_FLOAT_EQ(dist, 0.0);

    // Test within poly
    dist = upsp::shortest_distance(poly, cv::Point3f(0,0,0));
    EXPECT_FLOAT_EQ(dist, 1.0);

    dist = upsp::shortest_distance(poly, cv::Point3f(-0.5,0,0));
    EXPECT_FLOAT_EQ(dist, 0.5);

}

// Test intersects(tri, poly) */
TEST(ModelHTest, IntersectsTriPoly) {

    typedef upsp::Triangle<float> Tri;

    upsp::BoundingBox_<cv::Point3f> bb({-1,-1,-1},{1,1,1});
    upsp::Polyhedron<6,float> poly = upsp::convert(bb);
    
    // Define points
    cv::Point3f  pt1(-1  ,-1  ,-1);
    cv::Point3f  pt2( 0  , 0  , 0);
    cv::Point3f  pt3(-0.5, 0.5, 0);
    cv::Point3f  pt4(-0.5,-0.5, 0);
    cv::Point3f  pt5(-1  , 0  , 0);
    cv::Point3f  pt6(-2  ,-1  ,-1);
    cv::Point3f  pt7(-2  , 0.5, 0);
    cv::Point3f  pt8(-0.5, 1.5, 0);
    cv::Point3f  pt9(-0.5,-2  , 0.5);
    cv::Point3f pt10(-0.5, 1  , 0);
    cv::Point3f pt11(-1.5, 0  , 0);
    cv::Point3f pt12( 0  ,-1.5, 1);
    cv::Point3f pt13(-0.5,-1.0, 0);
    cv::Point3f pt14( 1.0, 1.0, 0);
    cv::Point3f pt15( 1.5, 0.5, 0);
    cv::Point3f pt16(-3.5, 0.0, -1);
    cv::Point3f pt17( 2.5, 3.5, 1.5);
    cv::Point3f pt18( 2.5,-2.5, 1.5);
    cv::Point3f pt19( 1.0,-2.5, 0);
    cv::Point3f pt20( 1.0, 3.0, 0);
    cv::Point3f pt21(-3.5, 0.0, 1.5);
    cv::Point3f pt22(-1.5, 1.5, 0);
    cv::Point3f pt23(-1.5, 0.8,-0.2);
    cv::Point3f pt24(-0.8, 1.5,-0.2);

    /********************************************
     * All nodes clearly inside or outside      *
     *******************************************/

    // all nodes in poly
    Tri tri(pt2,pt3,pt4);
    EXPECT_FALSE(upsp::intersects(tri,poly));

    // 1 node in poly, 2 outside
    tri = Tri(pt3,pt8,pt7);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    // 2 nodes in poly, 1 outside
    tri = Tri(pt3,pt2,pt6);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    // 3 nodes outside
    // edge intersection
    tri = Tri(pt11,pt6,pt12);
    EXPECT_TRUE(upsp::intersects(tri, poly));
    // triangle "contains" poly
    tri = Tri(pt16,pt17,pt18);
    EXPECT_TRUE(upsp::intersects(tri, poly));
    tri = Tri(pt16, pt19, pt20);
    EXPECT_TRUE(upsp::intersects(tri, poly));
    tri = Tri(pt20, pt18, pt16);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    // 3 nodes outside
    // no edge intersection
    tri = Tri(pt11,pt7,pt6);
    EXPECT_FALSE(upsp::intersects(tri, poly));
    // triangle is above poly
    tri = Tri(pt21, pt17, pt18);
    EXPECT_FALSE(upsp::intersects(tri, poly));
    // 3D effect
    tri = Tri(pt22, pt23, pt24);
    EXPECT_FALSE(upsp::intersects(tri, poly));


    //tri = Tri(pt11,pt9,pt6); // this test may fail due to numerical error
                             // but the consequence would just be a few
                             // extra triangles created (shouldn't impact integration)
    //EXPECT_FALSE(upsp::intersects(tri, poly));

    /********************************************
     * 1 node on the polyhedron                 *
     *******************************************/

    // no edge intersection
    tri = Tri(pt5,pt7,pt6);
    EXPECT_FALSE(upsp::intersects(tri,poly));

    tri = Tri(pt13,pt9,pt6);
    EXPECT_FALSE(upsp::intersects(tri, poly));

    // edge intersection
    tri = Tri(pt5,pt8, pt7);
    EXPECT_TRUE(upsp::intersects(tri, poly)); 

    tri = Tri(pt11,pt1,pt12);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    tri = Tri(pt1,pt9,pt6); // pt1 lies on two planes
    EXPECT_FALSE(upsp::intersects(tri, poly));

    /********************************************
     * 2 nodes on the polyhedron                 *
     *******************************************/

    // no edge intersection
    tri = Tri(pt1,pt5,pt7);
    EXPECT_FALSE(upsp::intersects(tri, poly));

    tri = Tri(pt1,pt13,pt9);
    EXPECT_FALSE(upsp::intersects(tri, poly));

    // edge intersection
    tri = Tri(pt7,pt10,pt5);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    tri = Tri(pt5,pt13,pt6);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    tri = Tri(pt14,pt11,pt1);
    EXPECT_TRUE(upsp::intersects(tri, poly));

    tri = Tri(pt1,pt5,pt15);
    EXPECT_TRUE(upsp::intersects(tri, poly));

}

/** Test split(Triangle, Polyhedron) */
TEST_F(SimpleTriModelTestClass, ModelHSplitTrianglePoly) {

    typedef upsp::Plane<float> Plane;
    typedef upsp::Triangle<float> Tri;
    typedef upsp::SplitTri<float> SplitTri;

    // set epsilon for area comparison
    float epsilon = 0.00001;

    // Test case where polyhedron does not split a triangle
    // triangle is inside of polyhedron
    upsp::BoundingBox_<cv::Point3f> bb1({-10,-10,-10},{10,10,10});
    upsp::Polyhedron<6,float> poly = upsp::convert(bb1);

    Tri tri = get_triangle(1);
    std::vector<SplitTri> new_tris = upsp::split(tri, poly);

    EXPECT_EQ(new_tris.size(), 0);

    // triangle is entirely separate from polyhedron
    upsp::BoundingBox_<cv::Point3f> bb2({2,2,2},{10,10,10});
    poly = upsp::convert(bb2);

    new_tris.clear();
    new_tris = upsp::split(tri, poly);

    EXPECT_EQ(new_tris.size(), 0);

    // 1 triangle node is on boundary of polyhedron
    // no intersection
    upsp::BoundingBox_<cv::Point3f> bb3({1,-1,-1},{10,10,10});
    poly = upsp::convert(bb3);

    new_tris.clear();
    new_tris = upsp::split(tri, poly);

    EXPECT_EQ(new_tris.size(), 0);

    // 2 triangle nodes are on boundary of polyhedron
    // no intersection
    upsp::BoundingBox_<cv::Point3f> bb4({-2,0,-1},{10,10,10});
    poly = upsp::convert(bb4);

    tri = get_triangle(0);

    new_tris.clear();
    new_tris = upsp::split(tri, poly);

    EXPECT_EQ(new_tris.size(), 0);
    
    // 1 polyhedron plane splits triangle
    upsp::BoundingBox_<cv::Point3f> bb5({-2,-0.4,-1},{10,10,10});
    poly = upsp::convert(bb5);

    new_tris.clear();
    new_tris = upsp::split(tri, poly);

    EXPECT_EQ(new_tris.size(), 3);
    
    float area = upsp::area(static_cast<Tri&>(tri));
    float new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));
    }
    EXPECT_TRUE(abs(new_area - area) < epsilon);

    // 2 polyhedron planes split triangle
    upsp::BoundingBox_<cv::Point3f> bb6({-1,-0.4,-1},{10,10,10});
    poly = upsp::convert(bb6);

    new_tris.clear();
    new_tris = upsp::split(tri, poly);

    EXPECT_TRUE(new_tris.size() >= 4);

    area = upsp::area(static_cast<Tri&>(tri));
    new_area = 0.0;
    for (auto it = new_tris.begin(); it != new_tris.end(); ++it) {
        SplitTri stri = *it;

        cv::Point3f n = upsp::normal(static_cast<Tri&>(stri));
        EXPECT_FLOAT_EQ(n.x, 0.0);
        EXPECT_FLOAT_EQ(n.y, 0.0);
        EXPECT_FLOAT_EQ(n.z, 1.0);
        new_area += upsp::area(static_cast<Tri&>(stri));
    }
    EXPECT_TRUE(abs(new_area - area) < epsilon);

}
