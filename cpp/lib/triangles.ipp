/** @file
 *  @brief  Handles Triangles, triangulation, convex hulls
 *  @date   May 8, 2019
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename FP>
std::vector<Triangle<FP>> triangulate(const Polygon<FP>& poly) {

    std::vector<Triangle<FP>> tris;

    for (unsigned int i=1; i < (poly.nodes.size()-1); ++i) {
        tris.push_back(Triangle<FP>(poly.nodes[0],poly.nodes[i],poly.nodes[i+1]));
    }

    return tris;
}

/*****************************************************************************/
template<typename FP, size_t S>
std::vector<Polygon<FP>> convert_polygons(const Polyhedron<S,FP>& poly) {

    std::vector<Polygon<FP>> polygons;

    cv::Point3_<FP> ref_vec(0,0,1);
    cv::Point3_<FP> ref_pt(0,0,0);

    // create a polygon for each plane
    for (unsigned int i=0; i < S; ++i) {
        // find all vertices on the plane
        std::vector<unsigned int> verts;
        for (unsigned int j=0; j < poly.vertices.size(); ++j) {
            if (std::abs(shortest_distance(poly.planes[i], poly.vertices[j])) < 
                poly.tol) {
                verts.push_back(j);
            }
        }
        assert(verts.size() > 2);

        // get the normal direction in Point
        cv::Point3_<FP> orig_normal(poly.planes[i].n[0], poly.planes[i].n[1],
                                    poly.planes[i].n[2]);

        // define a rotation vector
        // if the vectors are already aligned, then don't rotate
        bool dont_rotate = false;
        cv::Point3_<FP> rot = cross(orig_normal,ref_vec);
        if (cv::norm(rot) == 0.0) {
            dont_rotate = true;
        }
        FP angle = static_cast<FP>(angle_between(orig_normal, ref_vec));

        // Transform points onto a 2D plane
        // opencv convexHull requires 32bit 2D points
        std::vector<cv::Point2f> pts2d(verts.size());
        for (unsigned int j=0; j < verts.size(); ++j) {
            // rotate onto an xy plane
            cv::Point3_<FP> rot_pt;
            if (dont_rotate) {
                rot_pt = poly.vertices[verts[j]];
            } else {
                rot_pt = rotate(rot, ref_pt, angle, poly.vertices[verts[j]]);
            }

            // truncate for new 2D points
            pts2d[j].x = static_cast<float>(rot_pt.x);
            pts2d[j].y = static_cast<float>(rot_pt.y);
        }

        // get the convex hull
        std::vector<int> hull; // convexHull will not accept unsigned int
        cv::convexHull(pts2d, hull, false, false);

        assert(hull.size() > 2);
        // ensure that the normal direction is consistent
        Polygon<FP> polyg;
        for (unsigned int i=0; i < hull.size(); ++i) {
            polyg.nodes.push_back(poly.vertices[verts[hull[i]]]);
        }
        cv::Point3_<FP> norm = normal(polyg);
        if (angle_between(norm, orig_normal) > (deg2_rad(10))) {
            std::reverse(polyg.nodes.begin(), polyg.nodes.end());
        }

        polygons.push_back(polyg);
    }

    return polygons;
}

/*****************************************************************************/
template<typename FP>
BoundingBox_<cv::Point3_<FP>> minimal_bound(const Triangle<FP>& tri) {

    BoundingBox_<cv::Point3_<FP>> bb;
    bb.bounds[0].x = min(tri.nodes[0].x, tri.nodes[1].x, tri.nodes[2].x);
    bb.bounds[0].y = min(tri.nodes[0].y, tri.nodes[1].y, tri.nodes[2].y);
    bb.bounds[0].z = min(tri.nodes[0].z, tri.nodes[1].z, tri.nodes[2].z);
    bb.bounds[1].x = max(tri.nodes[0].x, tri.nodes[1].x, tri.nodes[2].x);
    bb.bounds[1].y = max(tri.nodes[0].y, tri.nodes[1].y, tri.nodes[2].y);
    bb.bounds[1].z = max(tri.nodes[0].z, tri.nodes[1].z, tri.nodes[2].z);

    // bb is [), so need to push out upper bound
    bb.bounds[1].x = std::nextafter(bb.bounds[1].x, std::numeric_limits<FP>::max());
    bb.bounds[1].y = std::nextafter(bb.bounds[1].y, std::numeric_limits<FP>::max());
    bb.bounds[1].z = std::nextafter(bb.bounds[1].z, std::numeric_limits<FP>::max());

    return bb;
}

} /* end namespace upsp */
