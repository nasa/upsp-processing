/** @file
 *  @brief  Simple Data Structure Definitions
 *  @date   Aug 1, 2017
 *  @author jmpowel2
 */

#ifndef UFML_DATA_STRUCTS_H_
#define UFML_DATA_STRUCTS_H_

#include <algorithm>
#include <array>
#include <Eigen/Dense>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <set>
#include <utility>
#include <vector>

#include "./utils/general_utils.h"
#include "grids.h"

/** @namespace upsp
 *  @brief  Unsteady PSP 
 */
namespace upsp {

/* Forward Declare types */
class CameraCal;
template<typename FP> class P3DModel_;
template<typename FP> class TriModel_;
template<typename Model, typename Elem> class Octree;

/* Handle structured/unstructured */
template<typename T>
struct is_structured : std::false_type {};
template<typename FP>
struct is_structured<P3DModel_<FP>> : std::true_type {};
template<typename FP>
struct is_structured<TriModel_<FP>> : std::false_type {};
template<typename FP>
struct is_structured<StructuredGrid<FP>> : std::true_type {};
template<typename FP>
struct is_structured<UnstructuredGrid<FP>> : std::false_type {};

/** Image circular target */
template<typename FP>
struct Target_ {
    typedef FP data_type;

    Target_() : xyz(cv::Point3_<FP>(0.0,0.0,0.0)), uv(cv::Point_<FP>(0.0,0.0)), 
            diameter(0.0), num(0) {}

    cv::Point3_<FP> xyz;
    cv::Point_<FP> uv;
    FP diameter;
    int num;
};
typedef Target_<float> Target;

/** Write out Target parameters */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const Target_<FP>& targ) {
    os << "Target " << targ.num << std::endl;
    os << "       xyz = " << targ.xyz << std::endl;
    os << "        uv = " << targ.uv << std::endl;
    os << "      diam = " << targ.diameter << std::endl;
    return os;
}

/** Image Circular Target with extra parameters for defining value */
template<typename FP>
struct Kulite_ : Target_<FP> {
    typedef FP data_type;   

    Kulite_() {}
    Kulite_(const Target_<FP> &targ) : Target_<FP>(targ) {}

    cv::Point2i top_left;
    int size;
    FP value;
};
typedef Kulite_<float> Kulite;

/** Functor to convert Target to just its 3D position */
template<typename FP>
struct Target2Point3 : std::unary_function<Target_<FP>, cv::Point3_<FP>> {
    cv::Point3_<FP> operator()(Target_<FP> targ) const {
        return targ.xyz;
    }
};

/** Functor to convert a Target to its 2D position in an image */
template<typename FP>
struct Target2Point2 : std::unary_function<Target_<FP>, cv::Point_<FP>> {
    cv::Point_<FP> operator()(Target_<FP> targ) const {
        return targ.uv;
    }
};

template<typename FP>
struct Kulite2Point3 : std::unary_function<Kulite_<FP>, cv::Point3_<FP>> {
    cv::Point3_<FP> operator()(Kulite_<FP> kul) const {
        return kul.xyz;
    }
};

template<typename FP>
struct Kulite2Point2 : std::unary_function<Kulite_<FP>, cv::Point_<FP>> {
    cv::Point_<FP> operator()(Kulite_<FP> kul) const {
        return kul.uv;
    }
};

template<typename FP>
struct Point32Kulite : std::unary_function<cv::Point3_<FP>, Kulite_<FP>> {
    Kulite_<FP> operator()(cv::Point3_<FP> pt) const {
        Kulite_<FP> kul;
        kul.xyz = pt;
        return kul;
    }
};

template<typename Pred, typename T>
struct PredPoint3 : std::unary_function<T, bool> {
    PredPoint3(const Pred& p) : p_(p) {}

    bool operator()(const T& t) const {
        return p_(t.xyz);
    }

    const Pred& p_;
};

template<typename Pred>
PredPoint3<Pred, Target> convert_predicate_point3_target(const Pred& p) {
    return PredPoint3<Pred, Target>(p);
}
template<typename Pred>
PredPoint3<Pred, Kulite> convert_predicate_point3_kulite(const Pred& p) {
    return PredPoint3<Pred, Kulite>(p);
}

template<typename FP>
struct IdentityTransform : std::unary_function<cv::Point3_<FP>, cv::Point3_<FP>> {
    cv::Point3_<FP> operator()(const cv::Point3_<FP>& pt) const {
        return pt;
    }
};

/** Represents an axis-aligned rectangle or rectangular prism */
template<typename Pt>
struct BoundingBox_ {
    BoundingBox_() {}

    BoundingBox_(Pt min, Pt max) : bounds {min,max}  {}
    Pt bounds[2];
};

/* Specialize 2d and 3d default constructors for zero bounds */
template<> inline BoundingBox_<cv::Point3d>::BoundingBox_() : bounds 
        {cv::Point3d(0.0,0.0,0.0),cv::Point3d(0.0,0.0,0.0)} {}
template<> inline BoundingBox_<cv::Point2d>::BoundingBox_() : bounds 
        {cv::Point2d(0.0,0.0),cv::Point2d(0.0,0.0)} {}
template<> inline BoundingBox_<cv::Point3f>::BoundingBox_() : bounds 
        {cv::Point3f(0.0,0.0,0.0),cv::Point3f(0.0,0.0,0.0)} {}
template<> inline BoundingBox_<cv::Point2f>::BoundingBox_() : bounds 
        {cv::Point2f(0.0,0.0),cv::Point2f(0.0,0.0)} {}

typedef BoundingBox_<cv::Point3d> BoundingBox3D;
typedef BoundingBox3D BoundingBox;
typedef BoundingBox_<cv::Point2d> BoundingBox2D;

/** Write out a 3D bounding box */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const BoundingBox_<cv::Point3_<FP>>& bb) {
    os << "x in [" << bb.bounds[0].x << ", " << bb.bounds[1].x << "), ";
    os << "y in [" << bb.bounds[0].y << ", " << bb.bounds[1].y << "), ";
    os << "z in [" << bb.bounds[0].z << ", " << bb.bounds[1].z << ")";
    return os;
}    

/** Write out a 2D bounding box */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const BoundingBox_<cv::Point_<FP>>& bb) {
    os << "x in [" << bb.bounds[0].x << ", " << bb.bounds[1].x << "), ";
    os << "y in [" << bb.bounds[0].y << ", " << bb.bounds[1].y << ")";
    return os;
}    

/** Hold a 3D ray */
template<typename FP>
struct Ray {
    Ray(const cv::Point3_<FP> &origin_in, const cv::Point3_<FP> &dir_in) : 
            origin(origin_in), dir(dir_in) {
        invdir.x = 1.0 / dir.x;
        invdir.y = 1.0 / dir.y;
        invdir.z = 1.0 / dir.z;
        sign[0] = (invdir.x < 0);
        sign[1] = (invdir.y < 0);
        sign[2] = (invdir.z < 0);
    }

    cv::Point3_<FP> origin, dir, invdir;
    int sign[3];
};

/** Write out a ray */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const Ray<FP>& r) {
    os << "origin = <" << r.origin.x << ", " << r.origin.y << ", " << r.origin.z;
    os << "> | direction = <" << r.dir.x << ", " << r.dir.y << ", " << r.dir.z;
    os << ">";
    return os;
}    

/** Holds a simple polygon */
template<typename FP>
struct Polygon {
    Polygon() {}
    Polygon(std::initializer_list<FP> il) {
        std::copy(il.begin(), il.end(), std::back_inserter(nodes));
    }
    template<typename InputIterator>
    Polygon(InputIterator begin, InputIterator end) {
        std::copy(begin, end, std::back_inserter(nodes));
    }
    std::vector<cv::Point3_<FP>> nodes;
};

/** Write out a polygon */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const Polygon<FP>& p) {
    if (p.nodes.size() == 0) {
        os << "empty polygon" << std::endl;
    } else {
        os << "nodes = ";
        for (unsigned int i=0; i < p.nodes.size(); ++i) {
            os << "<" << p.nodes[i].x << ", " << p.nodes[i].y << ", " << p.nodes[i].z;
            os << ">";
            if (i < (p.nodes.size()-1)) {
                os << ", ";
            }
        }
    }
    return os;
}    

/** Holds a simple triangle */
template<typename FP>
struct Triangle {
    Triangle(cv::Point3_<FP> n1, cv::Point3_<FP> n2, cv::Point3_<FP> n3) : nodes {n1,n2,n3} {} 
    std::array<cv::Point3_<FP>,3> nodes;
};

/** Write out a triangle */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const Triangle<FP>& t) {
    os << "nodes = <" << t.nodes[0].x << ", " << t.nodes[0].y << ", " << t.nodes[0].z;
    os << ">, <" << t.nodes[1].x << ", " << t.nodes[1].y << ", " << t.nodes[1].z;
    os << ">, <" << t.nodes[2].x << ", " << t.nodes[2].y << ", " << t.nodes[2].z;
    os << ">";
    return os;
}    

/** Holds a triangle with information about how each node maps to 
 * the original triangle */
template<typename FP>
struct SplitTri : public Triangle<FP> {

    /** Initialize with node locations
     *  mapping is initialized to zeros 
     */
    SplitTri(cv::Point3_<FP> n1, cv::Point3_<FP> n2, cv::Point3_<FP> n3) : 
            Triangle<FP>(n1,n2,n3), mapping(Eigen::Matrix<FP,3,3>::Zero()) {}

    /** Initialize with existing triangle
     *  mapping is initialized to identity
     */
    SplitTri(const Triangle<FP>& tri) : Triangle<FP>(tri), 
            mapping(Eigen::Matrix<FP,3,3>::Identity()) {}

    Eigen::Matrix<FP, 3, 3> mapping; // mapping[0,1] = fraction of old node1 
                                     //                contained in new node0
};

/** Holds a plane */
template<typename FP>
struct Plane {
    typedef FP data_type;

    Plane() {}

    Plane(const cv::Point3_<FP>& n, const cv::Point3_<FP>& pt) : 
            n {n.x,n.y,n.z}, pt {pt.x,pt.y,pt.z} {}

    Plane(const Triangle<FP>& tri) {
        cv::Point3_<FP> u1 = tri.nodes[1] - tri.nodes[0];
        cv::Point3_<FP> u2 = tri.nodes[2] - tri.nodes[0];
        cv::Point3_<FP> n_dir = u1.cross(u2);
        n_dir = n_dir / cv::norm(n_dir);
        n[0] = n_dir.x;
        n[1] = n_dir.y;
        n[2] = n_dir.z;
        pt[0] = tri.nodes[0].x;
        pt[1] = tri.nodes[0].y;
        pt[2] = tri.nodes[0].z;
    }

    std::array<FP,3> n; // normal direction
    std::array<FP,3> pt; // origin 
};

/** Write out a plane */
template<typename FP>
std::ostream& operator<<(std::ostream& os, const Plane<FP>& pl) {
    os << "normal = <" << pl.n[0] << ", " << pl.n[1] << ", " << pl.n[2];
    os << "> | anchor pt = <" << pl.pt[0] << ", " << pl.pt[1] << ", " << pl.pt[2];
    os << ">";
    return os;
}    

/** Holds a general convex polyhedron */
template<size_t Size, typename FP>
struct Polyhedron {
    typedef FP data_type;

    Polyhedron() : tol(0.001) {}

    // define a partial polyhedron from a triangle
    // walls for each triangle edge
    Polyhedron(const Triangle<FP>& tri) {
        cv::Point3_<FP> u1 = tri.nodes[1] - tri.nodes[0];
        u1 = u1 / cv::norm(u1);
        cv::Point3_<FP> u2 = tri.nodes[2] - tri.nodes[0];
        u2 = u2 / cv::norm(u2);
        cv::Point3_<FP> u3 = tri.nodes[2] - tri.nodes[1];
        u3 = u3 / cv::norm(u3);

        cv::Point3_<FP> n_dir = u1.cross(u2);

        cv::Point3_<FP> n1 = u1.cross(n_dir);
        cv::Point3_<FP> n2 = u2.cross(-n_dir);
        cv::Point3_<FP> n3 = u3.cross(n_dir);

        planes[0] = Plane<FP>(n1, tri.nodes[0]);
        planes[1] = Plane<FP>(n2, tri.nodes[0]);
        planes[2] = Plane<FP>(n3, tri.nodes[1]);

        edges.push_back(u1);
        edges.push_back(u2);
        edges.push_back(u3);
    }

    // store planes, normal points outside of polyhedron
    std::array<Plane<FP>,Size> planes;
    
    // also store vertices
    std::vector<cv::Point3_<FP>> vertices;

    // can store a triangulation of the polyhedron
    std::vector<Triangle<FP>> tris;

    // can store the unique edges of the polyhedron
    // for use in SAT (just need an axis not a line segment)
    std::vector<cv::Point3_<FP>> edges;

    FP tol; // maximum distance from vertex to plane
};

/** Write out a polyhedron */
template<size_t S, typename FP>
std::ostream& operator<<(std::ostream& os, const Polyhedron<S,FP>& p) {
    if (p.planes.size() == 0) {
        os << "no planes in polyhedron, ";
    } else {
        os << "planes = ";
        for (unsigned int i=0; i < p.planes.size(); ++i) {
            os << p.planes[i];
            if (i < (p.planes.size()-1)) {
                os << ", ";
            }
        }
    }
    if (p.vertices.size() == 0) {
        os << "no vertices in polyhedron, ";
    } else {
        os << std::endl << "vertices = ";
        for (unsigned int i=0; i < p.vertices.size(); ++i) {
            os << p.vertices[i];
            if (i < p.vertices.size()-1) {
                os << ", ";
            }
        }
    }
    return os;
}    

/** Create a hexahedron from a Bounding Box */
template<typename FP>
Polyhedron<6,FP> convert(const BoundingBox_<cv::Point3_<FP>>& bb);

template<typename Model>
struct Node2_Point : std::unary_function<typename Model::Node, cv::Point3_<typename Model::data_type>> {
    typedef typename Model::Node Node;
    typedef typename Model::data_type FP;
    cv::Point3_<FP> operator()(Node node) const {
        return node.get_position();
    }
};

/** Maps parent triangles to children
 *  used with splitting
 */
template<typename T>
class FaceMap {
public:

    class FaceMapIterator;

    /** Return the number of original triangles that were split */
    size_t size() const { return old2new_.size(); }

    /** Return the number of new triangles from a specific origin triangle
     *
     * @pre has(@a key)
     */
    size_t size(const T& key) const { return old2new_.at(key).size(); }

    /** Remove all splitting data */
    void clear() { old2new_.clear(); new2old_.clear(); }

    /** Add a single split */
    void add(const T& old_f, const T& new_f);

    /** Merge in another splitting 
     *
     * @pre @a o_fmap represents splitting performed after all splitting
     *      currently contained in this
     */
    void merge(const FaceMap<T>& o_fmap);

    /** Check if the Face is already split */
    bool has(const T& key) const {
        if (old2new_.find(key) != old2new_.end()) {
            return true;
        }
        return false;
    }

    /** Check if the Face resulted from a split */
    bool is_new(const T& key) const {
        if (new2old_.find(key) != new2old_.end()) {
            return true;
        }
        return false;
    }

    /** Get the children of the Face 
     *
     * @pre has(@a T)
     */
    const std::set<T>& children(const T& key) const {
        return old2new_.at(key);
    }

    /** Get beginning iterator to split faces */
    FaceMapIterator begin() { return FaceMapIterator(this, true); }

    /** Get ending iterator to split faces */
    FaceMapIterator end() { return FaceMapIterator(this, false); }

    class FaceMapIterator : private equality_comparable<FaceMapIterator> {
    public:
        using value_type        = T;
        using pointer           = T*;
        using reference         = T&;
        using difference_type   = std::ptrdiff_t;
        using iterator_category = std::input_iterator_tag;

        /** Default constructore creates an invalid object */
        FaceMapIterator() : fmp_(nullptr) {};

        /** Dereference the iterator
         * @pre iterator must be valid and < end iterator
         */
        T operator*() const { return it_->first; }

        /** Increment the iterator by 1 */
        FaceMapIterator& operator++() { ++it_; return *this; }

        /** Test for equality */
        bool operator==(const FaceMapIterator& fm_it) const {
            if (fmp_ == fm_it.fmp_) {
                return it_ == fm_it.it_;
            }
            return false;
        }

    private:
        friend class FaceMap;

        /** Create a valid FaceMapIterator 
         */
        FaceMapIterator(const FaceMap* fmp, bool begin=false) : 
               fmp_(const_cast<FaceMap*>(fmp)) {
            if (begin) {
                it_ = fmp_->old2new_.begin();
            } else {
                it_ = fmp_->old2new_.end();
            }
        }

        FaceMap<T>* fmp_;
        typename std::map<T, std::set<T>>::iterator it_;
    };
        

private:
    std::map<T, std::set<T>> old2new_; // second only contains valid faces
    std::unordered_map<T, T> new2old_; // includes all splits, even deprecated
};



} /* end namespace upsp */

#include "../lib/data_structs.ipp"

#endif /* UFML_DATA_STRUCTS_H_ */
