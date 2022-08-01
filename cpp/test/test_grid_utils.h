#ifndef TEST_GRID_UTILS_H_
#define TEST_GRID_UTILS_H_
#include <string>
#include "gtest/gtest.h"
#include "../include/grids.h"
#include "../include/P3DModel.h"
#include "../include/TriModel.h"
#include "../include/integration.h"

/** Compare two binary files for equality
 */
bool compare_files(const std::string& file1, const std::string& file2);

void create_single_structgrid(upsp::StructuredGrid<float>& grid);
void create_simple_unstructgrid(upsp::UnstructuredGrid<float>& ugrid);

/** Initializes a simple Unstructured Grid */
class SimpleSingleStructGridTestClass : public ::testing::Test {
protected:
    typedef upsp::StructuredGrid<float> Wrapped;
    typedef Wrapped::node_idx node_idx;
    SimpleSingleStructGridTestClass();
    void add_zones();
    upsp::StructuredGrid<float> model;
};

/** Initializes a simple P3DModel */
class SimpleP3DModelTestClass : public ::testing::Test {
protected:
    typedef upsp::P3DModel_<float> Wrapped;
    typedef Wrapped::node_idx node_idx;
    typedef Wrapped::Face Face;
    typedef Wrapped::Node Node;
    // Create the model from the simple unstructured grid
    SimpleP3DModelTestClass();
    /** release memory for model */
    ~SimpleP3DModelTestClass() override;
    /* add two additional zones to the model */
    void add_zones(float offset = 0.0, float tol = 1e-10);
    upsp::P3DModel_<float>* model;
};

/** Initializes the sphere in the TriModel */
class TriModelTestClass : public ::testing::Test {
protected:
    typedef upsp::TriModel_<float> Wrapped;
    typedef Wrapped::node_idx node_idx;
    typedef Wrapped::face_idx face_idx;
    typedef Wrapped::Node Node;
    typedef Wrapped::Face Face;
    TriModelTestClass();
    upsp::TriModel_<float> model;
};

/** Initializes a very simple TriModel */
class SimpleTriModelTestClass : public ::testing::Test {
protected:
    typedef upsp::TriModel_<float> Wrapped;
    typedef Wrapped::node_idx node_idx;
    typedef Wrapped::face_idx face_idx;
    typedef Wrapped::Face Face;
    typedef Wrapped::Node Node;
    typedef Wrapped::simple_tri simple_tri;
    typedef Wrapped::simple_edge simple_edge;
    SimpleTriModelTestClass();
    upsp::Triangle<float> get_triangle(face_idx fidx) const;
    upsp::TriModel_<float> model;
};

/** Initializes data and grids for simple interpolation
 */
class SimpleInterpTestClass : public ::testing::Test {
protected:
    SimpleInterpTestClass();
    upsp::TriModel_<float> ref_model;
    upsp::TriModel_<float> new_model;
};

#endif  // TEST_GRID_UTILS_H_