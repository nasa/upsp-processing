#include <fstream>
#include "test_grid_utils.h"
#include "../include/data_structs.h"

bool compare_files(const std::string& file1, const std::string& file2) {

    std::ifstream f1(file1, std::ifstream::binary | std::ifstream::ate);
    std::ifstream f2(file2, std::ifstream::binary | std::ifstream::ate);

    // Check that the files were succesfully opened
    if (f1.fail() || f2.fail()) {
        return false;
    }

    // Check that the files are the same size
    if (f1.tellg() != f2.tellg()) {
        return false;
    }

    // Seek back to the beginning
    f1.seekg(0, std::ifstream::beg);
    f2.seekg(0, std::ifstream::beg);

    // Compare contents for equality
    return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()), 
                      std::istreambuf_iterator<char>(),
                      std::istreambuf_iterator<char>(f2.rdbuf()));
}

/* Create a structured grid with a single component
 *
 * origin at (0), equally spaced in 1.0 increments
 *
 * (16) - (17) - (18) - (19)
 *  |      |      |      |
 * (12) - (13) - (14) - (15)
 *  |      |      |      |
 * (8)  - (9)  - (10) - (11)
 *  |      |      |      |
 * (4)  - (5)  - (6)  - (7)
 *  |      |      |      |
 * (0)  - (1)  - (2)  - (3)
 * 
 * ^ k,y
 * |
 * |___ > j,x
 * normal is out of the screen
 */
void create_single_structgrid(upsp::StructuredGrid<float>& grid) {

    // initialize the node storage
    grid.x.assign(20, 0.0);
    grid.y.assign(20, 0.0);
    grid.z.assign(20, 0.0);

    // assign values
    for (unsigned int k=0; k < 5; ++k) {
        for (unsigned int j=0; j < 4; ++j) {
            grid.x[k*4+j] = static_cast<float>(j);
            grid.y[k*4+j] = static_cast<float>(k);
        }
    }

    // set up the grid sizing
    grid.grid_size.resize(1);
    grid.grid_size[0].resize(3);
    grid.grid_size[0][0] = 4; // j size
    grid.grid_size[0][1] = 5; // k size
    grid.grid_size[0][2] = 1; // l size

}

// Create an unstructured grid with a single component
// (5) * - * (0)
//      \ / \
//   (1) * - * (2)
//        \ /
//         * (3)
//         
//         * (4)
//         
//   (6) * - * (7)
//        \ /
//         * (8)
// normal is out of the screen
// faces in order left to right, top to bottom
void add_zones_structgrid(upsp::StructuredGrid<float>& grid, float offset=0.0) {
    
    // initialize the extra node storage
    grid.x.resize(52, 0.0);
    grid.y.resize(52, 0.0);
    grid.z.resize(52, 0.0);

    // assign values for zone 1
    unsigned int idx = 20;
    for (unsigned int k=0; k < 4; ++k) {
        for (unsigned int j=0; j < 3; ++j) {
            grid.x[idx + k*3+j] = static_cast<float>(j+3) + offset;
            grid.y[idx + k*3+j] = static_cast<float>(k);
        }
    }

    // assign values for zone 2
    idx = 32;
    for (unsigned int k=0; k < 4; ++k) {
        for (unsigned int j=0; j < 5; ++j) {
            grid.x[idx + k*5+j] = 6.0 - static_cast<float>(k) + offset;
            grid.y[idx + k*5+j] = static_cast<float>(j) - 4.0 - offset;
        }
    }
    
    // set up the grid sizing
    grid.grid_size.resize(3);
    grid.grid_size[1].resize(3);
    grid.grid_size[1][0] = 3; // j size
    grid.grid_size[1][1] = 4; // k size
    grid.grid_size[1][2] = 1; // l size
    grid.grid_size[2].resize(3);
    grid.grid_size[2][0] = 5; // j size
    grid.grid_size[2][1] = 4; // k size
    grid.grid_size[2][2] = 1; // l size

}

// Create an unstructured grid with a single component
// (5) * - * (0)
//      \ / \
//   (1) * - * (2)
//        \ /
//         * (3)
//         
//         * (4)
//         
//   (6) * - * (7)
//        \ /
//         * (8)
// normal is out of the screen
// faces in order left to right, top to bottom
void create_simple_unstructgrid(upsp::UnstructuredGrid<float>& ugrid) {

    // Allocate memory
    ugrid.x.resize(9);
    ugrid.y.resize(9);
    ugrid.z.resize(9);

    ugrid.tris.resize(4);

    // Set x,y,z positions
    ugrid.x[0] =  0.0; ugrid.y[0] =  0.0; ugrid.z[0] = 0.0; // 0
    ugrid.x[1] = -1.0; ugrid.y[1] = -1.0; ugrid.z[1] = 0.0; // 1
    ugrid.x[2] =  1.0; ugrid.y[2] = -1.0; ugrid.z[2] = 0.0; // 2
    ugrid.x[3] =  0.0; ugrid.y[3] = -2.0; ugrid.z[3] = 0.0; // 3
    ugrid.x[4] =  0.0; ugrid.y[4] = -3.0; ugrid.z[4] = 0.0; // 4
    ugrid.x[5] = -2.0; ugrid.y[5] =  0.0; ugrid.z[5] = 0.0; // 5
    ugrid.x[6] = -1.0; ugrid.y[6] = -4.0; ugrid.z[6] = 0.0; // 6
    ugrid.x[7] =  1.0; ugrid.y[7] = -4.0; ugrid.z[7] = 0.0; // 7
    ugrid.x[8] =  0.0; ugrid.y[8] = -5.0; ugrid.z[8] = 0.0; // 8

    ugrid.tris[0] = {5,1,0}; // 0
    ugrid.tris[1] = {0,1,2}; // 1
    ugrid.tris[2] = {2,1,3}; // 2
    ugrid.tris[3] = {6,8,7}; // 3
    
}

SimpleSingleStructGridTestClass::SimpleSingleStructGridTestClass() {
    create_single_structgrid(model);
}

void SimpleSingleStructGridTestClass::add_zones() {
    add_zones_structgrid(model);
}

SimpleP3DModelTestClass::SimpleP3DModelTestClass() {
    upsp::StructuredGrid<float> grid;
    create_single_structgrid(grid);
    model = new upsp::P3DModel_<float>(grid, 1e-10);
}

/** release memory for model */
SimpleP3DModelTestClass::~SimpleP3DModelTestClass() {
    delete model;
}

/* add two additional zones to the model */
void SimpleP3DModelTestClass::add_zones(float offset, float tol) {
    upsp::StructuredGrid<float> grid;
    create_single_structgrid(grid);
    add_zones_structgrid(grid, offset);
    delete model;
    model = new upsp::P3DModel_<float>(grid, tol);
}

TriModelTestClass::TriModelTestClass() {
    const std::string input_file = "./inputs/sphere_unf_multi.i.tri";
    model.load_grid(input_file, false);
}

// Create an unstructured grid with a single component
// (5) * - * (0)
//      \ / \
//   (1) * - * (2)
//        \ /
//         * (3)
//         
//         * (4)
//         
//   (6) * - * (7)
//        \ /
//         * (8)
// normal is out of the screen
// faces in order left to right, top to bottom
SimpleTriModelTestClass::SimpleTriModelTestClass() {
    model.add_node( 0.0,  0.0, 0.0); // 0
    model.add_node(-1.0, -1.0, 0.0); // 1
    model.add_node( 1.0, -1.0, 0.0); // 2
    model.add_node( 0.0, -2.0, 0.0); // 3
    model.add_node( 0.0, -3.0, 0.0); // 4
    model.add_node(-2.0,  0.0, 0.0); // 5
    model.add_node(-1.0, -4.0, 0.0); // 6
    model.add_node( 1.0, -4.0, 0.0); // 7
    model.add_node( 0.0, -5.0, 0.0); // 8

    model.add_face(5,1,0); // 0
    model.add_face(0,1,2); // 1
    model.add_face(2,1,3); // 2
    model.add_face(6,8,7); // 3
}

upsp::Triangle<float> SimpleTriModelTestClass::get_triangle(face_idx fidx) const {
    simple_tri simp = model.face_nodes(fidx);

    return {model.get_position(simp[0]),model.get_position(simp[1]),
            model.get_position(simp[2])};
}

SimpleInterpTestClass::SimpleInterpTestClass() {
    upsp::UnstructuredGrid<float> ugrid;
    create_simple_unstructgrid(ugrid);
    ref_model.load_grid(ugrid);
    new_model.load_grid(ugrid);
}
