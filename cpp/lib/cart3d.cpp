/// @file
/// @brief  Cart3D File read/write utilities
/// @date   September 10, 2019
/// @author jmpowel2, mshawlec
#include <iostream>
#include <string>
#include "grids.h"

namespace upsp {

/*****************************************************************************/
std::vector<float> read_triq(std::string filename, UnstructuredGrid<float>& grid) {

    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs) {
        throw(std::invalid_argument("Cannot open triq file: " + filename));
    }

    // setup Fortran record-length variables
    int32_t sz, sz2;

    // read header
    int32_t n_node, n_tri, n_scalar;
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (3*sizeof(int32_t))) {
        throw(std::invalid_argument("Unable to read triq file: " + filename));
    }
    ifs.read((char*) &n_node, sizeof(int32_t));
    ifs.read((char*) &n_tri, sizeof(int32_t));
    ifs.read((char*) &n_scalar, sizeof(int32_t));
    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read triq file: " + filename));
    }

    // check that at least 1 scalar is included (Cp)
    if (n_scalar < 1) {
        throw(std::invalid_argument("triq file must include a scalar variable"));
    }

    // allocate storage
    std::vector<float> cp(n_node);
    grid.x.resize(n_node);
    grid.y.resize(n_node);
    grid.z.resize(n_node);
    grid.tris.resize(n_tri);
    grid.comps.resize(n_tri);

    // read nodes
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (sizeof(float)*3*n_node)) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of nodes"));
    }

    for (unsigned int i=0; i < n_node; ++i) {
        ifs.read((char*) &grid.x[i], sizeof(float));
        ifs.read((char*) &grid.y[i], sizeof(float));
        ifs.read((char*) &grid.z[i], sizeof(float));
    }

    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of nodes"));
    }

    // read faces
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (sizeof(int32_t)*3*n_tri)) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of tris"));
    }

    for (unsigned int i=0; i < n_tri; ++i) {
        ifs.read((char*) &grid.tris[0], sizeof(int32_t)*3);
    }

    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of tris"));
    }

    // read components (expect that they are included)
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != (sizeof(int32_t)*n_tri)) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of component labels"));
    }

    ifs.read((char*) &grid.comps[0], sizeof(int32_t)*n_tri);

    ifs.read((char*) &sz2, sizeof(int32_t));
    if (sz != sz2) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of component labels"));
    }

    // read 1st scalar
    ifs.read((char*) &sz, sizeof(int32_t));
    if (sz != sizeof(float)*n_node*n_scalar) {
        throw(std::invalid_argument("Unable to read triq file,"
                                    " inconsistent number of scalars"));
    }
    ifs.read((char*) &cp[0], sizeof(float)*n_node);

    ifs.close();

    return cp;
}

} /* end namespace upsp */
