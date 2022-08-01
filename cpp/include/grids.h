/** @file
 *  @brief  Grid Data Structure Definitions
 *  @date   Feb 16, 2018
 *  @author jmpowel2
 */

#ifndef UFML_GRIDS_H_
#define UFML_GRIDS_H_

#include <algorithm>
#include <assert.h>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <cstring>
#include <vector>

#include "utils/general_utils.h"
#include "utils/file_io.h"

namespace upsp {

/** Defines directions along a structured 2D grid */
enum GridDir { J_plus, J_minus, K_plus, K_minus };

/** Abstract Class for Simple Grid Structures */
template<typename T>
struct Grid {

    typedef T data_type;
    typedef unsigned int node_idx;

    /** Is the grid structured or unstructured */
    virtual bool is_structured() const = 0;

    /** Return the number of nodes in the grid */
    unsigned int size() const { return x.size(); }

    /** Get the x locations of all nodes */
    const std::vector<T> get_x() const { return x; }
    /** Get the y locations of all nodes */
    const std::vector<T> get_y() const { return y; }
    /** Get the z locations of all nodes */
    const std::vector<T> get_z() const { return z; }

    std::vector<T> x; 
    std::vector<T> y; 
    std::vector<T> z;
 
};

/** Simple Storage for Multi-Zone Structured Grids */
template<typename T>
struct StructuredGrid : public Grid<T> {

    using node_idx = typename Grid<T>::node_idx;
    using data_type = typename Grid<T>::data_type;

    /** Return true, this is a structured grid */
    bool is_structured() const { return true; }

    /** Return the number of zones in the grid */
    unsigned int num_zones() const { return grid_size.size(); }

    /** Return the number of nodes in the zone */
    unsigned int zone_size(unsigned int zone) const;

    /** Return the number of nodes in a single dimension of a zone */
    unsigned int zone_size(unsigned int zone, unsigned int idx) const;

    /** Return the first node index of the zone */
    unsigned int zone_start_idx(unsigned int zone) const;

    /*************************************************************************/

    std::vector<std::vector<node_idx>> grid_size;

};

/** Bare bones helper struct using references 
 *  for feeding data into P3DModel
 */
template<typename T>
struct RefStructuredGrid {

    using node_idx = unsigned int;
    using data_type = T;

    /** Initialize the references */
    RefStructuredGrid(std::vector<T>& x_in, std::vector<T>& y_in, 
                      std::vector<T>& z_in) : x(x_in), y(y_in), z(z_in) {}

    /** Return the number of grid points */
    unsigned int size() const { return x.size(); }

    /** Return the number of zones in the grid */
    unsigned int num_zones() const { return grid_size.size(); }

    /** Return the number of nodes in the zone */
    unsigned int zone_size(unsigned int zone) const;

    /*************************************************************************/

    std::vector<std::vector<node_idx>> grid_size;

    std::vector<T>& x;
    std::vector<T>& y;
    std::vector<T>& z;
};

/** Unstructured Grid Structure (Triangles only), allows for multiple zones */
template<typename T>
struct UnstructuredGrid : public Grid<T> {

    using node_idx = typename Grid<T>::node_idx;
    using data_type = typename Grid<T>::data_type;

    struct Triangle;
    struct TriIterator;

    typedef unsigned int face_idx;
    typedef CircularArray<node_idx,3> simple_tri;

    UnstructuredGrid<T>() : has_n_comps(false), n_comps(0) {}

    /** Return false, this is an unstructured grid */
    bool is_structured() const { return false; }

    /** Return the set number of components 
     *  number is set, not dynamically computed
     */
    unsigned int num_zones() const { return n_comps; }

    /* Return the number of triangles */
    unsigned int number_of_faces() const { return tris.size(); }

    /** Sort the triangles by component 
     *
     * @post comps is ordered smallest to largest
     * @post has_n_comps == true
     * @post n_comps = number of unique components
     */
    void sort_by_components();

    /*************************************************************************/   

    std::vector<CircularArray<node_idx,3>> tris;
    std::vector<int> comps; // component ids, indexed by face_idx

    bool has_n_comps; // 
    unsigned int n_comps;
};

} /* end namespace upsp */

#include "../lib/grids.ipp"

#endif /* UFML_GRIDS_H_ */
