#ifndef PLOT3D_H_
#define PLOT3D_H_
#include <vector>
#include <string>
#include "grids.h"

// Functions to read and write binary Plot3D files
// (currently supported: multi-zone grids and function files)

namespace upsp {

/// Read scalar function values from a Plot3D multi-zone scalar function file.
///
/// Parsing is controlled by "record_seps," related to the presence of unformatted
/// FORTRAN record separators in the file on disk. Currently, we only support parsing
/// both leading + trailing seps, or no seps. The most common convention is for
/// files to ***contain*** record separators; files without separators are supported
/// primarily for compatibility with other NASA Ames software packages.
///
/// @param[in] filename    plot3d function filename (often *.f, *.p3d)
/// @param[in] record_seps control handling of FORTRAN record separators
///                        0: assume the file does NOT contain record separators
///                        +1: assume the file DOES contain record separators
///                        -1: try to intelligently guess whether file contains record separators
///
/// @throws std::invalid_argument if parsing of 'filename' fails.
///
std::vector<float> read_plot3d_scalar_function_file(std::string filename, int record_seps = -1);

/// Read an unformatted plot3d grid file
/// 
/// Supports unformatted plot3d files: single/double precision, little/big endian,
/// multi/single zone.  Supports files with/without IBLANKS, but does not read IBLANKs.
/// 
/// File Format:
///  - utilizes Fortran record-lengths for each block
///  - each zone has dimensions i x j x k
///  - points are laid out flat with i the fastest changing dimension and k the slowest.
///    e.g. point[n] = (k[n] * len(i) * len(j)) + (j[n] * len(i)) + i[n]
///
/// Single Zone Format (each line is a block):
///  - [IDIM,JDIM,KDIM] : 3 32bit integers representing dimensions in i,j,k directions
///                       IDIM x JDIM x KDIM = NPOINTS
///  - [ [x1,x2,...,xNPOINTS], [y1,y2,...,yNPOINTS], [z1,z2,...,zNPOINTS] ] : 
///                       (3 * NPOINTS) float or double representing the x,y,z 
///                       positions of each point
///
/// Multi-Zone Format (each line is a block):
///  - [NZONES] : 1 32bit integer representing the number of zones
///  - [ [IDIM,JDIM,KDIM]_1, [IDIM,JDIM,KDIM]_2,... [IDIM,JDIM,KDIM]_NZONES ] :
///                       (3 * NZONES) 32bit integers representing dimensions in
///                       i,j,k directions: IDIM_n x JDIM_n x KDIM_n = NPOINTS_n
///  - 1 block for each zone:
///    [ [x1,x2,...,xNPOINTS_n], [y1,y2,...,yNPOINTS_n], [z1,z2,...,zNPOINTS_n] ] :
///                       (3 * NPOINTS_n) float or double representing the x,y,z 
///                       positions of each point
///
/// If there are IBLANKS, they are included in the x,y,z block for each zone as
/// [ [x], [y], [z], [iblanks] ] where iblanks is NPOINTS 32bit integers
///                       
/// @tparam StructGrid       structured grid struct or class 
///
///                          with public attributes: 
///                              - grid_size (vector of vector of unsigned int)
///                              - x,y,z (vector of float or double -- consistent)
///
///                           with public member functions:
///                              - unsigned zone_size(unsigned) : gives size of a zone
///
///                          and with typename data_type (float or double)
/// @param[in] filename      plot3d filename
/// @param[out] grid         structured grid
///
///
template <typename StructGrid>
void read_plot3d_grid_file(std::string filename, StructGrid& grid);

/// Write an unformatted plot3d file
/// 
/// Writes a machine endian multi or single zone plot3d file with precision
/// determined by StructGrid::data_type
/// 
/// See read_plot3d_grid_file(std::string, StructGrid&) for more details on the 
/// plot3d file format
/// 
/// @tparam StructGrid       structured grid struct or class 
///
///                          with public attributes: 
///                              - grid_size (vector of vector of unsigned int)
///                              - x,y,z (vector of float or double --consistent)
/// 
///                          with public member functions:
///                              - unsigned num_zones() : gives the number of zones
///                              - unsigned zone_size(unsigned) : gives size of a zone
///
///                          and with typename data_type (float or double)
/// @param[in] filename      plot3d filename
/// @param[in] grid          structured grid
///
/// @pre grid.x.size() == grid.y.size() == grid.z.size()
/// @pre \f$ \sum_{i=1}^{grid.num\_zones()}{grid.zone\_size(i)} = grid.x.size() \f$
///
template<typename StructGrid>
void write_plot3d_grid_file(std::string filename, const StructGrid& grid);

}  // namespace upsp

#endif // PLOT3D_H_ 