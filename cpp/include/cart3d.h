#ifndef CART3D_H_
#define CART3D_H_
#include <string>
#include <vector>
#include "grids.h"

/// Functions to read and write binary Cart3D files
/// (currently supported: annotated triangulation (*.triq) files)
/// Reference:
/// https://www.nas.nasa.gov/publications/software/docs/cart3d/pages/cart3dTriangulations.html

namespace upsp {

///  Read an annotated triangulation file (*.triq)
///  will just read the first q parameter (Cp)
/// 
/// @param[in] filename      name of the triq file
/// @param[out] grid         unstructured grid (triangles)
/// @return                  1st q parameter (Cp)
/// 
std::vector<float> read_triq(std::string filename, UnstructuredGrid<float>& grid);

}  // namespace upsp

#endif  // CART3D_H_