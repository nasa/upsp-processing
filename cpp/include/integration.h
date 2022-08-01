/** @file
 *  @brief  Integration over panels
 *  @date   May 1, 2019
 *  @author jmpowel2
 */

#ifndef UFML_INTEGRATION_H_
#define UFML_INTEGRATION_H_

#include <algorithm>
#include <array>
#include <boost/iterator/transform_iterator.hpp>
#include <Eigen/Sparse>
#include <opencv2/opencv.hpp>
#include <set>
#include <vector>

#include "data_structs.h"
#include "grids.h"
#include "models.h"

#include "Octree.h"
#include "TriModel.h"
#include "P3DModel.h"

namespace upsp {

/** Manages the full surface and volume Panel Definitions */
template<typename FP>
struct Panels {

    /** Create the hexagons by extending surface panels or directly
     *  using a given volume grid, in which case the surface panels are
     *  defined as the midpoint along the l-direction (j,k,l)
     *
     * @param[in] grid          plot3d grid, either surface or volume
     *                          defining the panels
     * @param[in] height_sf     scale factor for defining the height of the panels
     *                          height = height_sf * min edge length
     *                          only used if @a grid is a surface grid
     *
     * @pre @a grid.num_zones() > 0
     * @pre @a grid must be either a surface or volume grid (no mixing and 
     *      no curves)
     * @pre if @a grid is a surface grid, all zones must have size in l = 1
    */
    Panels(const StructuredGrid<FP>& grid, float height_sf=1.0);

    /** Return the number of panels */
    unsigned int size() const { return hexes.size(); }

    /*****************************************************/

    std::vector<Polyhedron<6,FP>> hexes;
    P3DModel_<FP> surfaces;

};

/** Holds the panel integration matrices and logged information */
template<typename FP>
struct PanelIntegration {

    /** Check if the object is self-consistent */
    bool is_valid() const;

    /*****************************************************/
    bool transpose;
    Eigen::SparseMatrix<FP> fx;
    Eigen::SparseMatrix<FP> fy;
    Eigen::SparseMatrix<FP> fz;
    Eigen::SparseMatrix<FP> mx;
    Eigen::SparseMatrix<FP> my;
    Eigen::SparseMatrix<FP> mz;
    std::vector<FP> center_x; // stores the panel centroid x-position
    std::vector<FP> center_y; // stores the panel centroid y-position
    std::vector<FP> center_z; // stores the panel centroid z-position
    std::vector<FP> area; // stores the area of the underlying triangles for each panel
    std::vector<FP> coverage; // stores the percent of area in panel with coverage
};

/** Holds the integrated data in each direction */
template<typename FP>
struct FoMo {
    std::vector<FP> fx;
    std::vector<FP> fy;
    std::vector<FP> fz;
    std::vector<FP> mx;
    std::vector<FP> my;
    std::vector<FP> mz;
};

/** Create the integration matrix for integrating over panels
 * Does not currently support non-data triangles for non-nodal data
 *
 * @param[in] tri           the triangle model (where data is located)
 * @param[in] panels        panel surface and volume definitions
 * @param[in] nodal         if true, data is stored at nodes and
 *                                   assume value of triangle is average of nodes
 *                          if false, data is stored at each triangle
 * @param[in] coverage      for each node or triangle, true if there is data 
 * @return the PanelIntegration object which stores all integration matrices
 *         and information about coverage on a panel basis
 *
 * @pre tri.using_nodes() == false
 * @pre if nodal == true, coverage.size() == tri.size()
 *      else            , coverage.size() == tri.number_of_faces()
 * @post coverage computation does not account for gaps in @a tri where
 *       @a panels exist
 * @post for 0 <= i < @a tri.number_of_faces(), 
 *       if @a tri.is_dataface(i) == false, then face i is not included in 
 *       integration matrices or coverage calculation
 * @post @return.transpose = true (matrices are n_panels x n_data_pts)
 */
template<typename FP>
PanelIntegration<FP> create_integration_matrices(const TriModel_<FP>& tri, 
        const Panels<FP>& panels, bool nodal, const std::vector<bool>& coverage);

/** Integrate over panels
 * 
 * @param[in] pinteg    PanelIntegration object containing information needed
 *                      for completing the integration
 * @param[in] values    pressure values of the data on the underyling mesh
 * @return              integrated values at each panel
 *                      forces (pressure * area (grid units^2))
 *                      moments (pressure * area (grid units^2) * moment arm (grid units))
 *
 * @pre @a pinteg.is_valid()
 * @pre if  pinteg.transpose: @a values.size() % @a pinteg.fx.rows() == 0
        else                : @a values.size() == pinteg.fx.cols()
 * @post @a return.fx.size() == @a return.fy.size() == @a return.fz.size() 
 *    == @a return.mx.size() == @a return.my.size() == @a return.mz.size()
 * @post if pinteg.transpose: @a return.mx.size() == 
 *                            @a values.size / @a pinteg.fx.rows() * @a pinteg.fx.cols()
 *       else               : @a return.mx.size() == @a pinteg.fx.rows()
 */
template<typename FP>
FoMo<FP> integrate_panels(const PanelIntegration<FP>& pinteg, 
                            const std::vector<FP>& values);

/** Create a polyhedron around a P3DModel Face
 *  on edges of face, define normal by the average of adjacent faces
 *
 * @param[in] face          P3DModel Face
 * @param[in] height_sf     set the height of polyhedron as a fraction of of its minimum
 *                          edge length
 *
 * @pre f.is_valid()
 */
template<typename FP>
Polyhedron<6,FP> build_polyhedron(const typename P3DModel_<FP>::Face& f, FP height_sf=1.0);

/** Create hexahedrons from a volume grid
 *
 * @param[in] grid      structured volume grid
 */
template<typename FP>
std::vector<Polyhedron<6,FP>> build_polyhedrons(const StructuredGrid<FP>& grid);

/** Split a TriModel along boundaries of P3DModel faces and set TriModel face
 *  components to match the face_idx of the P3DModel face they are contained within
 *
 * @param[in,out] tri           triangle mesh
 * @param[in]     panels        panel definitions
 * @param[in]     break_conn    if true, don't break edges on adjacent triangles
 *                              when an edge is split
 *
 * @pre tri.using_nodes() == false
 * @post any @a tri faces not in any @a panels faces, will have component -1
 */
template<typename FP>
void split_model(TriModel_<FP>& tri, const Panels<FP>& panels,
        bool break_conn=false);

/** Transpose panel integration matrices
 *
 * @param[in] pinteg    panel integration object
 * @return              transposed panel integration object
 *
 * @post pinteg.center_x == @return.center_x, pinteg.center_y == @return.center_y,
 *       pinteg.center_z == @return.center_z, pinteg.area == @return.area,
 *       pinteg.coverage == @return.coverage
 * @post @return.fx.rows() == @return.fy.rows() == @return.fz.rows() ==
 *       @return.mx.rows() == @return.my.rows() == @return.mz.rows() ==
 *       pinteg.fx.cols()
 * @post @return.fx.cols() == @return.fy.cols() == @return.fz.cols() ==
 *       @return.mx.cols() == @return.my.cols() == @return.mz.cols() ==
 *       pinteg.fx.rows()
 * @post @return.transpose == !pinteg.transpose
 */
template<typename FP>
PanelIntegration<FP> transpose(const PanelIntegration<FP>& pinteg);

} /* end namespace upsp */

#include "../lib/integration.ipp"

#endif /* UFML_INTEGRATION_H_ */
