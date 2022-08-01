/** @file
 *  @brief  Read uPSP Files
 *  @date   September 15, 2017
 *  @author jmpowel2
 */

#ifndef UFML_UTILS_FILE_READERS_H_
#define UFML_UTILS_FILE_READERS_H_

#include <algorithm>
#include <fstream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../data_structs.h"

#include "general_utils.h"

/** @namespace upsp_files
 * @brief   uPSP File Readers
 */

namespace upsp_files {

/** Set model surface normals based on input file
 *
 * @parma[in] normal_file   file with surface normal directions
 * @param[in] model         grid model
 * @param[out] success      true if successful, false otherwise
 */
template<typename M>
bool set_surface_normals(const std::string& normal_file, M& model);

/** Read in a csv file with 3D target locations
 *
 * @tparam          T               Target or Kulite 
 * @param[in]       target_file     input target data file
 * @param[in,out]   targs           T vector for loading data
 * @param[out]      success         true if successful, false otherwise
*/
template<typename T>
bool read_target_file(cv::String target_file, std::vector<T>& targs);

/** Read in *.tgts style target data file  
 *
 * @tparam          T               expects Target or Kulite
 * @param[in]       target_file     input target data file
 * @param[in,out]   targs           T vector loading xyz and diameter data
 * @param[in]       planar          if true, z is set to 0.0, else it is read from file
 * @param[in]       label           name of the set of points to include
 * @param[out]      success         true if successful, false otherwise
 */
template<typename T>
bool read_psp_target_file(cv::String target_file, std::vector<T>& targs, bool planar=false,
        cv::String label="*Targets");

/** Read in a csv file that defines which grid components should
 *  be used in the processing
 *
 * @param[in]   comp_file   file describing active grid components/zones
 * @return                  map between component id and bool
 *                          true if the component is active
 */
std::unordered_map<int,bool> read_active_comp_file(std::string comp_file);

} /* end namespace upsp_files */

#include "../utils/file_readers.ipp"

#endif /* UFML_UTILS_FILE_READERS_H_ */
