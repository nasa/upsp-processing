/** @file
 *  @brief  HDF5 File Reader/Writer
 *  @date   February 6, 2018
 *  @author jmpowel2
 */

#ifndef UFML_PSPHDF5_H_
#define UFML_PSPHDF5_H_

#include <algorithm>
#include <assert.h>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <utility>

#include "H5Cpp.h"

#include "data_structs.h"
#include "grids.h"
#include "utils/general_utils.h"

namespace upsp {

const int H5_STRING_LEN  = 256;

// track version of filetype
// if no "psph5_version" attribute in file, then version = 0
const unsigned int PSPH5_VERSION = 1;

/** Return true if the grid is structured */
bool hdf5_is_structured(std::string filename);

/** Return true if this is a valid upsp HDF5 file */
bool hdf5_is_valid(std::string filename);

/** Return true if the data is nodal */
bool hdf5_is_nodal(std::string filename);

/** Return the psph5 version number for the file */
unsigned int hdf5_get_version(std::string filename);

/** Return the version of the software used to generate this data */
std::string hdf5_get_code_version(std::string filename);

/** Return the number of frames in the file */
unsigned int hdf5_num_frames(std::string filename);

/** Return the number of grid nodes in the file */
unsigned int hdf5_num_nodes(std::string filename);

/** Return the number of grid faces in the file */
unsigned int hdf5_num_faces(std::string filename);

/** Return the number of data points in the file
    this is either the number of nodes or faces depending
    on whether or not the data is nodal
 */
unsigned int hdf5_num_pts(std::string filename);

/** Return the grid units 
 *
 * @post if no grid units in file, return empty string
 */
std::string hdf5_grid_units(std::string filename);

/** Return true if the file contains additional datasets
 *  other than the grid and /frames
 */
bool hdf5_has_additional_datasets(std::string filename);

/** Create a list of all primary datasets in the file 
 *  values for each node/face and frame 
 *
 * @param[in] filename  upsp HDF5 file
 * @param[out] datasets all datasets available in the file
 */
void hdf5_primary_datasets(std::string filename, std::vector<std::string>& datasets);

/** Create a list of all additional datasets in the file 
 * 
 * @param[in] filename  upsp HDF5 file
 * @param[out] datasets all datasets available in the file
 */
void hdf5_additional_datasets(std::string filename, std::vector<std::string>& datasets);

/** Create a list of all attributes at the root of the file 
 *
 * @param[in] filename      upsp HDF5 file
 * @parma[out] attr_name    names of all attributes
 */
void hdf5_attributes(std::string filename, std::vector<std::string>& attr_name);

/** Return true if the file has the attribute in group "/" */
bool hdf5_has_attribute(std::string filename, std::string attr_name);

/** Return the value of a string type attribute 
 *
 * @pre @a attr_name must be in group "/"
 */
std::string hdf5_read_attribute(std::string filename, std::string attr_name);

/** Return the number of grid nodes per chunk in the file 
 *
 * @pre @a filename must have a chunked primary dataset
 */
unsigned int hdf5_pts_per_chunk(std::string filename);

/** Return the number of frames per chunk in the file 
 *
 * @pre @a filename must be chunked
 */
unsigned int hdf5_frames_per_chunk(std::string filename);

/** Return true if the file is a transposed upsp HDF5 file 
 *
 * @post if @a filename has no primary datasets, return true 
 */
bool hdf5_is_transposed(std::string filename);

/** Read the units attribute for a dataset
 *
 * @param[in] filename  psp hdf5 file
 * @param[in] dataset   name of the dataset
 * @return  units, if not available (return "")
 */
std::string hdf5_read_dataset_units(std::string filename, std::string dataset);

/** Read grid from hdf5 file
 * @param[in] filename  name of hdf5 psp file
 * @param[out] model    grid model
 * @return grid units
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename Model>
std::string hdf5_read_grid(std::string filename, Model*& model);

/** Read a structured grid from hdf5 file
 * @param[in] filename  name of hdf5 psp file
 * @param[out] model    grid model
 * @return grid units
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename Model>
std::string hdf5_read_structured_grid(std::string filename, Model*& model);

/** Read an ustructured grid from hdf5 file
 * @param[in] filename  name of hdf5 psp file
 * @param[out] model    grid model
 * @return grid units
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename Model>
std::string hdf5_read_unstructured_grid(std::string filename, Model*& model);

/** Read a frame of "frames" from the file 
 * @param[in] filename  psp hdf5 file name
 * @param[in] frame     frame to read (0 based)
 * @param[out] sol      solution vector
 *
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename T>
void hdf5_read_solution(std::string filename, unsigned int frame, std::vector<T>& sol);
/** Read a frame of "frames" from the file 
 * @param[in] filename  psp hdf5 file name
 * @param[in] frame     frame to read (0 based)
 * @param[out] sol      solution vector 
 * @param[in] grid_size number of grid points in model associated with @a filename
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename T>
void hdf5_read_solution(std::string filename, unsigned int frame, std::vector<T>& sol,
    unsigned int grid_size);

/** Read a frame of a primary dataset from the file
 * @param[in] filename  psp hdf5 file name
 * @param[in] name      name of the dataset
 * @param[in] frame     frame to read (0 based)
 * @param[out] sol      solution vector
 *
 * @pre name must be a primary dataset 
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename T>
void hdf5_read_solution(std::string filename, std::string name, unsigned int frame, std::vector<T>& sol);
 
/** Read a single frame of "frames" for a single zone
 * 
 * @param[in] filename  psp hdf5 file name
 * @param[in] frame     frame to read (0 based)
 * @param[in] zone      zone to read (0 based)
 * @param[out] sol      solution vector 
 *
 * @pre hdf5_is_valid(@a filename) 
 * @pre hdf5_is_structured(@a filename)
 */
template<typename T>
void hdf5_read_solution_zone(std::string filename, unsigned int frame, unsigned int zone, std::vector<T>& sol);

/** Read a hyperslab of data from a nominal (non-transposed) file
 * 
 * @param[in] filename      psp hdf5 file name
 * @param[in] name          dataset name
 * @param[in] frame_start   first frame to read (0-based)
 * @param[in] frame_delta   increment between frames to read
 * @param[in] frame_end     last frame to read (-based)
 * @param[out] sol          solution vector
 *
 * @pre hdf5_is_valid(@a filename)
 */
template<typename T>
void hdf5_read_solution(std::string filename, std::string name, 
        unsigned int frame_start, unsigned int frame_delta, 
        unsigned int frame_end, std::vector<T>& sol);

/** Read nodes from transpose solution
 * @param[in] filename      psp hdf5 file name
 * @param[in] dset_name     name of the dataset to read
 * @param[in] pt_start      first data pt to read (0 based)
 * @param[in] pt_end        last data pt to read (0 based)
 * @param[out] sol          solution vector 
 * @param[in[ frame_start   first frame to read (0 based)
 * @param[in] frame_incr    read every frame_incr frames for these nodes
 * @param[in] frame_end     last frame to read (<0 means read from 
 *                          @a frame_start to end of file)
 *
 * @pre node_end >= node_start
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename T>
void hdf5_read_transpose_solution(std::string filename, std::string dset_name,
        unsigned int node_start, 
        unsigned int node_end, std::vector<T>& sol, unsigned int frame_start=0, 
        unsigned int frame_incr=1, int frame_end=-1);

/** Read nodes from transpose solution
 *  uses /frames dataset
 * @param[in] filename      psp hdf5 file name
 * @param[in] pt_start      first data pt to read (0 based)
 * @param[in] pt_end        last data pt to read (0 based)
 * @param[out] sol          solution vector 
 * @param[in[ frame_start   first frame to read (0 based)
 * @param[in] frame_incr    read every frame_incr frames for these nodes
 * @param[in] frame_end     last frame to read (<0 means read from 
 *                          @a frame_start to end of file)
 *
 * @pre node_end >= node_start
 * @pre hdf5_is_valid(@a filename) 
 */
template<typename T>
void hdf5_read_transpose_solution(std::string filename, unsigned int node_start, 
        unsigned int node_end, std::vector<T>& sol, unsigned int frame_start=0, 
        unsigned int frame_incr=1, int frame_end=-1);

/** Read a full dataset from the psp hdf5 file
 * @param[in] filename  psp hdf5 filename
 * @param[in] name      dataset name
 * @param[out] sol      return of solution
 * 
 * @pre size of dataset is equal to the size of the grid
 */ 
template<typename T>
std::string hdf5_read_dataset(std::string filename, std::string name, std::vector<T>& sol);

/** Read tunnel conditions from the file
 * @param[in] filename  hdf5 filename
 * @param[out] cond     tunnel condition structure
 */
template<typename Cond>
void hdf5_read_tunnel_conditions(std::string filename, Cond& cond);

/** Read camera settings from the file
 * @param[in] filename  hdf5 filename
 * @param[out] cs       camera settings structure
 */
template<typename CamSettings>
void hdf5_read_camera_settings(std::string filename, CamSettings& cs);

/** Determine the most efficient way to read in multiple
 *  frames from a chunked dataset
 *  for nominal (untransposed) hdf5 files
 *
 * @param[in] filename      hdf5 filename
 * @param[in] frame_start   first frame to read (0-based)
 * @param[in] frame_delta   increment between frames
 * @param[in] frame_end     last frame to read (0-based)
 * @param[in] max_memory    maximum size of data to read in (MB)
 * @param[out] partitions   vector of efficient read partitions
 *                          first partition is 
 *                          [partitions[0].get<0>(),partitions[0].get<1>())
 *
 * @pre @a filename has primary datasets
 * @pre hdf5_is_transposed(@a filename) == false
 * @pre assumes native floats in primary datasets
 */
void hdf5_efficient_read(std::string filename, unsigned int frame_start, 
        unsigned int frame_delta, unsigned int frame_end, 
        float max_memory, 
        std::vector<std::pair<unsigned int,unsigned int>>& partitions);

/** Convert C++ data type to hdf5 type */
template<typename T>
struct psph5_valid_type : std::false_type {};
template<>
struct psph5_valid_type<float> : std::true_type {};
template<>
struct psph5_valid_type<double> : std::true_type {};

template <typename Model>
class PSPWriter {
public:
    using mtype = typename Model::data_type;

    /** Create PSPWriter object
     * @param[in] filename      hdf5 file for writing
     * @param[in] model         model for writing grid
     * @param[in] num_frames    number of frames to write
     * @param[in] chunk_size    maximum size of hdf5 chunks in MB
     * @param[in] transposed    if true, write out transposed solutions
     * @param[in] trans_nodes   the number of nodes per chunk if transposed
     * @param[in] nodal         if true, data is recorded at each node
     *                          if false, data is at each face
     * 
     * @pre if @a transposed, @a num_frames > 0
     * @pre if num_frames == 0, use unlimited for solution size
     */ 
    PSPWriter(std::string filename, const Model* model, unsigned int num_frames, 
            float chunk_size=1.0, bool transposed = false, 
            unsigned int trans_nodes = 250, bool nodal = true); 

    /** Flush out buffers and close file */
    ~PSPWriter();

    /** Write the grid to the HDF5 file 
     *
     * @param[in] units     grid units
     */
    void write_grid(std::string units = "");

    /** Write out a solution to the file
     *  expects all model nodes and some number of frames
     *  repeated calling writes frames in sequence
     * TODO: improve documentation
     * @param[in] sol           solution of length 
     *                          model.size()*F if nodal
     *                          model.number_of_frames()*F if !nodal
     *                          F = some number of frames
     * @param[in] all_frames    if true, the transpose solution is for 
     *                          some number of data points, all frames
     */
    template<typename T>
    void write_solution(const std::vector<T>& sol, bool all_frames = false);

    /** Write out a specific solution to the file
     *  expects all model nodes and some number of frames
     *  repeated calling write frames in sequence
     *
     * @param[in] sol_name      name of the output dataset
     * @param[in] sol           solution of length
     *                          model.size()*F if nodal
     *                          model.number_of_frames()*F if !nodal
     *                          F = some number of frames
     * @param[in] all_frames    if true, the transpose solution is for
     *                          some number of data points, all frames
     */
    template<typename T>
    void write_solution(std::string sol_name, const std::vector<T>& sol, 
            bool all_frames = false);

    /* TODO: write specific block of data
     *  Write out a block of data to the file
     *  any number of consecutve model nodes and any number of consecutive
     *  frames
     *
     * @param[in] sol_name      name of the output dataset
     * @param[in] sol           solution data (ordered consistent with
     *                          final nominal or transpose matrix)
     * @param[in] 
     */

    /** Create an additional Dataset with a single value at each data point
     * 
     * @tparam T        float or double
     * @param[in] name  dataset name
     * @param[in] sol   values of the dataset at each grid point
     * @param[in] units string with units 
     *
     * @pre @a sol.size() == model_->size()
     */
    template<typename T>
    void write_new_dataset(std::string name, std::vector<T>& sol, std::string units="");

    /** Add a units attribute to a dataset
     *
     * @param[in] dataset   dataset name
     * @param[in] units     units string
     *
     * @pre @a name must be a dataset already written to the file
     */
    void add_units(std::string dataset, std::string units);

    /** Add camera settings to attributes */
    template<typename CamSet>
    void write_camera_settings(const CamSet& cs);
    
    /** Add tunnel conditions to attributes */
    template<typename Cond>
    void write_tunnel_conditions(const Cond& cond);

    /** Write the version of the code (git version) */
    void write_code_version(std::string code_version);

    /** Add a string attribute
     * @param[in] attr_name     The name of the attribute
     * @param[in] attr_value    The attribute data
     *
     * @pre attr_value.length() < H5_STRING_LEN
     */
    void write_string_attribute(std::string attr_name, std::string attr_value);

    /** Set chunk size in MB */
    void set_chunk_size(float chunk_size);

    /** Return the number of frames per chunk */
    unsigned int get_chunk_length() const { return chunk_length_; }

    /** Return the number of frames planned to be written */
    unsigned int get_num_frames() const { return num_frames_; }

    /** Return the underlying model */
    Model* get_model() { return model_; }

private:

    /** Write solution for next frame */
    template<typename T>
    void nominal_write_solution(unsigned int sol_id, const std::vector<T>& sol);

    /** Write solution for some number of frames, all model nodes */
    template<typename T>
    void transpose_write_solution(unsigned int sol_id, const std::vector<T>& sol);

    /** Write solution for all frames, some number of nodes */
    template<typename T>
    void transpose_write_solution2(unsigned int sol_id, const std::vector<T>& sol);

    /** Determine the H5 datatype based on model data type */
    void get_data_type(H5::DataType& ptyp) const;

    /** Set the size of the chunk */
    void set_chunk_length();

    /** Write a structured grid */
    void write_structured_grid(std::string units);

    /** Write an unstructured grid */
    void write_unstructured_grid(std::string units);

    /*************************************************************************/

    std::string filename_;

    bool transposed_; // is output file transposed
    bool nodal_; // is data nodal

    const Model* model_;

    float chunk_size_; // uncompressed size of a chunk in MB
    unsigned int chunk_length_; // number of frames in chunk
    unsigned int trans_nodes_; // number of unique data points in transposed chunk
    unsigned int num_frames_; // number of frames to write
    unsigned int data_pts_; // number of data points per frame

    H5::H5File* file_;

    // track current data_pt and frame for each major solution (default 1: frames)
    // vector storage should be fine for a small number of solutions
    std::vector<std::string> sols_;
    std::vector<unsigned int> curr_pt_;
    std::vector<unsigned int> curr_frame_;

    std::vector<bool> open_dataset_;
    std::vector<H5::DataSet> dataset_;
};

} /* end namespace upsp */

#include "../lib/PSPHDF5.ipp"

#endif /* UFML_PSPHDF5_H_ */
