/** @file
 *  @brief  Define Test Classes for H5 Read/Write
 *  @date   September 4, 2019
 *  @author jmpowel2
 */

#ifndef TEST_H5_CLASS_H_
#define TEST_H5_CLASS_H_

#include "gtest/gtest.h"

#include "PSPHDF5.h"

#include "sample_data/pencil.h"
#include "sample_data/sample_data.h"

class NominalH5TestClass : public ::testing::Test {
protected:

    NominalH5TestClass() {
        // Unstructured, non-intersected, nodal, nominal
        filename = "inputs/unstruct_nodal_pencil.h5";

        // Get the file format
        nodal      = upsp::hdf5_is_nodal(filename);
        transpose  = upsp::hdf5_is_transposed(filename);
        structured = upsp::hdf5_is_structured(filename);

        // Generate the expected grid file
        tri = pencil.get_tri_model(false); // not intersected
        
        n_pts = upsp::hdf5_num_pts(filename);

        // Get the number of frames in the file
        n_frames = upsp::hdf5_num_frames(filename);

    }

    /** load expected "/frames" data from the file */
    std::vector<float> load_truth() const {
        std::vector<float> truth;
        create_data(tri, nodal, n_frames, truth);
        return truth;
    } 

    /*****************************************************/
    std::string filename;

    unsigned int n_frames;
    unsigned int n_pts;

    bool nodal;
    bool transpose;
    bool structured;

    upsp::TriModel_<float> tri;

    Pencil<float> pencil;
};

class TransposeH5TestClass : public ::testing::Test {
protected:

    TransposeH5TestClass() {
        // Unstructured, non-intersected, nodal, nominal
        filename = "inputs/unstruct_nodal_pencil_trans.h5";

        // Get the file format
        nodal      = upsp::hdf5_is_nodal(filename);
        transpose  = upsp::hdf5_is_transposed(filename);
        structured = upsp::hdf5_is_structured(filename);

        // Generate the expected grid file
        tri = pencil.get_tri_model(false); // not intersected
        
        n_pts = upsp::hdf5_num_pts(filename);

        // Get the number of frames in the file
        n_frames = upsp::hdf5_num_frames(filename);

    }

    /** load expected "/frames" data from the file */
    std::vector<float> load_truth() const {
        std::vector<float> truth;
        create_data(tri, nodal, n_frames, truth);
        std::vector<float> t_truth(truth.size());
        transpose_block(&truth[0], &t_truth[0], n_frames, n_pts, 16);
        return t_truth;
    } 

    /*****************************************************/
    std::string filename;

    unsigned int n_frames;
    unsigned int n_pts;

    bool nodal;
    bool transpose;
    bool structured;

    upsp::TriModel_<float> tri;

    Pencil<float> pencil;
};

#endif /* TEST_H5_CLASS_H_ */
