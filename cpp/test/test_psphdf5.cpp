/** @file
 *  @brief  Test the PSPHDF5 read and write capabilities
 *  @date   August 15, 2019
 *  @author jmpowel2
 */
#include "gtest/gtest.h"

#include "../include/PSPHDF5.h"

#include "test_h5_class.h"

/** Test that NominalH5TestClass is setup as expected */
TEST_F(NominalH5TestClass, Initialization) {

    ASSERT_TRUE(nodal);
    ASSERT_FALSE(transpose);
    ASSERT_FALSE(structured);

    ASSERT_EQ(n_pts, tri.size());
}

/** Test Reading nominal HDF5 1 frame at a time */
TEST_F(NominalH5TestClass, ReadFrame) {

    // Generate truth data
    std::vector<float> truth = load_truth();
    
    // Load the data 1 frame at a time and compare to the truth data
    for (unsigned int f=0; f < n_frames; ++f) {
        std::vector<float> data;
        upsp::hdf5_read_solution(filename, f, data);

        for (unsigned int i=0; i < data.size(); ++i) {
            ASSERT_FLOAT_EQ(data[i], truth[f*data.size()+i]);
        }
    }

}

/** Test Reading nominal HDF5 in increments */
TEST_F(NominalH5TestClass, ReadFrameDelta) {

    unsigned int f_start  = 0;
    unsigned int d_frames = 3;

    // Load the data every d_frames from the file
    std::vector<float> data;
    upsp::hdf5_read_solution(filename, "frames", f_start, d_frames,
            n_frames-1, data);

    // Check that the correct number of frames were read
    unsigned int read_frames = (n_frames - f_start + d_frames -1) / d_frames;
    ASSERT_EQ(data.size(), read_frames*n_pts);

    // Generate truth data
    std::vector<float> truth = load_truth();

    // Compare to truth data
    unsigned int row=0;
    for (unsigned int f=f_start; f < n_frames; f+=d_frames, ++row) {
        for (unsigned int i=0; i < n_pts; ++i) {
            ASSERT_FLOAT_EQ(data[row*n_pts+i], truth[f*n_pts+i]) <<
                    "failed for f = " << f << ", i = " << i;
        }
    }

}

/** Test that TranposeH5TestClass is setup as expected */
TEST_F(TransposeH5TestClass, Initialization) {

    ASSERT_TRUE(nodal);
    ASSERT_TRUE(transpose);
    ASSERT_FALSE(structured);

    ASSERT_EQ(n_pts, tri.size());
}

/** Test Reading transpose HDF5, all nodes, blocks of frames */
TEST_F(TransposeH5TestClass, ReadAllNodes) {

    unsigned int f_start = 4; // 0-based
    unsigned int f_delta = 7;
    unsigned int f_end = 123; // 0-based, inclusive

    // Generate truth data
    std::vector<float> truth = load_truth();
    
    // Load the data from the file
    std::vector<float> data;
    upsp::hdf5_read_transpose_solution(filename, 0, n_pts-1, data,
            f_start, f_delta, f_end);

    unsigned int read_frames = (f_end + 1 - f_start + f_delta -1) / f_delta;

    ASSERT_EQ(data.size(), read_frames*n_pts);

    // Load the data 1 frame at a time and compare to the truth data
    for (unsigned int i=0; i < n_pts; ++i) {
        unsigned int col = 0;
        for (unsigned int f=f_start; f < f_end; f+= f_delta, ++col) {
            ASSERT_FLOAT_EQ(data[i*read_frames + col], truth[i*n_frames + f]) <<
                    "failed for i = " << i << ", f = " << f;
        }
    }

}

/** Test Reading transpose HDF5, some nodes, blocks of frames*/
TEST_F(TransposeH5TestClass, ReadSomeNodes) {

    unsigned int f_start = 0; // 0-based
    unsigned int f_delta = 3;
    int f_end = 199; // last frame

    unsigned int n_start = 7;
    unsigned int n_end = 23;

    // Generate truth data
    std::vector<float> truth = load_truth();
    
    // Load the data from the file
    std::vector<float> data;
    upsp::hdf5_read_transpose_solution(filename, n_start, n_end, data,
            f_start, f_delta, f_end);

    unsigned int read_frames = (f_end + 1 - f_start + f_delta -1) / f_delta;
    unsigned int read_pts    = n_end - n_start + 1;

    ASSERT_EQ(data.size(), read_frames*read_pts);

    // Load the data 1 frame at a time and compare to the truth data
    unsigned int row = 0;
    for (unsigned int i=n_start; i < n_end; ++i, ++row) {
        unsigned int col = 0;
        for (unsigned int f=f_start; f < f_end; f+= f_delta, ++col) {
            ASSERT_FLOAT_EQ(data[row*read_frames + col], truth[i*n_frames + f]) <<
                    "failed for i = " << i << ", f = " << f;
        }
    }

}
