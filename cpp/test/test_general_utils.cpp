/** @file
 *  @brief  Test the utils/general_utils.h functions
 *  @date   August 12, 2019
 *  @author jmpowel2
 */
#include <vector>

#include "gtest/gtest.h"

#include "../include/utils/general_utils.h"

/* TODO: Still need tests for:
 *
 * CircularArray
 * factorial
 * split_string
 * split_whitespace
 * rad2_deg
 * deg2_rad
 * min_ind
 * max_ind
 * min
 * max
 * erase_indices
 * round_up
 * round_down
 */

/** Test transpose scalar block */
TEST(GeneralUtilsTest, TransposeScalarBlock) {

    const unsigned int rows = 200;
    const unsigned int cols = 100;

    unsigned int block_size = 16;

    float A[rows][cols];
    float B[cols][rows];

    // initialize matrix
    for (unsigned int i=0; i < rows; ++i) {
        for (unsigned int j=0; j < cols; ++j) {
            A[i][j] = (i+1) * (j+2);
        }
    }
    
    // transpose initial block
    transpose_scalar_block((float*)A, (float*)B, rows, cols, block_size);

    for (unsigned int i=0; i < block_size; ++i) {
        for (unsigned int j=0; j < block_size; ++j) {
            ASSERT_EQ(A[i][j], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
            ASSERT_EQ(B[j][i], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
        }
    }

    // transpose a different block
    unsigned int offset_rows = 20;
    unsigned int offset_cols = 50;

    transpose_scalar_block((float*) &A[offset_rows][offset_cols], 
                           (float*) &B[offset_cols][offset_rows], 
                           rows, cols, block_size);

    for (unsigned int i=offset_rows; i < offset_rows+block_size; ++i) {
        for (unsigned int j=offset_cols; j < offset_cols+block_size; ++j) {
            ASSERT_EQ(A[i][j], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
            ASSERT_EQ(B[j][i], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
        }
    }
}

/** Test transpose matrix with blocks */
TEST(GeneralUtilsTest, TransposeBlock) {

    const unsigned int rows = 200;
    const unsigned int cols = 100;

    unsigned int block_size = 16;

    float A[rows][cols];
    float B[cols][rows];

    // initialize matrix
    for (unsigned int i=0; i < rows; ++i) {
        for (unsigned int j=0; j < cols; ++j) {
            A[i][j] = (i+1) * (j+2);
        }
    }

    // transpose
    transpose_block((float*) A, (float*) B, rows, cols, block_size);

    for (unsigned int i=0; i < rows; ++i) {
        for (unsigned int j=0; j < cols; ++j) {
            ASSERT_EQ(A[i][j], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
            ASSERT_EQ(B[j][i], (i+1) * (j+2)) << "i = " << i << ", j = " << j;
        }
    }

}
