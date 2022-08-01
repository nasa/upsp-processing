/** @file
 *  @brief  Run all Tests
 *  @date   April 26, 2019
 *  @author jmpowel2
 */

#include "gtest/gtest.h"

// todo-mshawlec: Disabling some unit tests that cover code
//                we aren't using in core psp_process because
//                they take a non-trivial amount of time to
//                run. Can re-enable as necessary if/once we
//                make use of these libraries.
// #include "test_integration.h"
// #include "test_interpolation.h"
// #include "test_octree.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
