find_package(GTest CONFIG REQUIRED)

add_executable(
    run_tests
    run_tests.cpp
    test_filtering.cpp
    test_general_utils.cpp
    test_grid_utils.cpp
    test_grids.cpp
    test_models.cpp
    test_mraw.cpp
    test_p3dmodel.cpp
    test_plot3d.cpp
    test_psphdf5.cpp
    test_trimodel.cpp
)

target_link_libraries(run_tests PRIVATE GTest::gtest GTest::gtest_main)
target_link_libraries(run_tests PRIVATE upsp upsp_utils upsp_video upsp_kdtree)
