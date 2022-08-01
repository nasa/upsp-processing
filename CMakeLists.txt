cmake_minimum_required(VERSION 3.20)
project(upsp) 
cmake_policy(SET CMP0074 NEW)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH True)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Direct inclusion of open-source tools for
# integrating pandoc w/ CMake, copied into cmake/Modules. Ref:
# https://github.com/jeetsukumaran/cmake-pandocology/
# (SHA: 10900f9aec4431b504fa8979576f950533cf20d9)
include(pandocology)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio tracking)
find_package(OpenMP REQUIRED)
find_package(MPI REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_library(LIBRT rt)
find_package(hdf5 CONFIG REQUIRED)
find_package(IlmBase CONFIG REQUIRED)

pybind11_add_module(cine cpp/pybind11/cine.cpp)
target_link_libraries(cine PRIVATE upsp_video)
pybind11_add_module(raycast cpp/pybind11/raycast.cpp)
target_link_libraries(raycast PRIVATE upsp_kdtree)

include_directories(cpp/include)

add_library(
    upsp_video
    SHARED
    cpp/lib/PSPVideo.cpp
    cpp/lib/MrawReader.cpp
    cpp/lib/CineReader.cpp
)
target_link_libraries(upsp_video PRIVATE opencv_core)
target_link_libraries(upsp_video PRIVATE Eigen3::Eigen)

add_library(
    upsp_utils
    SHARED
    cpp/utils/asyncIO.c
    cpp/utils/cv_extras.cpp
    cpp/utils/file_io.cpp
    cpp/utils/file_readers.cpp
    cpp/utils/file_writers.cpp
    cpp/utils/general_utils.cpp
)
target_link_libraries(upsp_utils PRIVATE opencv_core)
target_link_libraries(upsp_utils PRIVATE ${LIBRT})
target_link_libraries(upsp_utils PRIVATE upsp_video)
target_link_libraries(upsp_utils PRIVATE Eigen3::Eigen)
 
add_library(
    upsp
    SHARED
    cpp/lib/cart3d.cpp
    cpp/lib/image_processing.cpp
    cpp/lib/logging.cpp
    cpp/lib/kulites.cpp
    cpp/lib/non_cv_upsp.cpp
    cpp/lib/patches.cpp
    cpp/lib/plot3d.cpp
    cpp/lib/projection.cpp
    cpp/lib/registration.cpp
    cpp/lib/upsp_inputs.cpp
    cpp/lib/CameraCal.cpp
    cpp/lib/PSPHDF5.cpp
    cpp/lib/P3DModel.cpp
)
target_link_libraries(upsp PUBLIC opencv_core opencv_imgproc opencv_tracking opencv_calib3d)
target_link_libraries(upsp PRIVATE hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)
target_link_libraries(upsp PRIVATE Eigen3::Eigen)

add_library(
    upsp_kdtree
    SHARED
    cpp/utils/pspKdtree.c
    cpp/utils/pspRT.cpp
    cpp/utils/pspRTmem.cpp
)
target_link_libraries(upsp_kdtree IlmBase::Imath)

function(upsp_add_executable NAME)
    add_executable(${NAME} ${ARGN})
    target_link_libraries(${NAME} upsp)
    target_link_libraries(${NAME} upsp_video)
    target_link_libraries(${NAME} upsp_utils)
    target_link_libraries(${NAME} upsp_kdtree)
    target_link_libraries(${NAME} opencv_core)
    target_link_libraries(${NAME} opencv_videoio)
    target_link_libraries(${NAME} opencv_imgproc)
    target_link_libraries(${NAME} OpenMP::OpenMP_CXX)
    target_link_libraries(${NAME} hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)
    target_link_libraries(${NAME} Eigen3::Eigen)
endfunction()

upsp_add_executable(psp_process cpp/exec/psp_process.cpp)
target_link_libraries(psp_process MPI::MPI_CXX)
target_link_libraries(psp_process OpenMP::OpenMP_CXX)
target_link_libraries(psp_process hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)

add_executable(xyz_scalar_to_tbl cpp/exec/xyz_scalar_to_tbl.cpp)
add_executable(xyz_scalar_to_tbl_delta cpp/exec/xyz_scalar_to_tbl_delta.cpp)

upsp_add_executable(upsp-extract-frames cpp/exec/upsp_extract_frames.cpp)

add_executable(add_field cpp/exec/add_field.cpp cpp/utils/asyncIO.c)
target_link_libraries(add_field ${LIBRT})
target_link_libraries(add_field hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)

add_executable(upsp_matrix_transpose cpp/exec/upsp_matrix_transpose.cpp)
target_link_libraries(upsp_matrix_transpose MPI::MPI_CXX)
target_link_libraries(upsp_matrix_transpose OpenMP::OpenMP_CXX)


# GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
# TODO could automate smoke tests and gtest unit tests leveraging CTest
find_package(GTest CONFIG REQUIRED)
upsp_add_executable(
    run_tests
    cpp/test/run_tests.cpp
    cpp/test/test_filtering.cpp
    cpp/test/test_general_utils.cpp
    cpp/test/test_grid_utils.cpp
    cpp/test/test_grids.cpp
    cpp/test/test_models.cpp
    cpp/test/test_mraw.cpp
    cpp/test/test_p3dmodel.cpp
    cpp/test/test_plot3d.cpp
    cpp/test/test_psphdf5.cpp
    cpp/test/test_trimodel.cpp
)
target_link_libraries(run_tests GTest::gtest GTest::gtest_main)
target_link_libraries(run_tests OpenMP::OpenMP_CXX)
target_link_libraries(run_tests hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)

# Documentation (custom cmake macros from pandocology)
# NOTE: pandoc options ref: https://pandoc.org/MANUAL.html
string(TIMESTAMP TODAY "%Y-%m-%d")
function(upsp_add_document TARGET FILENAME)
    list(APPEND UPSP_ADD_DOCUMENT_TARGETS ${TARGET})
    set(UPSP_ADD_DOCUMENT_TARGETS ${UPSP_ADD_DOCUMENT_TARGETS} PARENT_SCOPE)
    get_filename_component(BASENAME_WE ${FILENAME} NAME_WE)
    add_document(
        TARGET ${TARGET}_docx
        OUTPUT_FILE ${BASENAME_WE}.docx
        SOURCES ${FILENAME}
        RESOURCE_DIRS docs/md/static
        PANDOC_DIRECTIVES --to docx
                        --from markdown+pandoc_title_block+table_captions+simple_tables+yaml_metadata_block
                        --filter pandoc-xnos
                        --mathjax
                        --standalone
                        --toc
                        --number-sections
                        --metadata date=${TODAY}
        NO_EXPORT_PRODUCT
    )

    add_document(
        TARGET ${TARGET}_html
        OUTPUT_FILE ${BASENAME_WE}.html
        SOURCES ${FILENAME}
        RESOURCE_DIRS docs/md/static
        PANDOC_DIRECTIVES --to html
                        --from markdown+pandoc_title_block+table_captions+simple_tables+yaml_metadata_block
                        --filter pandoc-xnos
                        --mathjax
                        --standalone
                        --toc
                        --number-sections
                        --metadata date=${TODAY}
                        -c static/upsp-styles.css
                        -A static/upsp-footer.html
        NO_EXPORT_PRODUCT
    )

    add_dependencies(${TARGET}_html ${TARGET}_docx)
endfunction()

function(upsp_serialize_document_depends)
    list(LENGTH UPSP_ADD_DOCUMENT_TARGETS NUMBER_TARGETS)
    if (${NUMBER_TARGETS} LESS_EQUAL 1)
        return()
    endif()
    math(EXPR START "1")
    math(EXPR STOP "${NUMBER_TARGETS} - 1")
    foreach(THIS_IDX RANGE ${START} ${STOP})
        math(EXPR PREV_IDX "${THIS_IDX} - 1")
        list(GET UPSP_ADD_DOCUMENT_TARGETS ${THIS_IDX} THIS_TGT)
        list(GET UPSP_ADD_DOCUMENT_TARGETS ${PREV_IDX} PREV_TGT)
        add_dependencies(${THIS_TGT}_docx ${PREV_TGT}_html)
    endforeach()
endfunction()

upsp_add_document(upsp_user_manual docs/md/upsp-user-manual.md)
upsp_add_document(upsp_swdd docs/md/upsp-swdd.md)
upsp_add_document(upsp_third_party_dependencies docs/md/upsp-third-party-dependencies.md)
upsp_serialize_document_depends()

# INSTALLATION TARGETS

# Documentation install targets
# A bit hacky but it works. Grab outputs from the build directory
# after pandoc has been run (pandoc configured by the various
# cmake pandocology macros) and copy them "by hand" to install location.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-swdd.docx
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-swdd.html
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-third-party-dependencies.docx
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-third-party-dependencies.html
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-user-manual.docx
    ${CMAKE_CURRENT_BINARY_DIR}/upsp-user-manual.html
    DESTINATION docs
)
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/static
    DESTINATION docs
)

install(
    TARGETS
    add_field
    psp_process
    upsp
    upsp-extract-frames
    upsp_kdtree
    upsp_utils
    upsp_video
    xyz_scalar_to_tbl
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    PROGRAMS
    scripts/upsp-external-calibration
    scripts/upsp-make-processing-tree
    scripts/upsp-kulite-comparison
    scripts/upsp-plotting
    scripts/upsp-unity-export
    DESTINATION
    bin
)

# Python install process could be a bit cleaner.
# - Ideally we would make use of setuputils.
# - However, there are a couple modules built with
#   pybind11 (e.g., raycast.so) that rely on other
#   solibs built by the project. If a user wants
#   to import them in Python, then either
#   a) raycast.so has to have its RPATH populated with
#      the path to the folder containing the solib
#      dependencies, or
#   b) the user has to have their LD_LIBRARY_PATH set.
#   Clearly, option (a) is preferred for Python
#   development, and option (b) is an annoying
#   workaround.
# - By using the CMake install() command on the pybind11
#   CMake targets, their RPATH's are updated correctly
#   so option (a) works. However, this precludes using
#   setuputils.
# - There is probably a way to configure CMake via
#   setuputils calls to properly replicate the behavior
#   but I haven't figured it out yet.
install(TARGETS cine raycast LIBRARY DESTINATION python/upsp)

# The rest of the non-pybind11 Python modules are then
# just manually copied here. We explicitly list out all files
# for safety (in general, globs in build scripts aren't best
# practice).
install(
    FILES
    python/upsp/__init__.py
    DESTINATION
    python/upsp
)

install(
    FILES
    python/upsp/processing/context.py
    python/upsp/processing/grids.py
    python/upsp/processing/io.py
    python/upsp/processing/kulite_processing.py
    python/upsp/processing/kulite_utilities.py
    python/upsp/processing/p3d_conversions.py
    python/upsp/processing/p3d_utilities.py
    python/upsp/processing/plot3d.py
    python/upsp/processing/tree.py
    python/upsp/processing/unity_conversions.py
    DESTINATION
    python/upsp/processing
)

install(
    FILES
    python/upsp/cam_cal_utils/external_calibrate.py
    python/upsp/cam_cal_utils/img_utils.py
    python/upsp/cam_cal_utils/parsers.py
    python/upsp/cam_cal_utils/photogrammetry.py
    python/upsp/cam_cal_utils/target_bumping.py
    python/upsp/cam_cal_utils/visibility.py
    python/upsp/cam_cal_utils/visualization.py
    DESTINATION
    python/upsp/cam_cal_utils
)

install(
    FILES
    python/upsp/target_localization/blob_detector_methods.py
    python/upsp/target_localization/gaussian_fitting_methods.py
    DESTINATION
    python/upsp/target_localization
)

install(
    FILES
    python/upsp/kulite_comparison/plotting.py
    python/upsp/kulite_comparison/selection.py
    python/upsp/kulite_comparison/spatial_queries.py
    DESTINATION
    python/upsp/kulite_comparison
)

install(
    FILES
    python/upsp/processing/templates/add-field.sh.template
    python/upsp/processing/templates/gltf-viewer.html.template
    python/upsp/processing/templates/launcher.sh.template
    python/upsp/processing/templates/run-step-parallel.sh.template
    python/upsp/processing/templates/run-step-serial.sh.template
    DESTINATION
    python/upsp/processing/templates
)

# Define the two required variables before including
# the source code for watching a git repository.
set(PRE_CONFIGURE_FILE "scripts/version")
set(POST_CONFIGURE_FILE "${CMAKE_BINARY_DIR}/version")
include(cmake/git_watcher.cmake)

install(
    PROGRAMS
    ${CMAKE_BINARY_DIR}/version
    DESTINATION
    ${CMAKE_INSTALL_PREFIX}
)

install(
    FILES
    scripts/activate.sh
    scripts/activate.csh
    RELEASE.md
    DESTINATION
    ${CMAKE_INSTALL_PREFIX}
)

