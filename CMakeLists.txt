cmake_minimum_required(VERSION 3.20)
project(upsp) 
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

# This ensures any link paths used at build time are preserved in
# the installed targets. This supports cases where the user has
# supplied third-party libraries in non-system locations (for
# instance, if making use of a local vcpkg setup)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)

if (DEFINED SKBUILD)
  # prevent an unused variable warning
  set(ignoreMe "${SKBUILD}")

  # TODO this is ugly but it works for now.
  #
  # When the project is built by scikit-build for deployment as a
  # Python package, libraries are dropped in the lib/ folder and there's
  # multiple consumers of those libraries that need to resolve them at runtime:
  #
  # - Executables, installed to bin/, need to look in $ORIGIN/../lib
  # - Python extension modules, installed to lib/pythonX/site-packages/upsp,
  #   need to look in $ORIGIN/../../..
  #
  # Lastly, there's some build-tree-only artifacts (eg gtest unit tests),
  # those need to look in their own build directory ($ORIGIN).
  #
  # We could do this on a per-target basis but I'd rather just leave
  # the hack in one spot for now instead of being peppered all over
  # this build file.
  #
  # A "better/cleaner" example of this setup can be found here (although
  # I don't think it handles all the use cases I listed above, it's
  # a small sample project):
  #
  # https://github.com/scikit-build/scikit-build-sample-projects/blob/master/projects/hello-cmake-package/CMakeLists.txt#L92
  #
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH "\$ORIGIN:\$ORIGIN/../lib:\$ORIGIN/../../../")
endif()
 
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
find_package(PythonExtensions)

include_directories(cpp/include)

pybind11_add_module(cine cpp/pybind11/cine.cpp)
target_link_libraries(cine PRIVATE upsp_video)

pybind11_add_module(raycast cpp/pybind11/raycast.cpp)
target_link_libraries(raycast PRIVATE upsp_kdtree)

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
    scripts/upsp-qsub-args
    DESTINATION
    bin
)

# TODO I think this works in a fairly brittle fashion right now.
# It just so happens that the cmake_install_dir staging directory
# created by scikit-build has a "python/upsp" folder mimicking the
# source tree layout... so these extensions are just "dropped" into the
# right spot. There's probably a more robust way to specify their
# install location, probably some CMAKE_ variable I'm ignorant of.
install(
    TARGETS
    cine
    raycast
    LIBRARY DESTINATION python/upsp
)
