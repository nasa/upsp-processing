find_package(Boost REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(MPI REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core videoio)
find_package(OpenMP REQUIRED)
find_package(hdf5 CONFIG REQUIRED)

add_executable(psp_process psp_process.cpp)
target_link_libraries(psp_process upsp)
target_link_libraries(psp_process upsp_video)
target_link_libraries(psp_process upsp_utils)
target_link_libraries(psp_process upsp_kdtree)
target_link_libraries(psp_process MPI::MPI_CXX)
target_link_libraries(psp_process OpenMP::OpenMP_CXX)
target_link_libraries(psp_process hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)
target_link_libraries(psp_process Eigen3::Eigen)

add_executable(xyz_scalar_to_tbl xyz_scalar_to_tbl.cpp)

add_executable(xyz_scalar_to_tbl_delta xyz_scalar_to_tbl_delta.cpp)

add_executable(upsp-extract-frames upsp_extract_frames.cpp)
target_link_libraries(upsp-extract-frames upsp_video)
target_link_libraries(upsp-extract-frames opencv_core)
target_link_libraries(upsp-extract-frames opencv_videoio)
target_include_directories(upsp-extract-frames SYSTEM PRIVATE ${Boost_INCLUDE_DIR})

add_executable(add_field add_field.cpp)
target_link_libraries(add_field upsp_video)
target_link_libraries(add_field upsp_utils)
target_link_libraries(add_field hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)

add_executable(upsp_matrix_transpose upsp_matrix_transpose.cpp)
target_link_libraries(upsp_matrix_transpose MPI::MPI_CXX)
target_link_libraries(upsp_matrix_transpose OpenMP::OpenMP_CXX)

install(
    TARGETS
    add_field
    psp_process
    upsp-extract-frames
    xyz_scalar_to_tbl
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
