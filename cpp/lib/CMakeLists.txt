find_package(Boost REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(MPI REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio tracking)
find_package(OpenMP REQUIRED)
find_package(hdf5 CONFIG REQUIRED)

add_library(
    upsp_video
    SHARED
    PSPVideo.cpp
    MrawReader.cpp
    CineReader.cpp
)
target_link_libraries(upsp_video PRIVATE opencv_core)
target_link_libraries(upsp_video PRIVATE Eigen3::Eigen)
 
add_library(
    upsp
    SHARED
    cart3d.cpp
    image_processing.cpp
    logging.cpp
    kulites.cpp
    non_cv_upsp.cpp
    patches.cpp
    plot3d.cpp
    projection.cpp
    registration.cpp
    upsp_inputs.cpp
    CameraCal.cpp
    PSPHDF5.cpp
    P3DModel.cpp
)
target_link_libraries(upsp PUBLIC opencv_core opencv_imgproc opencv_tracking opencv_calib3d)
target_link_libraries(upsp PUBLIC hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)
target_link_libraries(upsp PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(upsp PUBLIC Eigen3::Eigen)

install(
    TARGETS
    upsp
    upsp_video
    LIBRARY DESTINATION ${UPSP_CPP_INSTALL_LIB_DIR}
    RUNTIME DESTINATION ${UPSP_CPP_INSTALL_BIN_DIR}
)
