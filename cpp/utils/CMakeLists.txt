find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio tracking)
find_library(LIBRT rt)

add_library(
    upsp_utils
    SHARED
    asyncIO.c
    cv_extras.cpp
    file_io.cpp
    file_readers.cpp
    file_writers.cpp
    general_utils.cpp
)
target_link_libraries(upsp_utils PUBLIC opencv_core opencv_imgproc)
target_link_libraries(upsp_utils PUBLIC upsp_video)
target_link_libraries(upsp_utils PUBLIC ${LIBRT})

install(
    TARGETS upsp_utils
    LIBRARY DESTINATION ${UPSP_CPP_INSTALL_LIB_DIR}
    RUNTIME DESTINATION ${UPSP_CPP_INSTALL_BIN_DIR}
)
