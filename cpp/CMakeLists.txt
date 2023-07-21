include_directories(include)

option(UPSP_BUILD_APPLICATIONS "Build applications" OFF)
option(UPSP_BUILD_PYBIND11 "Build Python bindings" OFF)
option(UPSP_BUILD_TESTING "Build unit tests" OFF)

if (UPSP_BUILD_APPLICATIONS)
    add_subdirectory(exec)
    add_subdirectory(lib)
    add_subdirectory(utils)
endif()

if (UPSP_BUILD_PYBIND11)
    add_subdirectory(raycast)
    add_subdirectory(pybind11)
endif()

if (UPSP_BUILD_TESTING)
    add_subdirectory(test)
endif()
