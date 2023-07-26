include_directories(include)

option(UPSP_BUILD_APPLICATIONS "Build applications" OFF)
option(UPSP_BUILD_PYBIND11 "Build Python bindings" OFF)
option(UPSP_BUILD_TESTING "Build unit tests" OFF)

if (UPSP_BUILD_APPLICATIONS)
    add_subdirectory(exec)
    add_subdirectory(lib)
    add_subdirectory(utils)
    add_subdirectory(raycast)
endif()

if (UPSP_BUILD_PYBIND11)
    if (NOT UPSP_BUILD_APPLICATIONS)
        add_subdirectory(raycast)
    endif()
    add_subdirectory(pybind11)
endif()

if (UPSP_BUILD_TESTING)
    add_subdirectory(test)
endif()
