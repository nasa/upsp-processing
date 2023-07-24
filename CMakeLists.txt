cmake_minimum_required(VERSION 3.20)
project(upsp) 
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if (MSVC)
  # For MSVC to get things like "or", "and", ...
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-")
endif()

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
  # Hard-code the appropriate library RPATHs, handling all possible "install" use cases.
  # - Installing libs for use in "standard" C/C++ <prefix>/lib install
  # - Installing libs for use in Python extensions
  #
  # Exact rpath syntax depends on target arch.
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  if (APPLE)
    set(CMAKE_INSTALL_RPATH "@loader_path/;@loader_path/../../../")
  elseif (LINUX)
    set(CMAKE_INSTALL_RPATH "\$ORIGIN:\$ORIGIN/../lib:\$ORIGIN/../../../")
  endif()

  set(UPSP_INSTALL_BIN_DIR ${SKBUILD_SCRIPTS_DIR})
  set(UPSP_INSTALL_LIB_DIR ${SKBUILD_PROJECT_NAME})
else()
  set(UPSP_INSTALL_BIN_DIR "bin")
  set(UPSP_INSTALL_LIB_DIR "lib")
endif()

add_subdirectory(cpp)

install(
    PROGRAMS
    scripts/upsp-external-calibration
    scripts/upsp-make-processing-tree
    scripts/upsp-kulite-comparison
    scripts/upsp-plotting
    scripts/upsp-qsub-args
    DESTINATION ${UPSP_INSTALL_BIN_DIR}
)
