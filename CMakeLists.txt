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
 
add_subdirectory(cpp)

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
