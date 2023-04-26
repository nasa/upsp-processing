# Custom triplet for building shared libraries (*.so) for
# x64 linux architectures. Default for vcpkg is to build
# static libraries (*.a) only.

set(VCPKG_TARGET_ARCHITECTURE x64)

set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)

set(VCPKG_CMAKE_SYSTEM_NAME Linux)

set(VCPKG_CXX_FLAGS "-Wl,-rpath,'$ORIGIN'")
set(VCPKG_C_FLAGS "-Wl,-rpath,'$ORIGIN'")
set(VCPKG_LINKER_FLAGS "-Wl,-rpath,'$ORIGIN'")

# use, i.e. don't skip the full RPATH for the build tree
# when building, don't use the install RPATH already
# (but later on when installing)
# set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

#set(CMAKE_SKIP_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH

# # the RPATH to be used when installing, but only if it's not a system directory
# list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
# if("${isSystemDir}" STREQUAL "-1")
#     set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# endif("${isSystemDir}" STREQUAL "-1")

