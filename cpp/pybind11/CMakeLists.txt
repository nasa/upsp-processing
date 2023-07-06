find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# Ref: https://pybind11.readthedocs.io/en/stable/cmake/index.html
# "Python method" seems to be what works w/ skbuild, otherwise the
# built solib doesn't have SOABI filename extension.
python_add_library(raycast MODULE raycast.cpp WITH_SOABI)
target_link_libraries(raycast PRIVATE pybind11::headers upsp_kdtree)
set_target_properties(
    raycast
    PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON
    VISIBILITY_INLINES_HIDDEN ON
)
target_compile_definitions(raycast PRIVATE VERSION_INFO=${PROJECT_VERSION})

# TODO I think this works in a fairly brittle fashion right now.
# It just so happens that the cmake_install_dir staging directory
# created by scikit-build has a "python/upsp" folder mimicking the
# source tree layout... so these extensions are just "dropped" into the
# right spot. There's probably a more robust way to specify their
# install location, probably some CMAKE_ variable I'm ignorant of.
install(
    TARGETS
    # cine
    raycast
    upsp_kdtree
    LIBRARY DESTINATION ${SKBUILD_PROJECT_NAME}
    RUNTIME DESTINATION ${SKBUILD_PROJECT_NAME}
)
