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

install(
    TARGETS
    # cine
    raycast
    LIBRARY DESTINATION ${UPSP_INSTALL_LIB_DIR}
    RUNTIME DESTINATION ${UPSP_INSTALL_LIB_DIR}
)
