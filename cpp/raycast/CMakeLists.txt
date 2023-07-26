
# IlmBase library is deprecated in some cases, in which case
# we should depend on Imath directly.
find_package(Imath CONFIG)
if (Imath_FOUND)
  add_library(IlmBase::Imath ALIAS Imath::Imath)
  add_library(IlmBase::ImathConfig ALIAS Imath::ImathConfig)
else()
  find_package(IlmBase CONFIG REQUIRED)
endif()

add_library(
    upsp_kdtree
    SHARED
    pspKdtree.c
    pspRT.cpp
    pspRTmem.cpp
)
target_link_libraries(upsp_kdtree PUBLIC IlmBase::Imath)

# raycast extension module handles installation to site-packages
if(UPSP_BUILD_APPLICATIONS)
  install(
    TARGETS upsp_kdtree
    LIBRARY DESTINATION ${UPSP_INSTALL_APP_LIB_DIR}
    RUNTIME DESTINATION ${UPSP_INSTALL_BIN_DIR}
  )
endif()