cmake_minimum_required(VERSION 3.0)
project(realtime-tmp)
include(GNUInstallDirs)

SET(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1, dkms, usb_rt_driver")
set(CPACK_PACKAGE_VERSION 0.3.0)
set(VERSION ${CPACK_PACKAGE_VERSION})
configure_file(rt_version.h.in rt_version.h)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA 
    )
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 11)

add_subdirectory(3rdparty)
add_subdirectory(src)

add_subdirectory(example)
install(DIRECTORY example 
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})
