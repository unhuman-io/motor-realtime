cmake_minimum_required(VERSION 3.0)
project(realtime-tmp)
include(GNUInstallDirs)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1, dkms, usb_rt_driver")
set(CPACK_PACKAGE_VERSION 0.4.0)
set(VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_OUTPUT_FILE_PREFIX packages)
configure_file(rt_version.h.in rt_version.h)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA)
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 11)

option(INSTALL_COMPLETION "install bash completion script" ON)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif("${isSystemDir}" STREQUAL "-1")


add_subdirectory(3rdparty)
add_subdirectory(src)

add_subdirectory(example)
install(DIRECTORY example 
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})
