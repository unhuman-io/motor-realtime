cmake_minimum_required(VERSION 3.0)
project(realtime-tmp)
include(GNUInstallDirs)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1, dkms, usb_rt_driver")
execute_process(COMMAND bash -c "git describe --dirty --always --tags" OUTPUT_VARIABLE GIT_VERSION)
string(STRIP ${GIT_VERSION} GIT_VERSION)
execute_process(COMMAND bash -c "git describe --dirty --long --tags" OUTPUT_VARIABLE GIT_VERSION_LONG)
string(STRIP ${GIT_VERSION_LONG} GIT_VERSION_LONG)
execute_process(COMMAND bash -c "git rev-list 0.. --count" OUTPUT_VARIABLE GIT_REV)
string(STRIP ${GIT_REV} GIT_REV)
set(CPACK_PACKAGE_VERSION "0.4.${GIT_REV}")
set(VERSION ${GIT_VERSION_LONG})
set(CPACK_OUTPUT_FILE_PREFIX packages)
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
set(CPACK_PACKAGE_FILE_NAME realtime-tmp-${ARCH})
configure_file(rt_version.h.in rt_version.h)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA)
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 11)

option(INSTALL_COMPLETION "install bash completion script" ON)
option(BUILD_PYTHON_API "build a python api module" OFF)

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
