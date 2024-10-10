cmake_minimum_required(VERSION 3.9)
project(motor-realtime)
include(GNUInstallDirs)
enable_testing()

set(VERSION_MAJOR "0")
set(VERSION_MINOR "17")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1")
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "usb_rt_driver")
execute_process(
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMAND bash -c "git describe --dirty --always"
    OUTPUT_VARIABLE GIT_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMAND bash -c "git rev-list ${VERSION_MAJOR}.${VERSION_MINOR}.. --count"
    OUTPUT_VARIABLE GIT_REV OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(${GIT_REV})
    set(CPACK_PACKAGE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${GIT_REV}")
else()
    set(CPACK_PACKAGE_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}")
endif()
set(VERSION "${CPACK_PACKAGE_VERSION}.${GIT_VERSION}")
message("VERSION " ${VERSION})
set(CPACK_OUTPUT_FILE_PREFIX packages)
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
set(CPACK_PACKAGE_FILE_NAME motor-realtime-${ARCH})
configure_file(rt_version.h.in rt_version.h)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA)
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

option(BUILD_SHARED_LIBS "Build libraries shared" ON)
option(BUILD_PYTHON_API "build a python api module" OFF)
option(BUILD_MOTOR_UTIL "build motor util command line program" ON)
option(INSTALL_COMPLETION "install bash completion script" ON)
option(INSTALL_MOTOR_EMBEDDED "install motor embedded" ON)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif("${isSystemDir}" STREQUAL "-1")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything \
    -Wno-c++98-compat-pedantic \
    -Wc++98-compat-extra-semi \
    -Wno-c++20-compat \
    -Wno-declaration-after-statement \
    -Wno-ctad-maybe-unsupported \
    -Wno-weak-template-vtables \
    -Wno-cast-align \
    -Wno-disabled-macro-expansion \
    -Wno-documentation-unknown-command \
    -Wno-documentation \
    -Wno-double-promotion \
    -Wno-duplicate-enum \
    -Wno-exit-time-destructors \
    -Wno-float-equal \
    -Wno-implicit-int-conversion \
    -Wno-missing-prototypes \
    -Wno-missing-variable-declarations \
    -Wno-packed \
    -Wno-padded \
    -Wno-shorten-64-to-32 \
    -Wno-sign-conversion \
    -Wno-strict-prototypes \
    -Wno-switch-enum \
    -Wno-unneeded-member-function \
    -Wno-unused-member-function \
    -Wno-undefined-reinterpret-cast \
    -Wno-implicit-float-conversion \
    -Wno-c++20-designator \
    -Wno-nested-anon-types \
    -Wno-gnu-anonymous-struct \
    -Wno-vla-extension")
endif()


add_subdirectory(3rdparty)
add_subdirectory(src)
add_subdirectory(test)

add_subdirectory(example)
install(DIRECTORY example 
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})
install(DIRECTORY scripts USE_SOURCE_PERMISSIONS
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})
install(DIRECTORY share/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})
