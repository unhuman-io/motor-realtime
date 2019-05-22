cmake_minimum_required(VERSION 3.0)
project(realtime-tmp)
include(GNUInstallDirs)

SET(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS libudev1)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA 
    ${CMAKE_SOURCE_DIR}/postinst)
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 11)

add_subdirectory(src)

add_subdirectory(example)
install(DIRECTORY example 
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME})

install(FILES share/99-realtime-tmp.rules
    DESTINATION /etc/udev/rules.d)