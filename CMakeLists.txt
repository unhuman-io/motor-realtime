cmake_minimum_required(VERSION 3.0)
project(realtime)

SET(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Lee Magnusson")
set(CPACK_DEBIAN_PACKAGE_DEPENDS libudev1)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA postinst)
INCLUDE(CPack)

set (CMAKE_CXX_STANDARD 11)

add_library(motor_manager motor_manager.cpp)
target_link_libraries(motor_manager udev)

add_executable(motor-util motor_util.cpp)
target_link_libraries(motor-util motor_manager)
install(TARGETS motor-util DESTINATION bin)

add_executable(deadline deadline.cpp controller.cpp)
target_link_libraries(deadline pthread motor_manager udev)

execute_process(COMMAND uname -r
OUTPUT_VARIABLE os_version
OUTPUT_STRIP_TRAILING_WHITESPACE)
set(module_path /lib/modules/${os_version})
set(module_build_path ${module_path}/build)
add_custom_command(OUTPUT usb_skeleton.ko
                    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${module_build_path} 
                        M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR}
                    DEPENDS usb-skeleton.c Kbuild
                    COMMENT "Building usb_skeleton.ko")
add_custom_target(usb_skeleton ALL DEPENDS usb_skeleton.ko)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/usb-skeleton.ko DESTINATION /lib/modules/${os_version})