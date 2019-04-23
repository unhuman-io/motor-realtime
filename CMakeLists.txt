cmake_minimum_required(VERSION 3.12)
project(realtime)


add_library(motor_manager motor_manager.cpp)
target_link_libraries(motor_manager udev)

add_executable(motor_util motor_util.cpp)
target_link_libraries(motor_util motor_manager)

add_executable(deadline deadline.cpp)
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