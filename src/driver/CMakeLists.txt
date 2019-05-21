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
