execute_process(COMMAND uname -r
OUTPUT_VARIABLE os_version
OUTPUT_STRIP_TRAILING_WHITESPACE)
set(module_path /lib/modules/${os_version})
set(module_build_path ${module_path}/build)
add_custom_command(OUTPUT usb_skeleton.ko
                    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${module_build_path} 
                        M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR}
                        EXTRA_CFLAGS=-I${CMAKE_BINARY_DIR}
                    DEPENDS usb-skeleton.c Kbuild
                    COMMENT "Building usb_skeleton.ko")
add_custom_target(usb_skeleton ALL DEPENDS usb_skeleton.ko)
configure_file(dkms.conf.in dkms.conf)

install(FILES usb-skeleton.c Kbuild 
        ${CMAKE_BINARY_DIR}/rt_version.h
        ${CMAKE_CURRENT_BINARY_DIR}/dkms.conf DESTINATION /usr/src/usb_skeleton-${VERSION})