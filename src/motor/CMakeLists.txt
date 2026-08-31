add_library(motor_manager motor_manager.cpp motor.cpp motor_ip.cpp motor_uart.cpp motor_uart_obot.cpp motor_can.cpp
            motor_eth_l2.cpp
            realtime_thread.cpp motor_thread.cpp motor_app.cpp motor_util_fun.cpp terminal.cpp gdbserver.cpp)
target_link_libraries(motor_manager PUBLIC udev pthread cli11 obot-protocol)
target_include_directories(motor_manager PUBLIC ${PROJECT_SOURCE_DIR}/include)
if (ENABLE_IO_URING)
    target_sources(motor_manager PRIVATE motor_manager_uring.cpp)
    target_link_libraries(motor_manager PRIVATE uring)
endif()

set(MOTOR_MANAGER_PUBLIC_HEADERS 
    ${PROJECT_SOURCE_DIR}/include/motor_manager.h
    ${PROJECT_SOURCE_DIR}/include/motor_messages.h
    ${PROJECT_SOURCE_DIR}/include/motor.h
    ${PROJECT_SOURCE_DIR}/include/motor_ip.h
    ${PROJECT_SOURCE_DIR}/include/motor_uart.h
    ${PROJECT_SOURCE_DIR}/include/motor_uart_obot.h
    ${PROJECT_SOURCE_DIR}/include/motor_can.h
    ${PROJECT_SOURCE_DIR}/include/motor_eth_l2.h
    ${PROJECT_SOURCE_DIR}/include/realtime_thread.h
    ${PROJECT_SOURCE_DIR}/include/motor_thread.h
    ${PROJECT_SOURCE_DIR}/include/motor_app.h
    ${PROJECT_SOURCE_DIR}/include/motor_publisher.h
    ${PROJECT_SOURCE_DIR}/include/motor_subscriber.h
    ${PROJECT_SOURCE_DIR}/include/motor_chain_messages.h
    ${PROJECT_SOURCE_DIR}/include/motor_util_fun.h
    ${PROJECT_SOURCE_DIR}/include/cstack.h
    ${PROJECT_SOURCE_DIR}/include/terminal.h
    ${PROJECT_SOURCE_DIR}/include/exception.h
    )
set_target_properties(motor_manager PROPERTIES PUBLIC_HEADER 
    "${MOTOR_MANAGER_PUBLIC_HEADERS}")
if(BUILD_SHARED_LIBS)
    install(TARGETS motor_manager
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(FILES ${PROJECT_SOURCE_DIR}/include/motor_messages/motor_messages.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/motor_messages)

add_executable(eth2vcan eth2vcan.cpp)
target_link_libraries(eth2vcan cli11 motor_manager)
install(TARGETS eth2vcan DESTINATION bin)

if(BUILD_MOTOR_UTIL)
    add_executable(motor_util motor_util.cpp )
    target_link_libraries(motor_util motor_manager json)
    target_include_directories(motor_util PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/../../)
    install(TARGETS motor_util DESTINATION bin)

    add_executable(motor_usbmon motor_usbmon.cpp)
    target_link_libraries(motor_usbmon motor_manager)
    install(TARGETS motor_usbmon DESTINATION bin)
endif()

add_custom_target(setcap
    COMMAND sudo setcap cap_net_raw+ep eth2vcan
    COMMAND sudo setcap cap_net_raw+ep motor_util)
add_dependencies(setcap eth2vcan motor_util)

add_executable(motor_data_echo motor_data_echo.cpp)
target_include_directories(motor_data_echo PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(motor_data_echo rt pthread)


# can't seem to use ${CMAKE_INSTALL_SYSCONFDIR} below instead of /etc with cpack
if(INSTALL_COMPLETION)
    install(FILES motor_util_completion.bash DESTINATION /etc/bash_completion.d)
endif()

# a more minimal library
if(INSTALL_MOTOR_EMBEDDED)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_manager.h motor_manager_embedded/motor_manager.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor.h motor_manager_embedded/motor.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_ip.h motor_manager_embedded/motor_ip.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_eth_l2.h motor_manager_embedded/motor_eth_l2.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_uart.h motor_manager_embedded/motor_uart.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_uart_obot.h motor_manager_embedded/motor_uart_obot.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_can.h motor_manager_embedded/motor_can.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_util_fun.h motor_manager_embedded/motor_util_fun.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_messages.h motor_manager_embedded/motor_messages.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/motor_messages/motor_messages.h motor_manager_embedded/motor_messages/motor_messages.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/terminal.h motor_manager_embedded/terminal.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/include/exception.h motor_manager_embedded/exception.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/3rdparty/obot-protocol/crc.h motor_manager_embedded/crc.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/3rdparty/obot-protocol/protocol_parser.h motor_manager_embedded/protocol_parser.h COPYONLY)
    configure_file(${PROJECT_SOURCE_DIR}/3rdparty/obot-protocol/protocol_parser.cpp motor_manager_embedded/protocol_parser.cpp COPYONLY)
    configure_file(motor_manager.cpp motor_manager_embedded/motor_manager.cpp COPYONLY)
    configure_file(motor.cpp motor_manager_embedded/motor.cpp COPYONLY)
    configure_file(motor_ip.cpp motor_manager_embedded/motor_ip.cpp COPYONLY)
    configure_file(motor_eth_l2.cpp motor_manager_embedded/motor_eth_l2.cpp COPYONLY)
    configure_file(motor_uart.cpp motor_manager_embedded/motor_uart.cpp COPYONLY)
    configure_file(motor_uart_obot.cpp motor_manager_embedded/motor_uart_obot.cpp COPYONLY)
    configure_file(motor_can.cpp motor_manager_embedded/motor_can.cpp COPYONLY)
    configure_file(terminal.cpp motor_manager_embedded/terminal.cpp COPYONLY)
    configure_file(motor_manager_embedded/CMakeLists.txt motor_manager_embedded/CMakeLists.txt COPYONLY)
    configure_file(motor_manager_embedded/BUILD motor_manager_embedded/BUILD COPYONLY)
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/motor_manager_embedded ${CMAKE_CURRENT_BINARY_DIR}/motor_manager_embedded_build)
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/motor_manager_embedded DESTINATION share)
endif()
