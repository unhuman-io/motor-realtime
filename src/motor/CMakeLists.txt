add_library(motor_manager motor_manager.cpp motor.cpp realtime_thread.cpp motor_thread.cpp motor_app.cpp)
target_link_libraries(motor_manager udev pthread)
target_include_directories(motor_manager PUBLIC ${CMAKE_SOURCE_DIR}/include)
set(MOTOR_MANAGER_PUBLIC_HEADERS 
    ${CMAKE_SOURCE_DIR}/include/motor_manager.h
    ${CMAKE_SOURCE_DIR}/include/motor_messages.h
    ${CMAKE_SOURCE_DIR}/include/motor.h
    ${CMAKE_SOURCE_DIR}/include/realtime_thread.h
    ${CMAKE_SOURCE_DIR}/include/motor_thread.h
    ${CMAKE_SOURCE_DIR}/include/motor_app.h
    ${CMAKE_SOURCE_DIR}/include/motor_publisher.h
    ${CMAKE_SOURCE_DIR}/include/motor_subscriber.h
    ${CMAKE_SOURCE_DIR}/include/cstack.h)
set_target_properties(motor_manager PROPERTIES PUBLIC_HEADER 
    "${MOTOR_MANAGER_PUBLIC_HEADERS}")
install(TARGETS motor_manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_SOURCE_DIR}/include/motor_messages/motor_messages.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/motor_messages)

if(BUILD_MOTOR_UTIL)
    add_executable(motor_util motor_util.cpp)
    target_link_libraries(motor_util motor_manager cli11 pthread rt)
    target_include_directories(motor_util PUBLIC ${CMAKE_BINARY_DIR})
    install(TARGETS motor_util DESTINATION bin)
endif()

add_executable(motor_data_echo motor_data_echo.cpp)
target_include_directories(motor_data_echo PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(motor_data_echo rt pthread)


# can't seem to use ${CMAKE_INSTALL_SYSCONFDIR} below instead of /etc with cpack
if(INSTALL_COMPLETION)
    install(FILES motor_util_completion.bash DESTINATION /etc/bash_completion.d)
endif()

if(BUILD_MOTOR_SERVER)
    set(_PROTOBUF_LIBPROTOBUF libprotobuf)
    set(_REFLECTION grpc++_reflection)
    set(_PROTOBUF_PROTOC $<TARGET_FILE:protoc>)
    set(_GRPC_GRPCPP grpc++)
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)

    set(motor_server_proto_srcs ${CMAKE_CURRENT_BINARY_DIR}/motor_server.pb.cc)
    set(motor_server_proto_hdrs ${CMAKE_CURRENT_BINARY_DIR}/motor_server.pb.h)
    set(motor_server_grpc_srcs ${CMAKE_CURRENT_BINARY_DIR}/motor_server.grpc.pb.cc)
    set(motor_server_grpc_hdrs ${CMAKE_CURRENT_BINARY_DIR}/motor_server.grpc.pb.h)
    add_custom_command(
    OUTPUT "${motor_server_proto_srcs}" "${motor_server_proto_hdrs}" "${motor_server_grpc_srcs}" "${motor_server_grpc_hdrs}"
    COMMAND ${_PROTOBUF_PROTOC}
    ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    -I ${CMAKE_SOURCE_DIR}/protos
    --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
    "${CMAKE_SOURCE_DIR}/protos/motor_server.proto"
    DEPENDS "${CMAKE_SOURCE_DIR}/protos/motor_server.proto")

    add_library(motor_server_proto ${motor_server_proto_srcs} ${motor_server_proto_hdrs} ${motor_server_grpc_srcs} ${motor_server_grpc_hdrs})
    target_link_libraries(motor_server_proto
        ${_REFLECTION}
        ${_GRPC_GRPCPP}
        ${_PROTOBUF_LIBPROTOBUF})
    target_include_directories(motor_server_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    add_executable(motor_server motor_server.cpp)
    target_link_libraries(motor_server motor_manager cli11 grpc++ motor_server_proto)
    target_include_directories(motor_server PRIVATE ${CMAKE_BINARY_DIR})
    install(TARGETS motor_server DESTINATION bin)
endif()