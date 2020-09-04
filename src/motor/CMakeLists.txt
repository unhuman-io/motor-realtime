add_library(motor_manager SHARED motor_manager.cpp motor.cpp realtime_thread.cpp motor_thread.cpp motor_app.cpp)
target_link_libraries(motor_manager udev)
target_include_directories(motor_manager PUBLIC ${CMAKE_SOURCE_DIR}/include)
set(MOTOR_MANAGER_PUBLIC_HEADERS 
    ${CMAKE_SOURCE_DIR}/include/motor_manager.h
    ${CMAKE_SOURCE_DIR}/include/motor.h
    ${CMAKE_SOURCE_DIR}/include/realtime_thread.h
    ${CMAKE_SOURCE_DIR}/include/motor_thread.h
    ${CMAKE_SOURCE_DIR}/include/motor_app.h
    ${CMAKE_SOURCE_DIR}/include/motor_publisher.h
    ${CMAKE_SOURCE_DIR}/include/motor_subscriber.h)
set_target_properties(motor_manager PROPERTIES PUBLIC_HEADER 
    "${MOTOR_MANAGER_PUBLIC_HEADERS}")
install(TARGETS motor_manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_executable(motor_util motor_util.cpp)
target_link_libraries(motor_util motor_manager cli11 pthread rt)
target_include_directories(motor_util PUBLIC ${CMAKE_BINARY_DIR})

add_executable(motor_data_echo motor_data_echo.cpp)
target_include_directories(motor_data_echo PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(motor_data_echo rt pthread)

install(TARGETS motor_util DESTINATION bin)
# can't seem to use ${CMAKE_INSTALL_SYSCONFDIR} below instead of /etc with cpack
if(INSTALL_COMPLETION)
    install(FILES motor_util_completion.bash DESTINATION /etc/bash_completion.d)
endif()