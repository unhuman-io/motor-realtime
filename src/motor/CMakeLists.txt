add_library(motor_manager SHARED motor_manager.cpp)
target_link_libraries(motor_manager udev)
target_include_directories(motor_manager PUBLIC ${CMAKE_SOURCE_DIR}/include)
set(MOTOR_MANAGER_PUBLIC_HEADERS 
    ${CMAKE_SOURCE_DIR}/include/motor_manager.h
    ${CMAKE_SOURCE_DIR}/include/motor.h)
set_target_properties(motor_manager PROPERTIES PUBLIC_HEADER 
    "${MOTOR_MANAGER_PUBLIC_HEADERS}")
install(TARGETS motor_manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_executable(motor-util motor_util.cpp)
target_link_libraries(motor-util motor_manager)
install(TARGETS motor-util DESTINATION bin)