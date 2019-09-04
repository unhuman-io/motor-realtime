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

add_executable(motor_util motor_util.cpp)
target_link_libraries(motor_util motor_manager)
target_include_directories(motor_util PUBLIC ${CMAKE_BINARY_DIR})
install(TARGETS motor_util DESTINATION bin)