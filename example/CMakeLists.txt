add_executable(deadline deadline.cpp controller.cpp)
target_link_libraries(deadline pthread motor_manager)
