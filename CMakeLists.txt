cmake_minimum_required(VERSION 3.12)
project(realtime)


add_library(motor_manager motor_manager.cpp)
target_link_libraries(motor_manager udev)

add_executable(motor_util motor_util.cpp)
target_link_libraries(motor_util motor_manager)

add_executable(deadline deadline.cpp)
target_link_libraries(deadline pthread motor_manager udev)

