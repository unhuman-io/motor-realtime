cmake_minimum_required(VERSION 3.12)
project(realtime)


add_library(motor_manager motor_manager.cpp)
target_link_libraries(motor_manager udev)

add_executable(motor_util motor_util.cpp)
target_link_libraries(motor_util motor_manager)

add_executable(deadline deadline.cpp)
target_link_libraries(deadline pthread motor_manager udev)

add_executable(main main.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/../libusb/libusb)
target_link_libraries(main ${CMAKE_SOURCE_DIR}/../libusb/libusb/.libs/libusb-1.0.a udev pthread)
