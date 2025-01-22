cmake_minimum_required(VERSION 3.15...3.29)
project(motor_pub_sub LANGUAGES CXX)

list(APPEND CMAKE_PREFIX_PATH "../../../3rdparty")
add_subdirectory("../../../3rdparty/pybind11" "pybind11")

pybind11_add_module(motor_pub_sub motor_pub_sub.cpp)