add_subdirectory(motor)
pybind11_add_module(motor motor_python.cpp)
target_link_libraries(motor PRIVATE motor_manager)
#install(TARGETS motor LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})