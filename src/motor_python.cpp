#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor_manager.h"

namespace py = pybind11;

PYBIND11_MODULE(motor, m) {
    m.doc() = "Motor interface";
    py::class_<MotorManager>(m, "MotorManager")
        .def(py::init<>())
        .def("__repr__", [](const MotorManager &m){ 
            std::string s;
            for (auto motor : m.motors()) {
                s += motor->name() + " ";
            }
            return "<MotorManager connected to: " + s + ">"; })
        .def("get_connected_motors", &MotorManager::get_connected_motors);
    py::class_<Motor, std::shared_ptr<Motor>>(m, "Motor")
        .def(py::init<const std::string&>())
        .def("name", &Motor::name);
}