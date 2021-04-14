#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "motor_manager.h"

namespace py = pybind11;

PYBIND11_MODULE(motor, m) {
    m.doc() = "Motor interface";
    py::class_<MotorManager>(m, "MotorManager")
        // todo decide if it should connect by default in c++ also
        .def(py::init([](){ auto m = new MotorManager(); m->get_connected_motors(); return m; }))
        .def("__repr__", [](const MotorManager &m){ 
            std::string s;
            for (auto motor : m.motors()) {
                s += motor->name() + " ";
            }
            return "<MotorManager connected to: " + s + ">"; })
        .def("get_connected_motors", &MotorManager::get_connected_motors, py::arg("connect") = true)
        .def("get_motors_by_name", &MotorManager::get_motors_by_name, py::arg("names"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("get_motors_by_serial_number", &MotorManager::get_motors_by_serial_number, py::arg("serial_numbers"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("get_motors_by_path", &MotorManager::get_motors_by_path, py::arg("pathss"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("get_motors_by_devpath", &MotorManager::get_motors_by_devpath, py::arg("devpaths"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("motors", &MotorManager::motors)
        .def("set_motors", &MotorManager::set_motors)
        .def("read", &MotorManager::read)
        .def("write", &MotorManager::write)
        .def("write_saved_commands", &MotorManager::write_saved_commands)
        .def("aread", &MotorManager::aread)
        .def("poll", &MotorManager::poll)
        .def("commands", &MotorManager::commands)
        .def("set_commands", &MotorManager::set_commands)
        .def("set_auto_count", &MotorManager::set_auto_count, py::arg("on") = true)
        .def("set_command_count", &MotorManager::set_command_count)
        .def("set_command_mode", static_cast<void (MotorManager::*)(std::vector<uint8_t>)>(&MotorManager::set_command_mode))
        .def("set_command_mode", static_cast<void (MotorManager::*)(uint8_t)>(&MotorManager::set_command_mode))
        .def("set_command_current", &MotorManager::set_command_current)
        .def("set_command_position", &MotorManager::set_command_position)
        .def("set_command_velocity", &MotorManager::set_command_velocity)
        .def("set_command_torque", &MotorManager::set_command_torque)
        .def("set_command_reserved", &MotorManager::set_command_reserved)
        .def("set_command_tuning", &MotorManager::set_command_tuning);

    py::class_<Motor, std::shared_ptr<Motor>>(m, "Motor")
        .def(py::init<const std::string&>())
        .def("name", &Motor::name)
        .def("__repr__", [](const Motor &m){ return "<Motor " + m.name() + ">"; })
        .def("__getitem__", &Motor::operator[])
        .def("__setitem__", [](Motor &m, const std::string key, const std::string value) {
            m[key].set(value);
        });

    py::class_<TextAPIItem>(m, "TextAPIItem")
        .def("__repr__", &TextAPIItem::get)
        .def("get", &TextAPIItem::get)
        .def("set", &TextAPIItem::set)
        //.def("assign", static_cast<void (TextAPIItem::*)(const std::string &)>(&TextAPIItem::operator=));
        .def("assign", &TextAPIItem::set);

    py::enum_<ModeDesired>(m, "ModeDesired")
        .value("Open", ModeDesired::OPEN)
        .value("Damped", ModeDesired::DAMPED)
        .value("Current", ModeDesired::CURRENT)
        .value("Position", ModeDesired::POSITION)
        .value("Torque", ModeDesired::TORQUE)
        .value("Velocity", ModeDesired::VELOCITY)
        .value("CurrentTuning", ModeDesired::CURRENT_TUNING)
        .value("Reset", ModeDesired::RESET)
        .export_values();
    
    py::enum_<TuningMode>(m, "TuningMode")
        .value("Sine", TuningMode::SINE)
        .value("Square", TuningMode::SQUARE)
        .value("Triangle", TuningMode::TRIANGLE)
        .value("Chirp", TuningMode::CHIRP)
        .export_values();

    py::class_<Command>(m, "Command")
        .def(py::init<uint32_t, uint8_t, float, float, float, float>(),
            py::arg("host_timestamp") = 0,
            py::arg("mode") = ModeDesired::OPEN,
            py::arg("current") = 0,
            py::arg("position") = 0,
            py::arg("torque") = 0,
            py::arg("reserved") = 0)
        .def_readwrite("host_timestamp", &Command::host_timestamp)
        .def_readwrite("mode_desired", &Command::mode_desired)
        .def_readwrite("current_desired", &Command::current_desired)
        .def_readwrite("position_desired", &Command::position_desired)
        .def_readwrite("velocity_desired", &Command::velocity_desired)
        .def_readwrite("torque_desired", &Command::torque_desired)
        .def_readwrite("reserved", &Command::reserved)
        .def("__repr__", [](const Command &c) { return "<Command: " + std::to_string(c.host_timestamp) + ">"; });

    py::class_<Status>(m, "Status")
        .def_readonly("mcu_timestamp", &Status::mcu_timestamp)
        .def_readonly("host_timestamp_received", &Status::host_timestamp_received)
        .def_readonly("motor_position", &Status::motor_position)
        .def_readonly("joint_position", &Status::joint_position)
        .def_readonly("iq", &Status::iq)
        .def_readonly("torque", &Status::torque)
        .def_readonly("motor_encoder", &Status::motor_encoder)
        .def_property_readonly("reserved", [](const Status &s) { 
            std::vector<float> f = {s.reserved, s.reserved + sizeof(s.reserved)/sizeof(float)}; 
            return f;
        } )
        .def("__repr__", [](const Status &s) { return "<Status at: " + std::to_string(s.mcu_timestamp) + ">"; });
}