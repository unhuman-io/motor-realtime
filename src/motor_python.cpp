#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "motor_manager.h"

namespace py = pybind11;

using namespace obot;

static std::string hex(uint32_t u)
{
    char s[9];
    std::snprintf(s, 9, "%08x", u);
    return s;
}

static py::dict motor_error_dict(const MotorError &e)
{
    py::dict d;
    d["sequence"] = e.sequence;
    d["bus_voltage_low"] = e.bus_voltage_low;
    d["bus_voltage_high"] = e.bus_voltage_high;
    d["bus_current"] = e.bus_current;
    d["microcontroller_temperature"] = e.microcontroller_temperature;
    d["board_temperature"] = e.board_temperature;
    d["motor_temperature"] = e.motor_temperature;
    d["driver_fault"] = e.driver_fault;
    d["motor_overcurrent"] = e.motor_overcurrent;
    d["motor_phase_open"] = e.motor_phase_open;
    d["motor_encoder"] = e.motor_encoder;
    d["motor_encoder_limit"] = e.motor_encoder_limit;
    d["output_encoder"] = e.output_encoder;
    d["output_encoder_limit"] = e.output_encoder_limit;
    d["torque_sensor"] = e.torque_sensor;
    d["controller_tracking"] = e.controller_tracking;
    d["host_fault"] = e.host_fault;
    d["driver_not_enabled"] = e.driver_not_enabled;
    d["fault"] = e.fault;
    return d;
}

PYBIND11_MODULE(motor, m)
{
    m.doc() = "Motor interface";
    py::class_<MotorManager>(m, "MotorManager")
        // todo decide if it should connect by default in c++ also
        .def(py::init([]()
                      { auto m = new MotorManager(); m->get_connected_motors(); return m; }))
        .def("__repr__", [](const MotorManager &m)
             { 
            std::string s;
            for (auto motor : m.motors()) {
                s += motor->name() + " ";
            }
            return "<MotorManager connected to: " + s + ">"; })
        .def("get_connected_motors", &MotorManager::get_connected_motors, py::arg("connect") = true)
        .def("get_motors_by_name", &MotorManager::get_motors_by_name, py::arg("names"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("get_motors_by_serial_number", &MotorManager::get_motors_by_serial_number, py::arg("serial_numbers"), py::arg("connect") = true, py::arg("allow_simulated") = false)
        .def("get_motors_by_path", &MotorManager::get_motors_by_path, py::arg("paths"), py::arg("connect") = true, py::arg("allow_simulated") = false)
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
        .def("set_command_mode", static_cast<void (MotorManager::*)(const std::vector<uint8_t> &)>(&MotorManager::set_command_mode))
        .def("set_command_mode", static_cast<void (MotorManager::*)(uint8_t)>(&MotorManager::set_command_mode))
        .def("set_command_current", &MotorManager::set_command_current)
        .def("set_command_position", &MotorManager::set_command_position)
        .def("set_command_velocity", &MotorManager::set_command_velocity)
        .def("set_command_torque", &MotorManager::set_command_torque)
        .def("set_command_reserved", &MotorManager::set_command_reserved)
        .def("set_command_stepper_tuning", &MotorManager::set_command_stepper_tuning)
        .def("set_command_stepper_velocity", &MotorManager::set_command_stepper_velocity)
        .def("set_command_position_tuning", &MotorManager::set_command_position_tuning)
        .def("set_command_current_tuning", &MotorManager::set_command_current_tuning);

    py::class_<Motor, std::shared_ptr<Motor>>(m, "Motor")
        .def(py::init<const std::string &>())
        .def("name", &Motor::name)
        .def("serial_number", &Motor::serial_number)
        .def("get_fast_log", &Motor::get_fast_log)
        .def("__repr__", [](const Motor &m){ return "<Motor " + m.name() + ">"; })
        .def("__getitem__", &Motor::operator[])
        .def("__setitem__", [](Motor &m, const std::string key, const std::string value)
             { m[key].set(value); })
        .def("get_api_options", &Motor::get_api_options)
        .def("error_mask", [](Motor &m)
             { 
            MotorError mask;
            mask.all = std::stoul(m["error_mask"].get(), 0, 16);
            return motor_error_dict(mask); })
        .def("get_cpu_frequency", &Motor::get_cpu_frequency);

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
        .value("Voltage", ModeDesired::VOLTAGE)
        .value("PhaseLock", ModeDesired::PHASE_LOCK)
        .value("CurrentTuning", ModeDesired::CURRENT_TUNING)
        .value("JointPosition", ModeDesired::JOINT_POSITION)
        .value("DriverEnable", ModeDesired::DRIVER_ENABLE)
        .value("DriverDisable", ModeDesired::DRIVER_DISABLE)
        .value("Sleep", ModeDesired::SLEEP)
        .value("Reset", ModeDesired::RESET)
        .export_values();

    py::enum_<TuningMode>(m, "TuningMode")
        .value("Sine", TuningMode::SINE)
        .value("Square", TuningMode::SQUARE)
        .value("Triangle", TuningMode::TRIANGLE)
        .value("Chirp", TuningMode::CHIRP)
        .export_values();

    py::class_<CurrentTuningCommand>(m, "CurrentTuningCommand")
        .def_readwrite("amplitude", &CurrentTuningCommand::amplitude)
        .def_readwrite("mode", &CurrentTuningCommand::mode)
        .def_readwrite("bias", &CurrentTuningCommand::bias)
        .def_readwrite("frequency", &CurrentTuningCommand::frequency);

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
        .def_readwrite("current_tuning", &Command::current_tuning)
        .def("__repr__", [](const Command &c)
             { return "<Command: " + std::to_string(c.host_timestamp) + ">"; });

    py::class_<MotorError>(m, "MotorError")
        .def_readonly("all", &MotorError::all)
        .def_property_readonly("bits", [](const MotorError &e)
                               { return motor_error_dict(e); })
        .def("__repr__", [](const MotorError &e)
             { return "<MotorError: " + hex(e.all) + ">"; });

    py::class_<MotorFlags>(m, "MotorFlags")
        .def_property_readonly("mode", [](MotorFlags &f)
                               { return static_cast<MotorMode>(f.mode); })
        .def_readonly("error", &MotorFlags::error)
        .def("__repr__", [](const MotorFlags &f)
             { return "<MotorFlags: " + std::to_string(f.mode) + ">"; });

    py::class_<Status>(m, "Status")
        .def_readonly("mcu_timestamp", &Status::mcu_timestamp)
        .def_readonly("host_timestamp_received", &Status::host_timestamp_received)
        .def_readonly("motor_position", &Status::motor_position)
        .def_readonly("joint_position", &Status::joint_position)
        .def_readonly("iq", &Status::iq)
        .def_readonly("torque", &Status::torque)
        .def_readonly("motor_encoder", &Status::motor_encoder)
        .def_property_readonly("reserved", [](const Status &s)
                               { 
            std::vector<float> f = {s.reserved, s.reserved + sizeof(s.reserved)/sizeof(float)}; 
            return f; })
        .def_readonly("flags", &Status::flags)
        .def("__repr__", [](const Status &s)
             { return "<Status at: " + std::to_string(s.mcu_timestamp) + ">"; });

    m.def("diff_mcu_time", [](uint32_t t1, uint32_t t2)
          { return t1 - t2; });
    m.def("diff_encoder", [](int32_t p1, int32_t p2)
          { return (int32_t)((uint32_t)p1 - (uint32_t)p2); });
}