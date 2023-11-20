#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "motor_manager.h"
#include <sstream>

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
    d["encoder_disagreement"] = e.encoder_disagreement;
    d["torque_sensor_disagreement"] = e.torque_sensor_disagreement;
    d["motor_soft_limit"] = e.motor_soft_limit;
    d["fault"] = e.fault;
    return d;
}

static MotorError dict_to_motor_error(py::dict d) {
    MotorError e;
    e.sequence = d["sequence"].cast<bool>();
    e.bus_voltage_low = d["bus_voltage_low"].cast<bool>();
    e.bus_voltage_high = d["bus_voltage_high"].cast<bool>();
    e.bus_current = d["bus_current"].cast<bool>();
    e.microcontroller_temperature = d["microcontroller_temperature"].cast<bool>();
    e.board_temperature = d["board_temperature"].cast<bool>();
    e.motor_temperature = d["motor_temperature"].cast<bool>();
    e.driver_fault = d["driver_fault"].cast<bool>();
    e.motor_overcurrent = d["motor_overcurrent"].cast<bool>();
    e.motor_phase_open = d["motor_phase_open"].cast<bool>();
    e.motor_encoder = d["motor_encoder"].cast<bool>();
    e.motor_encoder_limit = d["motor_encoder_limit"].cast<bool>();
    e.output_encoder = d["output_encoder"].cast<bool>();
    e.output_encoder_limit = d["output_encoder_limit"].cast<bool>();
    e.torque_sensor = d["torque_sensor"].cast<bool>();
    e.controller_tracking = d["controller_tracking"].cast<bool>();
    e.host_fault = d["host_fault"].cast<bool>();
    e.driver_not_enabled = d["driver_not_enabled"].cast<bool>();
    e.encoder_disagreement = d["encoder_disagreement"].cast<bool>();
    e.torque_sensor_disagreement = d["torque_sensor_disagreement"].cast<bool>();
    e.motor_soft_limit = d["motor_soft_limit"].cast<bool>();
    e.fault = d["fault"].cast<bool>();
    return e;
}

PYBIND11_MODULE(motor, m)
{
    m.doc() = "Motor interface";

    py::enum_<ModeDesired>(m, "ModeDesired")
        .value("Open", ModeDesired::OPEN)
        .value("Damped", ModeDesired::DAMPED)
        .value("Current", ModeDesired::CURRENT)
        .value("Position", ModeDesired::POSITION)
        .value("Torque", ModeDesired::TORQUE)
        .value("Velocity", ModeDesired::VELOCITY)
        .value("Voltage", ModeDesired::VOLTAGE)
        .value("PhaseLock", ModeDesired::PHASE_LOCK)
        .value("StepperTuning", ModeDesired::STEPPER_TUNING)
        .value("StepperVelocity", ModeDesired::STEPPER_VELOCITY)
        .value("CurrentTuning", ModeDesired::CURRENT_TUNING)
        .value("PositionTuning", ModeDesired::POSITION_TUNING)
        .value("JointPosition", ModeDesired::JOINT_POSITION)
        .value("Admittance", ModeDesired::ADMITTANCE)
        .value("Tuning", ModeDesired::TUNING)
        .value("FindLimits", ModeDesired::FIND_LIMITS)
        .value("DriverEnable", ModeDesired::DRIVER_ENABLE)
        .value("DriverDisable", ModeDesired::DRIVER_DISABLE)
        .value("ClearFaults", ModeDesired::CLEAR_FAULTS)
        .value("Fault", ModeDesired::FAULT)
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

    
    py::class_<PositionTuningCommand>(m, "PositionTuningCommand")
        .def_readwrite("amplitude", &PositionTuningCommand::amplitude)
        .def_readwrite("mode", &PositionTuningCommand::mode)
        .def_readwrite("bias", &PositionTuningCommand::bias)
        .def_readwrite("frequency", &PositionTuningCommand::frequency);

    py::class_<TuningCommand>(m, "TuningCommand")
        .def_readwrite("amplitude", &TuningCommand::amplitude)
        .def_readwrite("mode", &TuningCommand::mode)
        .def_readwrite("tuning_mode", &TuningCommand::tuning_mode)
        .def_readwrite("bias", &TuningCommand::bias)
        .def_readwrite("frequency", &TuningCommand::frequency);

    py::enum_<StepperMode>(m, "StepperMode")
        .value("StepperCurrent", StepperMode::STEPPER_CURRENT)
        .value("StepperVoltage", StepperMode::STEPPER_VOLTAGE)
        .export_values();

    

    py::class_<MotorManager>(m, "MotorManager")
        // todo decide if it should connect by default in c++ also
        .def(py::init([](bool u)
                      { auto m = new MotorManager(u); m->get_connected_motors(); return m; }), py::arg("user_space_driver") = false)
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
        .def("free_motors", &MotorManager::free_motors)
        .def("set_motors", &MotorManager::set_motors)
        .def("read", &MotorManager::read)
        .def("read_average", &MotorManager::read_average)
        .def("start_nonblocking_read", &MotorManager::start_nonblocking_read)
        .def("write", &MotorManager::write)
        .def("write_saved_commands", &MotorManager::write_saved_commands)
        .def("aread", &MotorManager::aread)
        .def("poll", &MotorManager::poll)
        .def("commands", &MotorManager::commands)
        .def("set_commands", &MotorManager::set_commands)
        .def("set_auto_count", &MotorManager::set_auto_count, py::arg("on") = true)
        .def("clear_commands", &MotorManager::clear_commands)
        .def("set_command_count", &MotorManager::set_command_count)
        .def("set_command_mode", static_cast<void (MotorManager::*)(const std::vector<uint8_t> &)>(&MotorManager::set_command_mode))
        .def("set_command_mode", static_cast<void (MotorManager::*)(uint8_t)>(&MotorManager::set_command_mode))
        .def("set_command_current", &MotorManager::set_command_current)
        .def("set_command_position", &MotorManager::set_command_position)
        .def("set_command_velocity", &MotorManager::set_command_velocity)
        .def("set_command_torque", &MotorManager::set_command_torque)
        .def("set_command_reserved", &MotorManager::set_command_reserved)
        .def("set_command_stepper_tuning", &MotorManager::set_command_stepper_tuning)
        .def("set_command_stepper_velocity", &MotorManager::set_command_stepper_velocity, py::arg("current"), py::arg("velocity"), py::arg("voltage") = 0, py::arg("stepper_mode") = StepperMode::STEPPER_CURRENT)
        .def("set_command_position_tuning", &MotorManager::set_command_position_tuning)
        .def("set_command_current_tuning", &MotorManager::set_command_current_tuning);

    py::class_<Motor, std::shared_ptr<Motor>>(m, "Motor")
        .def(py::init<const std::string &>())
        .def("name", &Motor::name)
        .def("serial_number", &Motor::serial_number)
        .def("path", &Motor::base_path)
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
        .def("set_error_mask", [](Motor &m, py::dict d){ 
            MotorError e = dict_to_motor_error(d);
            std::stringstream s;
            s << std::hex << e.all;
            std::string shex(s.str());
            return m["error_mask"].set(shex); })
        .def("get_cpu_frequency", &Motor::get_cpu_frequency)
        .def("set_nonblock", &Motor::set_nonblock)
        .def("get_timeout_ms", &Motor::get_timeout_ms)
        .def("set_timeout_ms", &Motor::set_timeout_ms);

    py::class_<TextAPIItem>(m, "TextAPIItem")
        .def("__repr__", &TextAPIItem::get)
        .def("get", &TextAPIItem::get)
        .def("set", &TextAPIItem::set)
        //.def("assign", static_cast<void (TextAPIItem::*)(const std::string &)>(&TextAPIItem::operator=));
        .def("assign", &TextAPIItem::set);

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
        .def_readwrite("position_tuning", &Command::position_tuning)
        .def_readwrite("stepper_velocity", &Command::stepper_velocity)
        .def_readwrite("tuning_command", &Command::tuning_command)
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
        .def_readonly("motor_velocity", &Status::motor_velocity)
        .def_readonly("joint_velocity", &Status::joint_velocity)
        .def_readonly("iq_desired", &Status::iq_desired)
        .def_readonly("reserved", &Status::reserved)
        .def_readonly("flags", &Status::flags)
        .def("__repr__", [](const Status &s)
             { return "<Status at: " + std::to_string(s.mcu_timestamp) + ">"; });

    m.def("diff_mcu_time", [](uint32_t t1, uint32_t t2)
          { return t1 - t2; });
    m.def("diff_encoder", [](int32_t p1, int32_t p2)
          { return (int32_t)((uint32_t)p1 - (uint32_t)p2); });
}