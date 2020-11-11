#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
#include <ostream>
class Motor;

#include "motor.h"

class MotorManager {
 public:
    MotorManager(bool user_space_driver = false) : user_space_driver_(user_space_driver) {}
    std::vector<std::shared_ptr<Motor>> get_connected_motors();
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names);
    std::vector<std::shared_ptr<Motor>> get_motors_by_serial_number(std::vector<std::string> serial_numbers);
    std::vector<std::shared_ptr<Motor>> get_motors_by_path(std::vector<std::string> paths);
    std::vector<std::shared_ptr<Motor>> get_motors_by_devpath(std::vector<std::string> devpaths);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    void set_motors(std::vector<std::shared_ptr<Motor>> motors) { motors_ = motors; commands_.resize(motors_.size()); }
    std::vector<Command> commands() const { return commands_; }
    std::vector<Status> read();
    void write(std::vector<Command>);
    void write_saved_commands();
    void aread();
    int poll();

    void set_auto_count(bool on=true) { auto_count_ = on; }
    void set_commands(std::vector<Command> commands);
    void set_command_count(int32_t count);
    void set_command_mode(uint8_t mode);
    void set_command_current(std::vector<float> current);
    void set_command_position(std::vector<float> position);
    void set_command_velocity(std::vector<float> velocity);
    void set_command_torque(std::vector<float> torque);

    std::string command_headers() const;
    std::string status_headers() const;
    int serialize_command_size() const;
    int serialize_saved_commands(char *data) const;
    bool deserialize_saved_commands(char *data);
 private:
    std::vector<std::shared_ptr<Motor>> get_motors_by_name_function(std::vector<std::string> names, std::string (Motor::*name_fun)() const);
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<Command> commands_;
    bool user_space_driver_;
    uint32_t count_ = 0;
    bool auto_count_ = false;
};

inline std::vector<float> get_joint_position(std::vector<Status> statuses) {
   std::vector<float> out;
   for (auto stat : statuses) {
      out.push_back(stat.joint_position);
   }
   return out;
}

inline std::vector<float> get_motor_position(std::vector<Status> statuses) {
   std::vector<float> out;
   for (auto stat : statuses) {
      out.push_back(stat.motor_position);
   }
   return out;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<Command> command)
{
   for (auto c : command) {
      os << command << ", ";
   }
   for (auto c : command) {
      os << c.host_timestamp << ", ";
   }
   for (auto c : command) {
      os << +c.mode_desired << ", ";
   }
   for (auto c : command) {
      os << c.current_desired << ", ";
   }
   for (auto c : command) {
      os << c.position_desired << ", ";
   }
   for (auto c : command) {
      os << c.velocity_desired << ", ";
   }
   for (auto c : command) {
      os << c.torque_desired << ", ";
   }
   for (auto c : command) {
      os << c.reserved << ", ";
   }

   return os;
}


inline std::ostream& operator<<(std::ostream& os, const std::vector<Status> status)
{
   for (auto s : status) {
      os << s << ", ";
   }
}



#endif
