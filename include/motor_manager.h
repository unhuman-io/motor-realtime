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
    std::vector<std::shared_ptr<Motor>> get_connected_motors(bool user_space_driver=false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names, bool user_space_driver=false);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    std::vector<Command> commands() const { return commands_; }
    void open();
    std::vector<Status> read();
    void write(std::vector<Command>);
    void write_saved_commands();
    void aread();
    int poll();
    void close();

    void set_commands(std::vector<Command> commands);
    void set_command_count(int32_t count);
    void set_command_mode(uint8_t mode);
    void set_command_current(std::vector<float> current);
    void set_command_position(std::vector<float> position);
    void set_command_velocity(std::vector<float> velocity);
    std::string command_headers() const;
    std::string status_headers() const;
 private:
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<Command> commands_;
};

inline std::ostream& operator<<(std::ostream& os, const std::vector<Command> command)
{
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
      os << c.reserved << ", ";
   }

   return os;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<Status> status)
{
   for (auto s : status) {
      os << s.mcu_timestamp << ", ";
   }
   for (auto s : status) {
      os << s.host_timestamp_received << ", ";
   }
   for (auto s : status) {
      os << s.motor_position << ", ";
   }
   for (auto s : status) {
      os << s.joint_position << ", ";
   }
   for (auto s : status) {
      os << s.iq << ", ";
   }
   for (auto s : status) {
      os << s.motor_encoder << ", ";
   }
   for (auto s : status) {
      os << s.reserved[0] << ", ";
   }
   return os;
}

#endif
