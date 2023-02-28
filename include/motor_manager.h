#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
#include <ostream>
#include <iomanip>
#include <chrono>
#include <map>

#include "motor.h"

namespace obot {

class FrequencyLimiter {
 public:
    FrequencyLimiter(std::chrono::milliseconds t_diff) {
        t_diff_ = t_diff;
        last_time_ = std::chrono::steady_clock::now();
    }
    // returns true once for each time it is allowed to run
    bool run() {
        auto time = std::chrono::steady_clock::now();
        if (time - last_time_ > t_diff_) {
            last_time_ = time; //todo needs to account for time after t_diff
            return true;
        }
        return false;
    }
 private:
    std::chrono::milliseconds t_diff_;
    std::chrono::time_point<std::chrono::steady_clock> last_time_;
};

class MotorManager {
 public:
    static const std::map<const ModeDesired, const std::string> mode_map;

    MotorManager(bool user_space_driver = false) : user_space_driver_(user_space_driver) {}
    std::vector<std::shared_ptr<Motor>> get_connected_motors(bool connect = true);
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_serial_number(std::vector<std::string> serial_numbers, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_path(std::vector<std::string> paths, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_devpath(std::vector<std::string> devpaths, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    void set_motors(std::vector<std::shared_ptr<Motor>> motors) { motors_ = motors; commands_.resize(motors_.size()); }
    std::vector<Command> &commands() { return commands_; }
    std::vector<Status> &read();
    void write(std::vector<Command> &);
    void write_saved_commands();
    void aread();
    void start_nonblocking_read();
    int poll();
    int multipoll(uint32_t timeout_ns = 0);

    void set_auto_count(bool on=true) { auto_count_ = on; }
    uint32_t get_auto_count() const { return count_; }
    void set_reconnect(bool reconnect=true) { reconnect_ = reconnect; }
    void set_commands(const std::vector<Command> &commands);
    void set_command_count(int32_t count);
    void set_command_mode(uint8_t mode);
    void set_command_mode(const std::vector<uint8_t> &mode);
    void set_command_current(const std::vector<float> &current);
    void set_command_position(const std::vector<float> &position);
    void set_command_velocity(const std::vector<float> &velocity);
    void set_command_torque(const std::vector<float> &torque);
    void set_command_reserved(const std::vector<float> &reserved);
    void set_command_stepper_tuning(TuningMode tuning_mode, 
         double amplitude, double frequency, double bias, double kv);
    void set_command_current_tuning(TuningMode tuning_mode, 
         double amplitude, double frequency, double bias);
    void set_command_position_tuning(TuningMode tuning_mode, 
         double amplitude, double frequency, double bias);
    void set_command_stepper_velocity(double voltage, double velocity);

    std::string command_headers() const;
    std::string status_headers() const;
    int serialize_command_size() const;
    int serialize_saved_commands(char *data) const;
    bool deserialize_saved_commands(char *data);
    const std::vector<int> &get_read_error_count() const { return read_error_count_; }
 private:
    std::vector<std::shared_ptr<Motor>> get_motors_by_name_function(std::vector<std::string> names, std::string (Motor::*name_fun)() const, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<int> read_error_count_;
    std::vector<Command> commands_;
    std::vector<Status> statuses_;
    bool user_space_driver_;
    uint32_t count_ = 0;
    bool auto_count_ = false;
    bool reconnect_ = false;
    FrequencyLimiter reconnect_rate_ = std::chrono::milliseconds(100);
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
      os << c.host_timestamp << ", ";
   }
   for (auto c : command) {
      os << +c.mode_desired << ", ";
   }
   for (auto c : command) {
      os << +c.misc.byte << ", ";
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
      os << c.torque_dot_desired << ", ";
   }
   for (auto c : command) {
      os << c.reserved << ", ";
   }
   for (auto c : command) {
      os << c.reserved2[0] << ", ";
   }
   for (auto c : command) {
      os << c.reserved2[1] << ", ";
   }
   for (auto c : command) {
      os << c.reserved2[2] << ", ";
   }
   return os;
}

inline std::istream& operator>>(std::istream& is, std::vector<Command> &command)
{
   char s;
   for (auto &c : command) {
      is >> c.host_timestamp >> s;
   }
   for (auto &c : command) {
      uint16_t u;
      is >> u >> s;
      c.mode_desired = u;
   }
   for (auto &c : command) {
      uint16_t u;
      is >> u >> s;
      c.misc.byte = u;
   }
   for (auto &c : command) {
      is >> c.current_desired >> s;
   }
   for (auto &c : command) {
      is >> c.position_desired >> s;
   }
   for (auto &c : command) {
      is >> c.velocity_desired >> s;
   }
   for (auto &c : command) {
      is >> c.torque_desired >> s;
   }
   for (auto &c : command) {
      is >> c.reserved >> s;
   }

   return is;
}

#define PRINT_FLAG(flag) if (s.flags.error.flag) os << #flag " "

inline std::ostream& operator<<(std::ostream& os, const std::vector<Status> status)
{

   for (auto s : status) {
      os << std::setw(10) << s.mcu_timestamp << ", ";
   }
   for (auto s : status) {
      os << s.host_timestamp_received << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.motor_position << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.joint_position << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.iq << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.torque << ", ";
   }
   for (auto s : status) {
      os << s.motor_encoder << ", ";
   }
   for (auto s : status) {
      os << static_cast<int>(s.rr_data.index) << ", ";
      switch (s.rr_data.type) {
         case FLOAT:
            os << s.rr_data.data << ", ";
            break;
         case UINT32_T:
            os << *reinterpret_cast<uint32_t *>(&s.rr_data.data) << ", ";
            break;
         case INT32_T:
            os << *reinterpret_cast<uint32_t *>(&s.rr_data.data) << ", ";
            break;
      }
   }
   for (auto s : status) {
      os << s.reserved << ", ";
   }
   for (auto s : status) {
      os << static_cast<int>(s.flags.mode) << ", ";
   }
   os << std::hex;
   for (auto s : status) {
      os << static_cast<int>(s.flags.error.all) << ", ";
   }
   for (auto s : status) {
      os << static_cast<int>(s.flags.misc.byte) << ", ";
   }
   os << std::dec;
   for (auto s : status) {
      os << MotorManager::mode_map.at(static_cast<ModeDesired>(s.flags.mode)) << " ";
      if (s.flags.error.all) {
         PRINT_FLAG(sequence);
         PRINT_FLAG(bus_voltage_low);
         PRINT_FLAG(bus_voltage_high);
         PRINT_FLAG(bus_current);
         PRINT_FLAG(microcontroller_temperature);
         PRINT_FLAG(board_temperature);
         PRINT_FLAG(motor_temperature);
         PRINT_FLAG(driver_fault);
         PRINT_FLAG(motor_overcurent);
         PRINT_FLAG(motor_phase_open);
         PRINT_FLAG(motor_encoder);
         PRINT_FLAG(motor_encoder_limit);
         PRINT_FLAG(output_encoder);
         PRINT_FLAG(output_encoder_limit);
         PRINT_FLAG(torque_sensor);
         PRINT_FLAG(controller_tracking);
         PRINT_FLAG(host_fault);
         PRINT_FLAG(driver_not_enabled);
         PRINT_FLAG(fault);
      }
      os << ", ";
   }
   return os;
}

}  // namespace obot

#endif
