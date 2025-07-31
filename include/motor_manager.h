#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
#include <ostream>
#include <iomanip>
#include <chrono>
#include <map>
#include <poll.h>

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
    static constexpr std::string_view read_fields[] = {
      "mcu_timestamp", "host_timestamp_received", "motor_position", "joint_position",
      "iq", "torque", "motor_encoder", "rr_index", "rr_data", "reserved",
      "motor_velocity", "joint_velocity", "iq_desired", "mode", "error",
      "misc", "mode_error_text", "reserved1", "reserved2",
      "reserved3", "reserved4", "reserved5", "reserved6", "reserved7",
      "reserved8", "reserved9", "reserved10"
    };

    MotorManager(bool user_space_driver = false, Motor::MessagesCheck check_messages_version = Motor::MessagesCheck::MAJOR) :
       user_space_driver_(user_space_driver),
       check_messages_version_(check_messages_version) {}
    std::vector<std::shared_ptr<Motor>> get_connected_motors(bool connect = true);
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_serial_number(std::vector<std::string> serial_numbers, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_path(std::vector<std::string> paths, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_devpath(std::vector<std::string> devpaths, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_uart_by_devpath(std::vector<std::string> devpaths, bool raw = false, uint32_t baud_rate = 4000000, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> get_motors_by_ip(std::vector<std::string> ips, bool connect = true, bool print_unconnected = true, bool allow_simulated = false, std::vector<std::string> ip_aliases = {});
    std::vector<std::shared_ptr<Motor>> get_motors_can(std::vector<std::string> can_interfaces, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    int size() const { return motors_.size(); }
    void free_motors() {
      for(auto &m : motors_) {
         m.reset();
      }
    }
    void set_motors(std::vector<std::shared_ptr<Motor>> motors);
    std::vector<Command> &commands() { return commands_; }
    std::vector<Status> &read();
    std::vector<Status> &get_statuses() noexcept { return statuses_; }
    std::vector<Status> read_average(uint32_t num_average = 1);
    void write(std::vector<Command> &);
    void write_saved_commands();
    void aread();
    void start_nonblocking_read();
    int poll(uint32_t timeout_ms = 1);
    int multipoll(uint32_t timeout_ns = 0);
    void lock();
    void check_messages_version(Motor::MessagesCheck check_messages_version) { check_messages_version_ = check_messages_version; }

    void set_auto_count(bool on=true) { auto_count_ = on; }
    uint32_t get_auto_count() const { return count_; }
    void set_reconnect(bool reconnect=true) { reconnect_ = reconnect; }
    void set_commands(const std::vector<Command> &commands);
    void clear_commands();
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
    void set_command_stepper_velocity(double current,  double velocity, double voltage = 0, StepperMode mode = StepperMode::STEPPER_CURRENT);

    std::string command_headers() const;
    std::string status_headers(bool mini = false, bool print_reserved = false) const;
    int serialize_command_size() const;
    int serialize_saved_commands(char *data) const;
    bool deserialize_saved_commands(char *data);
    const std::vector<int> &get_read_error_count() const { return read_error_count_; }
    const std::vector<int> &get_nonblock_not_ready_error_count() const { return nonblock_not_ready_error_count_; }
 private:
    std::vector<std::shared_ptr<Motor>> get_motors_by_name_function(std::vector<std::string> names, std::string (Motor::*name_fun)() const, bool connect = true, bool allow_simulated = false);
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<int> read_error_count_;
    std::vector<int> nonblock_not_ready_error_count_;
    std::vector<Command> commands_;
    std::vector<Status> statuses_;
    bool user_space_driver_;
    uint32_t count_ = 0;
    bool auto_count_ = false;
    bool reconnect_ = false;
    FrequencyLimiter reconnect_rate_ = std::chrono::milliseconds(100);
    std::vector<pollfd> pollfds_;
    Motor::MessagesCheck check_messages_version_;
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

#define PRINT_FLAG(flag) if (error.flag) os << #flag " "

inline std::ostream& operator<<(std::ostream& os, const MotorError &error)
{
   if (error.all) {
      PRINT_FLAG(sequence);
      PRINT_FLAG(bus_voltage_low);
      PRINT_FLAG(bus_voltage_high);
      PRINT_FLAG(bus_current);
      PRINT_FLAG(microcontroller_temperature);
      PRINT_FLAG(board_temperature);
      PRINT_FLAG(motor_temperature);
      PRINT_FLAG(driver_fault);
      PRINT_FLAG(motor_overcurrent);
      PRINT_FLAG(motor_phase_open);
      PRINT_FLAG(motor_encoder);
      PRINT_FLAG(motor_encoder_limit);
      PRINT_FLAG(output_encoder);
      PRINT_FLAG(output_encoder_limit);
      PRINT_FLAG(torque_sensor);
      PRINT_FLAG(controller_tracking);
      PRINT_FLAG(host_fault);
      PRINT_FLAG(driver_not_enabled);
      PRINT_FLAG(encoder_disagreement);
      PRINT_FLAG(torque_sensor_disagreement);
      PRINT_FLAG(init_failure);
      PRINT_FLAG(invalid_command);
      PRINT_FLAG(motor_encoder_warning);
      PRINT_FLAG(output_encoder_warning);
      PRINT_FLAG(torque_sensor_warning);
      PRINT_FLAG(motor_current_limit);
      PRINT_FLAG(motor_voltage_limit);
      PRINT_FLAG(motor_soft_limit);
      PRINT_FLAG(fault);
   }
   return os;
}

inline int reserved_print_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& reserved_print_on(std::ostream& os) {
   os.iword(reserved_print_index()) = 1;
   return os;
}

inline std::ostream& reserved_print_off(std::ostream& os) {
   os.iword(reserved_print_index()) = 0;
   return os;
}

inline int fields_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& fields_on(std::ostream& os) {
   os.iword(fields_index()) = 1;
   return os;
}

inline std::ostream& fields_off(std::ostream& os) {
   os.iword(fields_index()) = 0;
   return os;
}

inline int mcu_timestamp_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& mcu_timestamp_on(std::ostream& os) {
   os.iword(mcu_timestamp_index()) = 1;
   return os;
}

inline std::ostream& mcu_timestamp_off(std::ostream& os) {
   os.iword(mcu_timestamp_index()) = 0;
   return os;
}

inline int host_timestamp_received_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& host_timestamp_received_on(std::ostream& os) {
   os.iword(host_timestamp_received_index()) = 1;
   return os;
}

inline std::ostream& host_timestamp_received_off(std::ostream& os) {
   os.iword(host_timestamp_received_index()) = 0;
   return os;
}

inline int motor_position_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& motor_position_on(std::ostream& os) {
   os.iword(motor_position_index()) = 1;
   return os;
}

inline std::ostream& motor_position_off(std::ostream& os) {
   os.iword(motor_position_index()) = 0;
   return os;
}

inline int joint_position_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& joint_position_on(std::ostream& os) {
   os.iword(joint_position_index()) = 1;
   return os;
}

inline std::ostream& joint_position_off(std::ostream& os) {
   os.iword(joint_position_index()) = 0;
   return os;
}

inline int reserved_index() {
   static int index = std::ios_base::xalloc();
   return index;
}

inline std::ostream& reserved_on(std::ostream& os) {
   os.iword(reserved_index()) = 1;
   return os;
}

inline std::ostream& reserved_off(std::ostream& os) {
   os.iword(reserved_index()) = 0;
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<Status> &status)
{

   if (!os.iword(fields_index()) || os.iword(mcu_timestamp_index())) {
      for (auto s : status) {
         os << std::setw(10) << s.mcu_timestamp << ", ";
      }
   }
   if (!os.iword(fields_index()) || os.iword(host_timestamp_received_index())) {
      for (auto s : status) {
         os << std::setw(10) << s.host_timestamp_received << ", ";
      }
   }
   if (!os.iword(fields_index()) || os.iword(motor_position_index())) {
      for (auto s : status) {
         os << std::setw(8) << s.motor_position << ", ";
      }
   }
   if (!os.iword(fields_index()) || os.iword(joint_position_index())) {
      for (auto s : status) {
         os << std::setw(8) << s.joint_position << ", ";
      }
   }
   if (!os.iword(fields_index())) {
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
            os << s.rr_data.data_u32 << ", ";
            break;
         case INT32_T:
            os << s.rr_data.data_i32 << ", ";
            break;
      }
   }
   }
   if (!os.iword(fields_index()) || os.iword(reserved_index())) {
      for (auto s : status) {
         os << s.reserved << ", ";
      }
   }
   if (!os.iword(fields_index())) {
   for (auto s : status) {
      os << std::setw(8) << s.motor_velocity << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.joint_velocity << ", ";
   }
   for (auto s : status) {
      os << std::setw(8) << s.iq_desired << ", ";
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
      os << s.flags.error << ", ";
   }
   if (os.iword(reserved_print_index())) {
      for (auto s : status) {
         for (int i=0; i < sizeof(s.large.reserved)/sizeof(s.large.reserved[0]); i++) {
            os << s.large.reserved[i] << ", ";
         }
      }
   }
   }
   return os;
}

}  // namespace obot

#endif
