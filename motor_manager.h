#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
class Motor;

#include "motor.h"

class MotorManager {
 public:
    std::vector<std::shared_ptr<Motor>> get_connected_motors();
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
    std::vector<Command> commands() const { return commands_; }
    void open();
    std::vector<Status> read();
    void write(std::vector<Command>);
    void write_saved_commands();
    void aread();
    void close();

    void set_command_count(int32_t count);
    void set_command_mode(uint8_t mode);
    void set_command_current(std::vector<float> current);
    void set_command_position(std::vector<float> position);
 private:
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<Command> commands_;
};

#endif
