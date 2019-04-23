#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
#include <string>
class Motor;

class MotorManager {
 public:
    std::vector<std::shared_ptr<Motor>> get_connected_motors();
    std::vector<std::shared_ptr<Motor>> get_motors_by_name(std::vector<std::string> names);
    std::vector<std::shared_ptr<Motor>> motors() const { return motors_; }
 private:
    std::vector<std::shared_ptr<Motor>> motors_;
};

#endif
