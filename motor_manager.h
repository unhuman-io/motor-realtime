#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <vector>
#include <memory>
class Motor;

class MotorManager {
 public:
    std::vector<std::shared_ptr<Motor>> get_connected_motors();
};

#endif
