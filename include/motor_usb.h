#pragma once

#include "motor.h"

namespace obot {

class MotorUSB : public Motor {
   public:
    MotorUSB() = default;

    MotorUSB(std::string dev_path);

    virtual ~MotorUSB();

    ssize_t read() final;

    ssize_t write() final;

    int lock() final;
};

}  // namespace obot