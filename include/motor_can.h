#pragma once

#include "motor.h"

namespace obot {

class MotorCAN : public Motor {
 public:
    MotorCAN(std::string address) {

        std::cout << "MotorCAN constructor: " << address << std::endl;

    }
    virtual ~MotorCAN() {}
    

 private:

};

}; // namespace obot