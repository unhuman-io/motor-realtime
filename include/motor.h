#pragma once

class Motor {
 public:
    void write() {};
    void read();
 private:
    MotorCommunication motor_communication_;
    Status status_;
    Command command_;
};