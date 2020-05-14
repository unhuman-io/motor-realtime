#pragma once

class MotorThread;

class MotorApp {
 public:
    MotorApp(int argc, char **argv, MotorThread *motor_thread);
    int run();
 private:
    MotorThread *motor_thread_;
};