#pragma once

class MotorThread;
class MotorManager;

class MotorApp {
 public:
    MotorApp(int argc, char **argv, MotorThread *motor_thread);
    int run();
    virtual void select_motors(MotorManager *);
 private:
    MotorThread *motor_thread_;
};