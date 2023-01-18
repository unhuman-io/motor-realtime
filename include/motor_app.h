#pragma once

namespace obot {

class MotorThread;
class MotorManager;

class MotorApp {
 public:
    MotorApp(int argc, char **argv, MotorThread *motor_thread);
    virtual ~MotorApp() {}
    int run();
    virtual void select_motors(MotorManager *);
 private:
    MotorThread *motor_thread_;
};

}  // namespace obot
