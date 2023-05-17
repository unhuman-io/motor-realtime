#pragma once
#include <string>
#include <vector>

namespace obot {

class MotorThread;
class MotorManager;

class MotorApp {
 public:
    MotorApp(int argc, char **argv, MotorThread *motor_thread, std::string app_name = "Motor app");
    virtual ~MotorApp() {}
    int run();
    virtual void select_motors(MotorManager *);
 private:
    MotorThread *motor_thread_;
    std::vector<std::string> names_;
    bool allow_simulated_ = false;
};

}  // namespace obot
