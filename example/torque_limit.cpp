// A switch pro controller to differential drive two wheel motors
#include "motor_manager.h"

#include <iostream>
#include <iomanip>

#include "motor_app.h"
#include "motor_thread.h"
#include "controller.h"
#include <cmath>


using namespace obot;

static double sat(double val, double limit) {
    return std::max(std::min(limit,val), -limit);
}

double current_gain = 500;

class Task : public MotorThread {
 public:
  Task() : MotorThread(1000) {
        torque_controller_.kp = 1;
        torque_controller_.kd = 0.001;
	}
	~Task() {
	}
 protected:
    virtual void post_init() {
        motor_manager_.set_command_mode(ModeDesired::CURRENT);
    }

	virtual void pre_update() {
        t_ += 0.001;
        double position_desired = std::sin(2*M_PI*t_);
        current_desired_[0] = position_controller_.update(position_desired, data_.statuses[0].motor_position, data_.statuses[0].mcu_timestamp);
        current_desired_[0] = torque_controller_.update(0, data_.statuses[1].torque, data_.statuses[1].mcu_timestamp);
        current_desired_[0] = sat(current_desired_[0], 10.0);
        motor_manager_.set_command_count(x_++);
        motor_manager_.set_command_current(current_desired_);
	}

 private:
    double t_ = 0;
	uint32_t x_ = 0;
    std::vector<float> current_desired_ = {0,0};
    PositionController position_controller_;
    PositionController torque_controller_;
};

class TorqueLimitApp : public MotorApp {
 public:
    TorqueLimitApp(int argc, char **argv, MotorThread *motor_thread) : 
        MotorApp(argc, argv, motor_thread) {}
    virtual void select_motors(MotorManager *m) {
        std::vector<std::string> motor_names = {"A", "J1"};   
        m->get_motors_by_name(motor_names);
        std::cout << m->motors().size() << std::endl;
    }
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = TorqueLimitApp(argc, argv, &task);
	return app.run();
}
