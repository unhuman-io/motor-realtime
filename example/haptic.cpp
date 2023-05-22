#include "motor_app.h"
#include "motor_thread.h"
#include <poll.h>
#include <cmath>

using namespace obot;

class Task : public MotorThread {
 public:
  Task() : MotorThread(2000) {
	}
	~Task() {
	}
 protected:
	virtual void controller_update() {
		motor_manager_.set_command_mode(TORQUE);
		double joint_position = gear_ratio * data_.statuses[0].motor_position;
		double torque_desired = 0;

		for (int i=0; i<num; i++) {
			double offset = -spacing*(num/2) + i*spacing;
			torque_desired += -amplitude * exp(-1/width*std::pow(joint_position - offset,2));
		}

		motor_manager_.set_command_torque({(float) torque_desired});
	}
	virtual void post_init() {
		auto motor = motor_manager_.motors()[0];

	}

 private:
	double dt_  = 1./2000;
	double width = .01;
	double amplitude = 2;
	double spacing = .1;
	int num = 5;
	double gear_ratio = 81;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task, "Motor haptics");
	return app.run();
}
