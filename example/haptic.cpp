#include "motor_app.h"
#include "motor_thread.h"
#include <poll.h>

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
		float torque_desired = -.01 * data_.statuses[0].motor_position;

		motor_manager_.set_command_torque({torque_desired});
	}
	virtual void post_init() {
		auto motor = motor_manager_.motors()[0];

	}

 private:
	double dt_  = 1./2000;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task, "Motor haptics");
	return app.run();
}
