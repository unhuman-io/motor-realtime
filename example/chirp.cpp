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
	virtual void pre_update() {
		motor_manager_.set_command_mode(POSITION);
		phi_ += dt_ * frequency_ramp_;
		float position_desired = amplitude_*std::sin(2*M_PI*phi_);
		motor_manager_.set_command_position([position_desired]);
	}
	virtual void post_init() {
		auto motor = motor_manager_.motors()[0];
		(*motor)["vlimit"] = "10";
		(*motor)["desired_filter"] = "10";
	}

 private:
	double dt_  = 1./2000;
	double phi_ = 0;
	double frequency_ramp_ = .1;
	double amplitude_ = 1;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task, "Motor chirp");
	return app.run();
}
