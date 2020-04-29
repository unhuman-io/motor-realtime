#include "motor_app.h"
#include "motor_thread.h"
#include <cmath>

class Task : public MotorThread {
 public:
  Task() : MotorThread(2000) {}
 protected:
	virtual void pre_update() {
		motor_manager_.set_command_count(x_++);
		motor_manager_.set_command_mode(ModeDesired::POSITION);

		// a square wave in position
		//double position = std::chrono::duration_cast<std::chrono::seconds>(data_.time_start - start_time_).count() % 2;
		double position = 10*std::sin(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
		double velocity = 10*std::cos(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
		motor_manager_.set_command_position(std::vector<float>(motor_manager_.motors().size(), position));
		motor_manager_.set_command_velocity(std::vector<float>(motor_manager_.motors().size(), velocity));
	}

 private:
	uint32_t x_ = 0;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task);
	return app.run();
}
