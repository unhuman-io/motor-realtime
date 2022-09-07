#include "motor_app.h"
#include "motor_thread.h"
#include <cmath>

class Task : public MotorThread {
 public:
  Task() : MotorThread(2000) {}
 protected:
	virtual void controller_update() {
		motor_manager_.set_command_count(x_++);
        // toggle voltage mode on/off depending on motor encoder reading 0 or 1, a round trip test
        std::vector<uint8_t> mode(motor_manager_.motors().size(),ModeDesired::OPEN);
        for(int i=0; i<motor_manager_.motors().size(); i++) {
            if (data_.statuses[i].motor_encoder) {
                mode[i] = ModeDesired::VOLTAGE;
            }
        }

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
