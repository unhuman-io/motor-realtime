// A switch pro controller to differential drive two wheel motors
#include "motor_manager.h"

#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <iomanip>

#include "motor_app.h"
#include "motor_thread.h"
#include <poll.h>

using namespace obot;

struct Joystick {
    uint8_t buttons[6];
    uint8_t l_stick[3]; 
    uint8_t r_stick[3];
    uint8_t pad[52];
} joystick;

double current_gain = 500;

class Task : public MotorThread {
 public:
  Task() : MotorThread(2000) {
		fd_ = open("/dev/hidraw8", O_RDONLY);
	}
	~Task() {
		close(fd_);
	}
 protected:
    virtual void post_init() {
        //motor_manager_.set_command_mode(ModeDesired::CURRENT);
        motor_manager_.set_command_mode(ModeDesired::VELOCITY);
    }

	virtual void pre_update() {
		// check for data on the pipe, timeout before a usb read is likely to finish
		pollfd poll_fds[] = {{.fd=fd_, .events=POLLIN}};
		timespec timeout_ts = {};
        timeout_ts.tv_nsec=100 * 1000;
		int retval = ppoll(poll_fds, 1, &timeout_ts, nullptr);
		if (retval) {
            read(fd_, &joystick, sizeof(joystick));
            uint16_t ud = (joystick.l_stick[2] << 4l) | ((joystick.l_stick[1] & 0xF0) >> 4);
            uint16_t lr = joystick.l_stick[0] | ((joystick.l_stick[1] & 0xF) << 8l);
            //uint16_t ud = joystick.l_stick[1] >> 4;
            //uint16_t lr = joystick.l_stick[0] >> 4;
            double ud2 = -1*((int16_t) ud - 2048)/1500.;
            double lr2 = ((int16_t) lr - 2048)/1500.;
            //double enable = (joystick.buttons[0] & 0x200) ? 1.0 : 0.0;
            double enable = (joystick.buttons[3] & 0x8) ? 1.0 : 0.0;
            current_desired_[0] = enable*current_gain*(ud2 + lr2);
            double j2 = current_gain*(ud2 - lr2);
            //std::cout << "ud: " << std::setw(12) << ud2 << ", lr:" << lr2 << std::endl;
           // std::cout << current_desired_[0] << std::endl;
            motor_manager_.set_command_count(x_++);
            //motor_manager_.set_command_current(current_desired_);
            motor_manager_.set_command_velocity(current_desired_);
		}
	}

 private:
	int fd_ = 0;
	uint32_t x_ = 0;
    std::vector<float> current_desired_ = {0};
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task);
	return app.run();
}
