#include "motor_app.h"
#include "motor_thread.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <motor_publisher.h>
#include <motor_subscriber.h>

using namespace obot;

class Task : public MotorThread {
 public:
  Task(int frequency) : MotorThread(frequency) {
	  
  }
 protected:
	virtual void post_init() {

	}
	virtual void pre_update() {
		command_ = sub_.read();
	}

    virtual void controller_update() {
        std::vector<MotorCommand> commands(motor_manager_.motors().size(), command_);
        motor_manager_.set_commands(commands);
    }

    virtual void post_update() {
        pub_.publish(data_.statuses[0]);
    }

 private:
    MotorSubscriber<MotorCommand> sub_;
    MotorPublisher<MotorStatus> pub_;
    MotorCommand command_ = {};
};

int main (int argc, char **argv)
{	
	Task task(1000);
	auto app = MotorApp(argc, argv, &task);
	return app.run();
}
