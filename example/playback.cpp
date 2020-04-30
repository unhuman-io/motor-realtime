#include "motor_app.h"
#include "motor_thread.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

class Task : public MotorThread {
 public:
  Task() : MotorThread(1000) {}
 protected:
	virtual void post_init() {
		data_file_.open("data1.csv");
		std::string s;
		std::getline(data_file_, s);	// headers
	}
	virtual void pre_update() {
		std::string s, s2;
		std::getline(data_file_, s);
		std::istringstream iss(s);
		uint64_t timestamp;
		std::vector<Command> commands;
		commands.resize(motor_manager_.motors().size());
		//std::cout << s << x_++ << std::endl;
		iss >> timestamp >> s2 >> commands;
		std::cout << timestamp << ", " << commands << std::endl;
		motor_manager_.set_commands(commands);
	}

 private:
	uint32_t x_ = 0;
	std::ifstream data_file_;
	int size_commands_ = 1;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task);
	return app.run();
}
