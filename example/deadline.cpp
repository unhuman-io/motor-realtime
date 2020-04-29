#include <iostream>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <errno.h>

#include <string>
#include <vector>
#include <algorithm>
#include "motor.h"
#include "motor_manager.h"
#include <fstream>
#include "controller.h"
#include <cmath>
#include <poll.h>
#include "realtime_thread.h"
#include <thread>

struct Data {
  std::vector<Status> statuses;
	std::vector<Command> commands;
	std::chrono::steady_clock::time_point time_start, last_time_start, last_time_end, aread_time, read_time, control_time, write_time;
};

// A circular stack. If data is written by one thread and read by one other thread, data is read from the top without worrying about thread safety.
template <class T>
class CStack {
 public:
    void push(T &t) {
		int future_pos = pos_ + 1;
		if (future_pos >= 100) {
			future_pos = 0;
		}
		data_[future_pos] = t;
		pos_ = future_pos;
	}
	T top() {
		return data_[pos_];
	}
 private:
	T data_[100];
	int pos_ = 0;
};

class Task : public RealtimeThread {
 public:
  Task(CStack<Data> &cstack, MotorManager &motors) : cstack_(cstack), motors_(motors), 
			controller_(motors.motors().size()), RealtimeThread(2000) {
		motors_.open();
		std::cout << "Connecting to motors:" << std::endl;
		for (auto m : motors_.motors()) {
			std:: cout << m->name() << std::endl;
		}
		data_.commands.resize(motors_.motors().size());
		data_.statuses.resize(motors_.motors().size());

		controller_.set_current(1);
		controller_.set_position(0);
		controller_.set_mode(Controller::CURRENT);

		if (pipe_) {
			umask(0000);
			mkfifo("/tmp/deadline", 0666);
			pipe_fd_ = open("/tmp/deadline", O_RDWR | O_NONBLOCK); // read write so this keeps the fifo open
		}
		x=0;
	}
	~Task() {
		if (pipe_) {
			close(pipe_fd_);
			remove("/tmp/deadline");
		}
		motors_.close();
	}
 protected:
	virtual void update() {
		x++;
		data_.last_time_start = data_.time_start;
		data_.time_start = std::chrono::steady_clock::now();

		// start a read on all motors
		motors_.aread();
		// poll all motors, will block until at least one has data
		//motors_.poll();
		data_.aread_time = std::chrono::steady_clock::now();

		if (pipe_) {
			// check for data on the pipe, timeout before a usb read is likely to finish
			pollfd pipe_fds[] = {{.fd=pipe_fd_, .events=POLLIN}};
			timespec timeout_ts = {.tv_nsec=100 * 1000};
			int retval = ppoll(pipe_fds, 1, &timeout_ts, NULL);
			if (retval) {
				char data[motors_.serialize_command_size()];
				int n = read(pipe_fd_, data, motors_.serialize_command_size());
				//printf("read %d bytes\n",n);
				if (n == motors_.serialize_command_size()) {
					motors_.deserialize_saved_commands(data);
				}
			}
			//pipe_commands_ = pipe_.read();
		}

		// blocking io to get the data alread set up and wait if not ready yet
		data_.statuses = motors_.read();
		data_.read_time = std::chrono::steady_clock::now();

		if (!pipe_) { // a demo
			motors_.set_command_count(x);
			motors_.set_command_mode(ModeDesired::POSITION);
	
			// a square wave in position
			//double position = std::chrono::duration_cast<std::chrono::seconds>(data_.time_start - start_time_).count() % 2;
			double position = 10*std::sin(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
			double velocity = 10*std::cos(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
			motors_.set_command_position(std::vector<float>(motors_.motors().size(), position));
			motors_.set_command_velocity(std::vector<float>(motors_.motors().size(), velocity));

			
		}
		
		data_.control_time = std::chrono::steady_clock::now();
		motors_.write_saved_commands();
		data_.commands = motors_.commands();
		data_.write_time = std::chrono::steady_clock::now();

		cstack_.push(data_);
		data_.last_time_end = std::chrono::steady_clock::now();
}

 private:
	CStack<Data> &cstack_;
	Data data_;
	
	std::chrono::steady_clock::time_point start_time_;
	int fid_;
	int fid_flags_;
	MotorManager &motors_;
	Controller controller_;
	int first_switch_ = 1;
	bool pipe_ = true;
	int pipe_fd_ = 0;
	uint32_t x;
};

#include <csignal>
sig_atomic_t volatile running = 1;

int main (int argc, char **argv)
{
	MotorManager motor_manager;
	//auto motors = motor_manager.get_motors_by_name({"J1", "J2", "J3", "J4", "J5", "J6"});
	// or just get all the motors
	auto motors = motor_manager.get_connected_motors();
	printf("main thread [%d]\n", gettid());
	CStack<Data> cstack;
	Task task(cstack, motor_manager);
	task.run();
	std::chrono::steady_clock::time_point system_start = std::chrono::steady_clock::now();
	std::ofstream file;
	file.open("data.csv");
	file << "timestamp, " << std::endl;


	signal(SIGINT, [] (int signum) {running = 0;});

	for(int i=0;; i++) {
		if (!running) {
			break;
		}
		Data data = cstack.top();
		int32_t count = 0;
		int32_t count_received = 0;
		if(data.commands.size()) {
			count = data.commands[0].host_timestamp;
		}
		if(data.statuses.size()) {
			count_received = data.statuses[0].host_timestamp_received;
		}
		auto last_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(data.last_time_end - data.last_time_start).count();
		auto last_period =  std::chrono::duration_cast<std::chrono::nanoseconds>(data.time_start - data.last_time_start).count();
		auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(data.time_start - system_start).count();
		std::cout << "last_period: " << last_period << " last_exec: " << last_exec 
				<< " count_received: " << count_received << " current_count: " << count 
				<< " aread_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.aread_time - data.time_start).count()
				<< " read_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.read_time - data.time_start).count()
				<< " control_exec: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.control_time - data.read_time).count()
				<< " write_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.write_time - data.time_start).count()
				<< std::endl;
			
		for (int j=0; j<500; j++) {
			data = cstack.top();
			file << data.time_start.time_since_epoch().count() << ", " << data.commands << data.statuses << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	task.done();

	printf("main dies [%d]\n", gettid());
	return 0;
}
