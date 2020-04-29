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
#include "motor_thread.h"
#include <thread>



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

class Task : public MotorThread {
 public:
  Task(CStack<Data> &cstack, MotorManager &motors) : cstack_(cstack),
			controller_(motors.motors().size()), MotorThread(motors, 2000) {
		if (pipe_) {
			umask(0000);
			mkfifo("/tmp/deadline", 0666);
			pipe_fd_ = open("/tmp/deadline", O_RDWR | O_NONBLOCK); // read write so this keeps the fifo open
		}
	}
	~Task() {
		if (pipe_) {
			close(pipe_fd_);
			remove("/tmp/deadline");
		}
		motor_manager_.close();
	}
 protected:
	virtual void pre_update() {
		if (pipe_) {
			// check for data on the pipe, timeout before a usb read is likely to finish
			pollfd pipe_fds[] = {{.fd=pipe_fd_, .events=POLLIN}};
			timespec timeout_ts = {.tv_nsec=100 * 1000};
			int retval = ppoll(pipe_fds, 1, &timeout_ts, NULL);
			if (retval) {
				char data[motor_manager_.serialize_command_size()];
				int n = read(pipe_fd_, data, motor_manager_.serialize_command_size());
				//printf("read %d bytes\n",n);
				if (n == motor_manager_.serialize_command_size()) {
					motor_manager_.deserialize_saved_commands(data);
				}
			}
			//pipe_commands_ = pipe_.read();
		}
	}

	virtual void controller_update() {
		if (!pipe_) { // a demo
			motor_manager_.set_command_count(x_++);
			motor_manager_.set_command_mode(ModeDesired::POSITION);
	
			// a square wave in position
			//double position = std::chrono::duration_cast<std::chrono::seconds>(data_.time_start - start_time_).count() % 2;
			double position = 10*std::sin(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
			double velocity = 10*std::cos(std::chrono::duration_cast<std::chrono::nanoseconds>(data_.time_start - start_time_).count() / 1.0e9);
			motor_manager_.set_command_position(std::vector<float>(motor_manager_.motors().size(), position));
			motor_manager_.set_command_velocity(std::vector<float>(motor_manager_.motors().size(), velocity));
		}
	}

	virtual void post_update() {
		cstack_.push(data_);
	}

 private:
	CStack<Data> &cstack_;
	Controller controller_;
	int first_switch_ = 1;
	bool pipe_ = true;
	int pipe_fd_ = 0;
	uint32_t x_ = 0;
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
