#include "motor_app.h"
#include "motor_thread.h"

#include <iostream>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <errno.h>

#include <string>
#include <vector>
#include <algorithm>

#include "motor_manager.h"
#include <fstream>
#include <cmath>

#include "motor_thread.h"
#include <thread>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#include <csignal>
sig_atomic_t volatile running = 1;

MotorApp::MotorApp(int /* argc */, char ** /* argv */, MotorThread *motor_thread) 
    : motor_thread_(motor_thread) {

}

void MotorApp::select_motors(MotorManager *m) {
	m->get_connected_motors();
}

int MotorApp::run() {
	//auto motors = motor_manager.get_motors_by_name({"J1", "J2", "J3", "J4", "J5", "J6"});
	// or just get all the motors
    printf("main thread [%ld]\n", gettid());
    auto &motor_manager = motor_thread_->motor_manager();
	select_motors(&motor_manager);
    motor_thread_->init();
	auto &cstack = motor_thread_->cstack();
	
	motor_thread_->run();
	std::ofstream file;
	file.open("data.csv");
	file << "timestamp, " << motor_manager.command_headers() << motor_manager.status_headers() << std::endl;


	signal(SIGINT, [] (int /* signum */) {running = 0;});

	for(int i=0;; i++) {
		if (!running) {
			break;
		}
		Data data = cstack.top();
		int32_t count = 0;
		int32_t count_received = 0;
		if(data.size()) {
			count = data.commands[0].host_timestamp;
		}
		if(data.size()) {
			count_received = data.statuses[0].host_timestamp_received;
		}
		auto last_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(data.last_time_end - data.last_time_start).count();
		auto last_period =  std::chrono::duration_cast<std::chrono::nanoseconds>(data.time_start - data.last_time_start).count();
		std::cout << "last_period: " << last_period << " last_exec: " << last_exec 
				<< " count_received: " << count_received << " current_count: " << count 
				<< " aread_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.aread_time - data.time_start).count()
				<< " read_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.read_time - data.time_start).count()
				<< " control_exec: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.control_time - data.read_time).count()
				<< " write_time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(data.write_time - data.time_start).count()
				<< std::endl;
			
		for (int j=0; j<500; j++) {
			data = cstack.top();
			std::vector<MotorStatus> statuses(data.statuses, data.statuses + motor_manager.motors().size());
			std::vector<MotorCommand> commands(data.commands, data.commands + motor_manager.motors().size());
			file << data.time_start.time_since_epoch().count() << ", " << commands << statuses << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	motor_thread_->done();

	printf("main dies [%ld]\n", gettid());
    return 0;
}
