#include "motor_app.h"
#include "motor_thread.h"
#include <poll.h>

class Task : public MotorThread {
 public:
  Task() : MotorThread(2000) {
		umask(0000);
		mkfifo("/tmp/deadline", 0666);
		pipe_fd_ = open("/tmp/deadline", O_RDWR | O_NONBLOCK); // read write so this keeps the fifo open
	}
	~Task() {
		close(pipe_fd_);
		remove("/tmp/deadline");
		motor_manager_.close();
	}
 protected:
	virtual void pre_update() {
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
	}

 private:
	int pipe_fd_ = 0;
	uint32_t x_ = 0;
};

int main (int argc, char **argv)
{	
	Task task;
	auto app = MotorApp(argc, argv, &task);
	return app.run();
}
