#include "motor_can.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "poll.h"

namespace obot {

class CANFile : public TextFile {
 public:
    CANFile() {
    }
    virtual ssize_t read(char * /* data */, unsigned int /* length */, bool write_read = false) {
        return 0;
    }
    virtual ssize_t write(const char * /* data */, unsigned int /* length */, bool write_read = false) {
        return 0;
    }
    virtual ssize_t writeread(const char * /* *data_out */, unsigned int /* length_out */, char * /* data_in */, unsigned int /* length_in */) {
        return 0;
    }
};

MotorCAN::MotorCAN(std::string address) {

    std::cout << "MotorCAN constructor: " << address << std::endl;

    int n = address.find(":");
    if (n == std::string::npos) {
        dev_path_ = address;
        devnum_ = 1;
    } else {
        dev_path_ = address.substr(0, n);
        std::string tmp = address.substr(n+1,-1);
        if (!tmp.size()) {
           devnum_ = 1;
        } else {
            try {
                devnum_ = std::stoi(tmp);
            } catch (std::exception &e) {
                throw std::runtime_error("Error parsing address " + address + ": " + e.what());
            }
        }
    }
    open();
    motor_txt_ = std::move(std::unique_ptr<CANFile>(new CANFile()));
	messages_version_ = MOTOR_MESSAGES_VERSION;
}

void MotorCAN::open() {
	struct sockaddr_can addr;
	struct ifreq ifr;
    int canfd_on = 1;

	const char *ifname = dev_path_.c_str();

	if ((fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		throw std::runtime_error("Error opening socket for " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on))){
        throw std::runtime_error("Error enabling canfd for " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

	strcpy(ifr.ifr_name, ifname);
	ioctl(fd_, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw std::runtime_error("Error binding " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
	}


}

ssize_t MotorCAN::read() {
    struct canfd_frame frame;
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int poll_result = ::poll(&tmp, 1, 10 /* ms */);
    int nbytes = 0;
    if (poll_result > 0) {
        nbytes = ::read(fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes > 0) {
            std::memcpy(&status_, frame.data, sizeof(status_));
        }
    }
    return nbytes;
}

ssize_t MotorCAN::write() {
    struct canfd_frame frame = {};
	frame.can_id  = devnum_;
	frame.len = 48; //sizeof(command_);
	std::memcpy(frame.data, &command_, sizeof(command_));

	int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
        throw std::runtime_error("Error writing can " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    return nbytes;
}

}; // namespace obot
