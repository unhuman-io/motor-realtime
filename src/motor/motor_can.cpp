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

namespace obot {

MotorCAN::MotorCAN(std::string address) {

    std::cout << "MotorCAN constructor: " << address << std::endl;
    address_ = address;
    open();
	messages_version_ = MOTOR_MESSAGES_VERSION;
}

void MotorCAN::open() {
	struct sockaddr_can addr;
	struct ifreq ifr;
    int canfd_on = 1;

	const char *ifname = address_.c_str();

	if ((fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		throw std::runtime_error("Error opening socket for " + address_ + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on))){
        throw std::runtime_error("Error enabling canfd for " + address_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

	strcpy(ifr.ifr_name, ifname);
	ioctl(fd_, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw std::runtime_error("Error binding " + address_ + ": " + std::to_string(errno) + ": " + strerror(errno));
	}


}

ssize_t MotorCAN::read() {
    return 0;
}

ssize_t MotorCAN::write() {
    struct canfd_frame frame = {};
	frame.can_id  = 0x111;
	frame.len = 48;//sizeof(command_);
	std::memcpy(frame.data, &command_, sizeof(command_));

	int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
        throw std::runtime_error("Error writing can " + address_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    return nbytes;
}

}; // namespace obot
