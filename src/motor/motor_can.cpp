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

#include <ifaddrs.h>

#include "poll.h"

namespace obot {

class CANFile : public TextFile {
 public:
    CANFile(int fd, uint32_t devnum) : fd_(fd), devnum_(devnum) {}

    virtual ssize_t read(char * data, unsigned int length) {
        struct canfd_frame frame;
        pollfd tmp;
        tmp.fd = fd_;
        tmp.events = POLLIN;
        int count = 0;
        int nbytes = 0;
        bool success = false;
        do {
            int poll_result = ::poll(&tmp, 1, 10 /* ms */);
            if (poll_result > 0) {
                nbytes = ::read(fd_, &frame, sizeof(struct canfd_frame));
                if (nbytes > 0) {
                    if (frame.can_id == (5 << 7 | devnum_)) {
                        success = true;
                        length = std::min(length, (unsigned int) frame.len);
                        std::memcpy(data, frame.data, length);
                    }
                }
            }
            count++; //todo change to timeout
        } while (!success && count < 10);
        return nbytes;
    }

    virtual ssize_t write(const char * data, unsigned int length) {
        struct canfd_frame frame = {};
        length = std::min(length, (unsigned int) CANFD_MAX_DLEN-1);
        frame.can_id  = 4 << 7 | devnum_;
        frame.len = ++length;
        frame.flags = CANFD_BRS;
        std::memcpy(frame.data, data, length);
        frame.data[length] = 0;

        int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            throw std::runtime_error("Error writing canfile " + std::to_string(devnum_) + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
        return nbytes;
    }

    virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
        ssize_t nbytes = write(data_out, length_out);
        if (nbytes < 0) {
            return nbytes;
        }
        return read(data_in, length_in);
    }

    int fd_;
    uint32_t devnum_;
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
    struct can_filter rfilter[1];
    rfilter[0].can_id   = devnum_;
    rfilter[0].can_mask = 0x7F | CAN_EFF_FLAG | CAN_RTR_FLAG;

    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))) {
        throw std::runtime_error("Error setting filter for " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

    motor_txt_ = std::move(std::unique_ptr<CANFile>(new CANFile(fd_, devnum_)));
	messages_version_ = MOTOR_MESSAGES_VERSION;
}

void MotorCAN::open() {
    fd_ = open_socket(dev_path_);
}

int MotorCAN::open_socket(std::string if_name) {
	struct sockaddr_can addr;
	struct ifreq ifr;
    int canfd_on = 1;

	const char *ifname = if_name.c_str();

    int fd;
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		throw std::runtime_error("Error opening socket for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on))){
        throw std::runtime_error("Error enabling canfd for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

    strcpy(ifr.ifr_name, ifname);
    
    if (if_name == "any") {
        ifr.ifr_ifindex = 0;
    } else {	
	    if(ioctl(fd, SIOCGIFINDEX, &ifr)) {
            throw std::runtime_error("Error getting ifindex for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    }
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	// printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw std::runtime_error("Error binding " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    return fd;
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
            if (frame.can_id == 3 << 7 | devnum_) {
                std::memcpy(&status_, frame.data, sizeof(status_));
            }
        }
    }
    return nbytes;
}

ssize_t MotorCAN::write() {
    struct canfd_frame frame = {};
	frame.can_id  = 2 << 7 | devnum_; // 1 : command, 2: command/req status
	frame.len = 48; //sizeof(command_);
    frame.flags = CANFD_BRS;
	std::memcpy(frame.data, &command_, sizeof(command_));

	int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
        throw std::runtime_error("Error writing can " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    return nbytes;
}


static std::vector<std::string> get_can_interfaces() {
    std::vector<std::string> interfaces;
    struct ifaddrs *addrs,*tmp;

    if (getifaddrs(&addrs)) {
        throw std::runtime_error("Error getting interfaces: " + std::to_string(errno) + ": " + strerror(errno));
    }
    tmp = addrs;

    while (tmp) {
        // std::cout << "interface: " << tmp->ifa_name << " flags " << tmp->ifa_flags << std::endl;
        try {
            // test if interface supports can
            if (tmp->ifa_flags & IFF_UP) {
                int fd = MotorCAN::open_socket(tmp->ifa_name);
                interfaces.push_back(tmp->ifa_name);
                ::close(fd);
            }
        } catch (std::runtime_error &e) {}
        
        tmp = tmp->ifa_next;
    }
    freeifaddrs(addrs);
    return interfaces;
}

std::vector<std::string> MotorCAN::enumerate_can_devices(std::string interface) {
    std::vector<std::string> devices;
    std::vector<std::string> interfaces;
    if (interface == "any") {
        interfaces = get_can_interfaces();
    } else {
        interfaces.push_back(interface);
    }

    int fd = open_socket(interface);
    struct can_filter rfilter[1];
    rfilter[0].can_id   = 0x780;
    rfilter[0].can_mask = 0x780;

    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))) {
        throw std::runtime_error("Error setting filter for " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
    }


    for (std::string &interface : interfaces) {
        int write_fd = open_socket(interface);

        struct canfd_frame frame = {};
        frame.can_id  = 0xf << 7;
        frame.len = 0;

        int nbytes = ::write(write_fd, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            throw std::runtime_error("Error writing can " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    }

    pollfd tmp;
    tmp.fd = fd;
    tmp.events = POLLIN;
    Timer t(10 * 1000 * 1000); // 10 ms
    do {
        struct timespec timeout = {};
        timeout.tv_nsec = t.get_time_remaining_ns();
        int poll_result = ::ppoll(&tmp, 1, &timeout, nullptr /*sigmask*/);
        if (poll_result > 0) {
            struct canfd_frame frame;
            int nbytes = ::read(fd, &frame, sizeof(struct canfd_frame));
            if (nbytes >= 0) {
                int devnum = frame.can_id & 0x7F;
                devices.push_back(interface + ":" + std::to_string(devnum));
            } else {
                throw std::runtime_error("Error reading " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
            }
        } else if (poll_result < 0) {
            throw std::runtime_error("Error polling " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    } while (t.get_time_remaining_ns() > 0);

    return devices;
}

}; // namespace obot
