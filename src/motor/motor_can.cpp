#include "motor_can.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#if __has_include(<charconv>)
#include <charconv>
#endif

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
    CANFile(std::string ifname, uint32_t devnum) : devnum_(devnum) {
        ifname_ = ifname;

        // lock file to prevent multiple instances at the same time
        lock_file_ = "/tmp/obot." + ifname + ":" + std::to_string(devnum) + ".lock";
        fd_lock_ = ::open(lock_file_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_lock_ < 0) {
            throw RuntimeException("Error opening lock file " + lock_file_ + ":" + std::to_string(errno) + ": " + strerror(errno));
        }
        int err = ::lseek(fd_lock_, 0, SEEK_SET);
        if (err < 0) {
            throw RuntimeException("Error lseek lock file " + lock_file_ + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
        open();
    }

    void open() {
        fd_ = MotorCAN::open_socket(ifname_);
        struct can_filter rfilter[1];
        rfilter[0].can_id   = 5 << 7 | devnum_;
        rfilter[0].can_mask = 0x7FF | CAN_EFF_FLAG | CAN_RTR_FLAG;

        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))) {
            throw RuntimeException("Error setting filter for " + name() + ": "
                + std::to_string(errno) + ": " + strerror(errno));
        }
    }

    void close() {
        ::close(fd_);
    }

    void flush() {
        // flush frames received before filter
        canfd_frame frame;
        while (true) {
            int retval = ::read(fd_, &frame, sizeof(frame));
            if (retval < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                } else {
                    throw RuntimeErrnoException("read error during flush " + name());
                }
            }
        }
    }

    // use a lock file to provide exclusive access to the CAN device during a 
    // write followed by read interface to the text api
    int lock() {
        int err = lockf(fd_lock_, F_LOCK, 1);
        if (err) {
            std::cerr << "error locking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", already locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        // option 1: open and close the socket - throughput at 600 packets/second
        // option 2: flush the socket - throughput at 620 packets/second
        // open();
        flush();
        return err;
    }

    int unlock() {
        // close();
        int err = lockf(fd_lock_, F_ULOCK, 0);
        if (err) {
            std::cerr << "error unlocking " + lock_file_;
            pid_t pid;
            int err2 = get_lock_pid(fd_lock_, &pid);
            if (err2 == 0) {
                std::cerr << ", locked by process: " << pid;
            }
            std::cerr << std::endl;
        }
        return err;
    }

    ssize_t _read(char * data, unsigned int length) {
        struct canfd_frame frame;
        pollfd poll_fd {
            .fd = fd_,
            .events = POLLIN
        };
        int nbytes = -1;
        int length_recv = 0;
        int can_id = 5 << 7 | devnum_;

        // flush but save read frame
        while (true) {
            canfd_frame tmp_frame;
            int tmp_nbytes = ::read(fd_, &tmp_frame, sizeof(tmp_frame));
            if (tmp_nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                } else {
                    throw RuntimeErrnoException("read error during flush " + name());
                }
            } else {
                frame = tmp_frame;
                nbytes = tmp_nbytes;
            }
        }

        // if nothing from the no timeout flush/read, then do a timeout read
        if (nbytes < 0) {
            if (int poll_result = poll(&poll_fd, 1, timeout_ms_); poll_result < 0) {
                throw RuntimeErrnoException("poll error in read " + name());
            } else if (poll_result == 0) {
                throw RuntimeException("poll timeout in read " + name());
            }

            nbytes = ::read(fd_, &frame, sizeof(frame));
        }

        if (nbytes > 0) {
            if (frame.can_id == can_id) {
                length_recv = std::min(length, (unsigned int) frame.len);
                if (frame.data[0] != 0) {
                    // an ascii packet, not a special control packet
                    // search for embedded 0 terminator
                    length_recv = strnlen((const char*) frame.data, length_recv);
                }
                std::memcpy(data, frame.data, length_recv);
            }
        }
        return length_recv;
    }

    virtual ssize_t read(char * data, unsigned int length) {
        ssize_t retval = _read(data, length);
        
        if (retval >= sizeof(APIControlPacket) && data[0] == 0) {
            // a control packet
            APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data);
            if (packet->type == TIMEOUT_REQUEST) {
                // timeout request
                if (retval == sizeof(APIControlPacket)) {
                    // timeout request
                    // retriggers the read with the new timeout
                    uint32_t old_timeout_ms = timeout_ms_;
                    timeout_ms_ += packet->timeout_request.timeout_us/1000;
                    ssize_t retval = read(data, length);
                    timeout_ms_ = old_timeout_ms;
                    return retval;
                }
            } else if (packet->type == LONG_PACKET) {
                // long packet
                uint16_t total_length = packet->long_packet.total_length;
                uint16_t packet_number = packet->long_packet.packet_number;
                const uint8_t header_size = sizeof(APIControlPacket);
                uint16_t total_count_received = retval - header_size;
                if (total_length > length) {
                    // too long
                    return -EINVAL;
                }
                std::memmove(data, data + header_size, total_count_received);
                while (total_length > total_count_received) {
                    // assemble multiple packets
                    char * data_ptr = data + total_count_received;
                    retval = _read(data_ptr, length);
                    if (retval < 0) {
                        return retval;
                    }
                    APIControlPacket * packet = reinterpret_cast<APIControlPacket *>(data_ptr);
                    if (packet->type != LONG_PACKET) {
                        std::cerr << "Error: expected long packet, got " << packet->type << std::endl;
                        return -EINVAL;
                    }
                    if (packet->long_packet.packet_number != ++packet_number) {
                        std::cerr << "Error: expected packet number " << packet_number << ", got " << packet->long_packet.packet_number << std::endl;
                        return -EINVAL;
                    }
                    //std::cout << packet->long_packet.packet_number << retval << std::endl;
                    total_count_received += retval - header_size;
                    std::memmove(data_ptr, data_ptr + header_size, retval - header_size);
                }
                if (total_count_received < total_length) {
                    std::cerr << "Error: expected " << total_length << " bytes, got " << total_count_received << std::endl;
                    return -EINVAL;
                }
                retval = total_count_received;
            }
        }
        unlock();
        return retval;
    }


    virtual ssize_t write(const char * data, unsigned int length) {
        lock();
        struct canfd_frame frame = {};
        length = std::min(length, (unsigned int) CANFD_MAX_DLEN-1);
        frame.can_id  = 4 << 7 | devnum_;
        frame.len = length + 1;
        frame.flags = CANFD_BRS;
        std::memcpy(frame.data, data, length);
        frame.data[length] = 0;

        int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            throw RuntimeException("Error writing canfile " + name() + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
        return nbytes;
    }

    virtual ssize_t writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
        //int err = lock();
        // if (err) {
        //     return err;
        // }
        ssize_t nbytes = write(data_out, length_out);
        if (nbytes < 0) {
            return nbytes;
        }
        int retval = read(data_in, length_in);
        //err = unlock();
        // if (err) {
        //     return err;
        // }
        return retval;
    }

    std::string name() const {
        return ifname_ + ":" + std::to_string(devnum_);
    }

    int fd_;
    uint32_t devnum_;
    int timeout_ms_ = 10;
    int fd_lock_;
    std::string ifname_;
    std::string lock_file_;
};

MotorCAN::MotorCAN(std::string address) {
    int n = address.find(":");
    if (n == std::string::npos) {
        throw RuntimeException("Error parsing address " + address + ": missing ':'");
    } else {
        dev_path_ = address.substr(0, n);
        std::string tmp = address.substr(n+1,-1);
        if (!tmp.size()) {
           devnum_ = 1;
        } else {
            try {
                // base 0 for auto-detect base
                devnum_ = std::stoi(tmp, nullptr, 0);
            } catch (std::exception &e) {
                throw RuntimeException("Error parsing address " + address + ": " + e.what());
            }
        }
    }
    open();
    struct can_filter rfilter[1];
    rfilter[0].can_id   = 3 << 7 | devnum_;
    rfilter[0].can_mask = 0x7FF | CAN_EFF_FLAG | CAN_RTR_FLAG;

    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter))) {
        throw RuntimeException("Error setting filter for " + dev_path_ + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

    motor_txt_ = std::move(std::unique_ptr<CANFile>(new CANFile(dev_path_, devnum_)));
	
    messages_version_ = operator[]("messages_version").get();
    name_ = operator[]("name").get();
    version_ = operator[]("version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    // hex in base_path_ for convenience
#if __has_include(<charconv>)
    char buffer[5] = {"0x"};
    std::to_chars(buffer+2, buffer+4, devnum_, 16);
    base_path_ = buffer;
#endif
}

uint32_t MotorCAN::timeout_ms_ = 10;
void MotorCAN::set_timeout_ms(int timeout_ms) {
    timeout_ms_ = timeout_ms;
    static_cast<CANFile*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
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
		throw RuntimeException("Error opening socket for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on))){
        throw RuntimeException("Error enabling canfd for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
    }

    strcpy(ifr.ifr_name, ifname);
    
    if (if_name == "any") {
        ifr.ifr_ifindex = 0;
    } else {	
	    if(ioctl(fd, SIOCGIFINDEX, &ifr)) {
            throw RuntimeException("Error getting ifindex for " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    }
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
        throw RuntimeErrnoException("socket set non-block failed for " + if_name);
    }

	// printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw RuntimeException("Error binding " + if_name + ": " + std::to_string(errno) + ": " + strerror(errno));
	}
    return fd;
}

ssize_t MotorCAN::aread() {
    struct canfd_frame frame = {
        .can_id = 3 << 7 | devnum_,
        .len = 0,
        .flags = CANFD_BRS
    };
    if (int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
        nbytes < 0) {
        throw RuntimeErrnoException("write read request error");
    }
    return 0;
}

ssize_t MotorCAN::read() {
    struct canfd_frame frame;

    int nbytes = -1;
    int length_recv = 0;

    pollfd poll_fd {
        .fd = fd_,
        .events = POLLIN
    };

    // flush but save read frame
    while (true) {
        canfd_frame tmp_frame;
        int tmp_nbytes = ::read(fd_, &tmp_frame, sizeof(tmp_frame));
        if (tmp_nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            } else {
                throw RuntimeErrnoException("read error during flush " + dev_path());
            }
        } else {
            frame = tmp_frame;
            nbytes = tmp_nbytes;
        }
    }

    // if nothing from the no timeout flush/read, then do a timeout read
    if (nbytes < 0) {
        if (int poll_result = poll(&poll_fd, 1, timeout_ms_); poll_result < 0) {
            throw RuntimeErrnoException("poll error in read " + dev_path());
        } else if (poll_result == 0) {
            throw RuntimeException("poll timeout in read " + dev_path());
        }

        nbytes = ::read(fd_, &frame, sizeof(frame));
    }

    if (nbytes > 0) {
        if (frame.can_id == 3 << 7 | devnum_) {
            length_recv = std::min((int)frame.len, (int)sizeof(status_));
            std::memcpy(&status_, frame.data, length_recv);
        }
    }
    return length_recv;
}

ssize_t MotorCAN::write() {
    struct canfd_frame frame = {
        // 1 : command, 2: command/req status
        .can_id = ((cmd_status_req_ ? 2 : 1) << 7) | devnum_,
	    .len = 48, //sizeof(command_);
        .flags = CANFD_BRS
    };
	std::memcpy(frame.data, &command_, sizeof(command_));

	int nbytes = ::write(fd_, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
        throw RuntimeException("Error writing can " + dev_path() + ": " + std::to_string(errno) + ": " + strerror(errno));
    }
    return nbytes;
}


static std::vector<std::string> get_can_interfaces() {
    std::vector<std::string> interfaces;
    struct ifaddrs *addrs,*tmp;

    if (getifaddrs(&addrs)) {
        throw RuntimeException("Error getting interfaces: " + std::to_string(errno) + ": " + strerror(errno));
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
        } catch (RuntimeException &e) {}
        
        tmp = tmp->ifa_next;
    }
    if (interfaces.empty()) {
        throw RuntimeException("No valid CAN interfaces found");
    } else {
        std::cout << "Found CAN interfaces: ";
        for (auto &s : interfaces) {
            std::cout << s << " ";
        }
        std::cout << std::endl;
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
    rfilter[0].can_mask = 0x780 | CAN_EFF_FLAG | CAN_RTR_FLAG;

    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))) {
        throw RuntimeException("Error setting filter for " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
    }


    for (std::string &interface : interfaces) {
        int write_fd = open_socket(interface);

        struct canfd_frame frame = {};
        frame.can_id  = 0xf << 7 | 0x7f | CAN_RTR_FLAG;
        frame.len = 0;

        int nbytes = ::write(write_fd, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            throw RuntimeException("Error writing can " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    }

    pollfd tmp;
    tmp.fd = fd;
    tmp.events = POLLIN;
    Timer t(timeout_ms_ * 1000 * 1000); // 10 ms
    do {
        struct timespec timeout = {};
        timeout.tv_nsec = t.get_time_remaining_ns();
        if (timeout.tv_nsec == 0) {
            break;
        }
        int poll_result = ::ppoll(&tmp, 1, &timeout, nullptr /*sigmask*/);
        if (poll_result > 0) {
            struct canfd_frame frame;
            struct sockaddr_can addr;
            socklen_t len = sizeof(addr);
            int nbytes = recvfrom(fd, &frame, sizeof(struct can_frame),
                  0, (struct sockaddr*)&addr, &len);
            struct ifreq ifr = {};
            ifr.ifr_ifindex = addr.can_ifindex;
            ioctl(fd, SIOCGIFNAME, &ifr);
            if (nbytes >= 0) {
                int devnum = frame.can_id & 0x7F;
                devices.push_back(std::string(ifr.ifr_name) + ":" + std::to_string(devnum));
            } else {
                throw RuntimeException("Error reading " + interface + "(" + std::string(ifr.ifr_name) + ")" ": " + std::to_string(errno) + ": " + strerror(errno));
            }
        } else if (poll_result < 0) {
            throw RuntimeException("Error polling " + interface + ": " + std::to_string(errno) + ": " + strerror(errno));
        }
    } while (t.get_time_remaining_ns() > 0);

    return devices;
}

}; // namespace obot
