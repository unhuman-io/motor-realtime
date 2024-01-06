#include "motor_ip.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include "poll.h"

namespace obot {

void UDPFile::open() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    addrinfo hints = {};
    hints.ai_family = AF_INET;      // IPv4
    hints.ai_socktype = SOCK_DGRAM; // UDP

    addrinfo *result, *res;
    int addr_info_result = getaddrinfo(ip_.c_str(), port_.c_str(), &hints, &result);
    if (addr_info_result != 0) {
      throw std::runtime_error("getaddrinfo failed for " + ip_ + ":" + port_ + ", error: " + std::to_string(addr_info_result));
    }

    char addrstr[100];
    void *ptr;
    res = result;
    int n_results = 0;
    while(res) {
        n_results++;
        socklen_t addr_len = 0;
        if (res->ai_family != AF_INET) {
          throw std::runtime_error(ip_ + ":" + port_ + ", not ipv4");
        }
        ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
        addr_len = sizeof(sockaddr_in);
        inet_ntop (res->ai_family, ptr, addrstr, 100);
        addrstr_ = addrstr;
        res = res->ai_next;
    }
    if (n_results != 1) {
      throw std::runtime_error(ip_ + ":" + port_ + ", n_results error: " + std::to_string(n_results));
    }
    std::memcpy(&addr_, result->ai_addr, result->ai_addrlen);
    freeaddrinfo(result);
    flush();
}

void UDPFile::flush() {
    int result;
    char c[1024];
    do {
      result = read(c, 1024);
    } while (result > 0);
}

int UDPFile::poll() {
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int poll_result = ::poll(&tmp, 1, timeout_ms_);
    if (poll_result == 0) {
        return -ETIMEDOUT;
    } else if (poll_result < 0) {
        return poll_result;
    }
    return poll_result;
}

ssize_t UDPFile::read(char * data, unsigned int length) {
    ObotPacket packet;
    packet.command = 2;
    packet.mailbox = recv_mailbox_;
    int send_result = sendto(fd_, &packet, 6, 0, (sockaddr *) &addr_, sizeof(addr_));
    if (send_result < 0) {
      return send_result;
    }

    int poll_result = poll();
    if (poll_result < 0) {
      return poll_result;
    }
    int recv_result = recv(fd_, data, length-1, 0);
    if (recv_result > 0) {
      data[recv_result] = 0;
    }
    return recv_result;
}

ssize_t UDPFile::write(const char * data, unsigned int length) {
    ObotPacket packet;
    packet.command = 1;
    packet.mailbox = send_mailbox_;
    std::memcpy(packet.data, data, length);
    int send_result = sendto(fd_, &packet, 6+length, 0, (sockaddr *) &addr_, sizeof(addr_));
    if (send_result < 0) {
      return send_result;
    }

    int poll_result = poll();
    if (poll_result < 0) {
      return poll_result;
    }
    char data_in[20];
    int recv_result = recv(fd_, data_in, 19, 0);
    if (recv_result > 0) {
      data_in[recv_result] = 0;
    }
    return recv_result;
}

ssize_t UDPFile::writeread(const char * data_out, unsigned int length_out, char * data_in, unsigned int length_in) {
    int write_result = write(data_out, length_out);
    if (write_result < 0) {
      return write_result;
    }
    return read(data_in, length_in);
}

void MotorIP::connect() {
    name_ = operator[]("name").get();
    if (name_ == "") {
      name_ = ip() + ":" + std::to_string(port());
    }
    version_ = operator[]("version").get();
    messages_version_ = operator[]("messages_version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
    serial_number_ = operator[]("serial").get();
    dev_path_ = realtime_communication_.addrstr_;
    base_path_ = ip();
    devnum_ = port();
    fd_flags_ = fcntl(fd_, F_GETFL); 
}

void MotorIP::set_timeout_ms(int timeout_ms) {
    static_cast<UDPFile*>(motor_txt_.get())->timeout_ms_ = timeout_ms;
    realtime_communication_.timeout_ms_ = timeout_ms;
}

ssize_t MotorIP::read() {
    return realtime_communication_.read((char *) &status_, sizeof(status_));
}

ssize_t MotorIP::write() {
    return realtime_communication_.write((char *) &command_, sizeof(command_));
}

}; // namespace obot
