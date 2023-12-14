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
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo *result, *res;
    int addr_info_result = ::getaddrinfo(ip_.c_str(), port_.c_str(), &hints, &result);
    //std::cout << "addr info result " << addr_info_result << std::endl;
    char addrstr[100];
    void *ptr;


    res = result;
    while(res) {
        //std::cout << "res " << res << std::endl;
        socklen_t addr_len = 0;
      switch (res->ai_family)
        {
        case AF_INET:
          ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
          addr_len = sizeof(sockaddr_in);
          break;
        }
      inet_ntop (res->ai_family, ptr, addrstr, 100);
      // printf ("IPv%d address: %s (%s)\n", res->ai_family == PF_INET6 ? 6 : 4,
      //         addrstr, res->ai_canonname);
      

    
    //std::cout << addrstr << std::endl;
     res = res->ai_next;
    }
    std::memcpy(&addr_, result->ai_addr, result->ai_addrlen);
    //std::cout << "fd_: " << fd_ << std::endl;
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
//     std::cout << "Connecting " << ip_ << std::endl;
//     fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
//     set_timeout_ms(10);

//     addrinfo hints = {};
//     hints.ai_flags = AI_CANONNAME;
//     hints.ai_family = AF_INET;
//     hints.ai_socktype = SOCK_DGRAM;
//     addrinfo *result, *res;
//     int addr_info_result = ::getaddrinfo(ip_.c_str(), "7770", &hints, &result);
//     std::cout << "addr info result " << addr_info_result << std::endl;
//     char addrstr[100];
//     void *ptr;
// char hostname[100];

//     res = result;
//     while(res) {
//         std::cout << "res " << res << std::endl;
//         socklen_t addr_len = 0;
//       switch (res->ai_family)
//         {
//         case AF_INET:
//           ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
//           addr_len = sizeof(sockaddr_in);
//           break;
//         case AF_INET6:
//           ptr = &((struct sockaddr_in6 *) res->ai_addr)->sin6_addr;
//           addr_len = sizeof(sockaddr_in6);
//           break;
//         }
//       inet_ntop (res->ai_family, ptr, addrstr, 100);
//       printf ("IPv%d address: %s (%s)\n", res->ai_family == PF_INET6 ? 6 : 4,
//               addrstr, res->ai_canonname);
      

    
//     std::cout << addrstr << std::endl;
// //EAI_FAMILY
    
//     int name_info_result = ::getnameinfo(res->ai_addr, addr_len, hostname, sizeof(hostname), NULL, 0, 0);
//     std::cout << "name info result " << name_info_result << std::endl;
    
//     std::cout << "hostname: " << hostname << std::endl;
//      res = res->ai_next;
//     }
    
//     // sockaddr_in addr = {};
//     // addr.sin_family = AF_INET;
//     // int result = ::connect(fd_, &addr, sizeof(addr));
//     // std::cout << "result: " << result << std::endl;
//     base_path_ = addrstr;
//     dev_path_ = hostname;
//     std::cout << "fd_: " << fd_ << std::endl;

//     sockaddr_in addr_in = {};
//     addr_in.sin_family = AF_INET;
//     int bind_result = bind(fd_, (sockaddr*) &addr_in, sizeof(addr_in));
//     std::cout << "bind result " << bind_result << " " << errno << std::endl;

//     int send_result = sendto(fd_, "OBOT\x01\x02version", sizeof("OBOT\x02\x01version"), 0, result->ai_addr, result->ai_addrlen);
//     std::cout << "send result " << send_result << std::endl;

//     std::memcpy(&addr_, result->ai_addr, result->ai_addrlen);

//     char buf[1024];
//     int recv_result = recv(fd_, buf, 1023, 0);
//     std::cout << "recv result " << recv_result << " " << buf << std::endl;
//     if (recv_result > 0) {
//       buf[recv_result] = 0;
//       version_ = buf;
//     }

//     send_result = sendto(fd_, "name", sizeof("name"), 0, result->ai_addr, result->ai_addrlen);
//     std::cout << "send result " << send_result << std::endl;

//     recv_result = recv(fd_, buf, 1023, 0);
//     std::cout << "recv result " << recv_result << " " << buf << std::endl;
//     if (recv_result > 0) {
//       buf[recv_result] = 0;
//       name_ = buf;
//     }
//     freeaddrinfo(result);
//     std::cout << "fd " << fd_ << std::endl;
    std::string s = operator[]("messages_version").get(); // todo, don't know why this fixes startup
    //std::cout << "get messages version0: " << s << std::endl;
    
    name_ = operator[]("name").get();
    version_ = operator[]("version").get();
    messages_version_ = operator[]("messages_version").get();
    board_name_ = operator[]("board_name").get();
    board_rev_ = operator[]("board_rev").get();
    board_num_ = operator[]("board_num").get();
    config_ = operator[]("config").get();
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
