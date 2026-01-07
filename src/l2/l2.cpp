
#include "l2.h"
#include "exception.h"
#include <netdb.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <cstring>
#include <poll.h>

namespace obot {

mac_address_t mac_ascii_to_raw(std::string mac_ascii) {
    mac_address_t mac;
    if (sscanf(mac_ascii.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
        &mac[0], &mac[1], &mac[2],
        &mac[3], &mac[4], &mac[5]) == 6) {
    } else {
        throw RuntimeException("Invalid MAC address format: " + mac_ascii);
    }
    return mac;
}

void L2Socket::open() {
  fd_ = ::socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (fd_ < 0) {
    throw RuntimeErrnoException("socket create failed");
  }

  sockaddr_ll server_addr = {};
  server_addr.sll_family = AF_PACKET;
  server_addr.sll_protocol = htons(0x88B5);
  server_addr.sll_ifindex = if_nametoindex(interface_.c_str());

  int retval = bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (retval < 0) {
    throw RuntimeErrnoException("bind failed");
  }
}

void L2Socket::send(const char * data, std::size_t length) {
    std::memcpy(&frame_out_.payload, data, length);
    int send_result = ::send(fd_, &frame_out_, length+14, 0);
}

int L2Socket::recv() {
    pollfd tmp;
    tmp.fd = fd_;
    tmp.events = POLLIN;
    int result = ::poll(&tmp, 1, 0);
    if (result > 0) {
      result = ::recv(fd_, &frame_in_, sizeof(frame_in_), 0);
      if (result < 0) {
        throw RuntimeErrnoException("Error recv");
      }
    }
    return result;
}

L2Device::L2Device(std::string interface, std::string mac_address) : L2Socket(interface) {
  mac_address_t src_mac = mac_ascii_to_raw(mac_address);
  std::memcpy(frame_out_.src_mac, &src_mac, 6);
}

} // namespace obot
