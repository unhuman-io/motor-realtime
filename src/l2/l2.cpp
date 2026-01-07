
#include "l2.h"
#include "exception.h"
#include <netdb.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
//#include <string.h>

namespace obot {

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
    int send_result = ::send(fd_, data, length, 0);
}

} // namespace obot
