#include "can_time_sync.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace obot {

namespace {
const canid_t time_id = 0 << 7 | 0x7F;  // the time message, to all nodes
const uint8_t sync_type = 0x10, follow_up_type = 0x18;
const int echo_timeout_ms = 100;
const int64_t second_ns = 1000000000;

int64_t to_ns(const timespec &t) { return t.tv_sec * second_ns + t.tv_nsec; }

void big_endian(uint8_t *bytes, uint32_t value) {
  bytes[0] = value >> 24;
  bytes[1] = value >> 16;
  bytes[2] = value >> 8;
  bytes[3] = value;
}
}  // namespace

CANTimeSync::CANTimeSync(const std::string &interface, uint8_t domain)
    : interface_(interface), domain_(domain & 0xF) {
  const auto fail = [this](const std::string &what) {
    const int saved = errno;
    if (fd_ >= 0) ::close(fd_);
    throw std::runtime_error(what + " " + interface_ + ": " + strerror(saved));
  };
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) fail("no can socket for");
  ifreq ifr = {};
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) fail("no interface");
  // Only this socket's own SYNC comes back here, with the kernel's time for it
  can_filter filter = {time_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
  const int on = 1;
  const int timestamps = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0 ||
      ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0 ||
      ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPING, &timestamps, sizeof(timestamps)) < 0) {
    fail("socket options for");
  }
  sockaddr_can address = {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) fail("bind to");
}

CANTimeSync::~CANTimeSync() {
  if (fd_ >= 0) ::close(fd_);
}

CANTimeSync::Result CANTimeSync::send() {
  Result result;
  const uint8_t sequence = sequence_;
  sequence_ = (sequence_ + 1) & 0xF;
  timespec before;
  clock_gettime(CLOCK_REALTIME, &before);
  const uint32_t seconds = static_cast<uint32_t>(before.tv_sec);

  can_frame sync = {};
  sync.can_id = time_id;
  sync.can_dlc = 8;
  sync.data[0] = sync_type;
  sync.data[2] = domain_ << 4 | sequence;
  big_endian(sync.data + 4, seconds);
  if (::write(fd_, &sync, sizeof(sync)) != sizeof(sync)) return result;

  // The SYNC back from the kernel, with its time. Other hosts' time frames
  // and this one's earlier ones are passed over.
  int64_t time_ns = 0;
  const int64_t deadline = to_ns(before) + echo_timeout_ms * 1000000ll;
  while (time_ns == 0) {
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    const int64_t left_ms = (deadline - to_ns(now)) / 1000000;
    pollfd poll_fd = {fd_, POLLIN, 0};
    if (left_ms <= 0 || ::poll(&poll_fd, 1, left_ms) <= 0) return result;
    can_frame frame;
    char control[256];
    iovec iov = {&frame, sizeof(frame)};
    msghdr message = {};
    message.msg_iov = &iov;
    message.msg_iovlen = 1;
    message.msg_control = control;
    message.msg_controllen = sizeof(control);
    if (::recvmsg(fd_, &message, 0) != sizeof(frame)) continue;
    if (!(message.msg_flags & MSG_CONFIRM) || std::memcmp(frame.data, sync.data, 8) != 0) continue;
    for (cmsghdr *c = CMSG_FIRSTHDR(&message); c != nullptr; c = CMSG_NXTHDR(&message, c)) {
      if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPING) {
        timespec stamps[3];
        std::memcpy(stamps, CMSG_DATA(c), sizeof(stamps));
        time_ns = to_ns(stamps[0]);  // the kernel's, in CLOCK_REALTIME
      }
    }
    if (time_ns == 0) return result;  // no time with it
  }

  // seconds + overflow + nanoseconds is the time; the overflow has two bits
  const int64_t since = time_ns - seconds * second_ns;
  if (since < 0 || since >= 4 * second_ns) return result;  // the clock was set in between
  can_frame follow_up = {};
  follow_up.can_id = time_id;
  follow_up.can_dlc = 8;
  follow_up.data[0] = follow_up_type;
  follow_up.data[2] = domain_ << 4 | sequence;
  follow_up.data[3] = static_cast<uint8_t>(since / second_ns);
  big_endian(follow_up.data + 4, static_cast<uint32_t>(since % second_ns));
  if (::write(fd_, &follow_up, sizeof(follow_up)) != sizeof(follow_up)) return result;
  result.sent = true;
  result.time_ns = time_ns;
  result.delay_ns = time_ns - to_ns(before);
  return result;
}

int can_time_sync(const std::vector<std::string> &interfaces, double period_s,
                  uint8_t domain, bool verbose, std::atomic<bool> &signal_exit) {
  std::vector<std::unique_ptr<CANTimeSync>> senders;
  try {
    for (const std::string &interface : interfaces) {
      senders.push_back(std::make_unique<CANTimeSync>(interface, domain));
    }
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  if (period_s < 0.001) period_s = 0.001;
  const int64_t period_ns = static_cast<int64_t>(period_s * 1e9);
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  while (!signal_exit) {
    for (auto &sender : senders) {
      const CANTimeSync::Result result = sender->send();
      if (!result.sent) {
        std::cerr << sender->interface() << ": time not sent" << std::endl;
      } else if (verbose) {
        std::cout << sender->interface() << " " << result.time_ns / second_ns << "."
                  << std::setw(9) << std::setfill('0') << result.time_ns % second_ns
                  << std::setfill(' ') << " after " << result.delay_ns / 1000 << " us"
                  << std::endl;
      }
    }
    const int64_t at = to_ns(next) + period_ns;
    next.tv_sec = at / second_ns;
    next.tv_nsec = at % second_ns;
    // interrupted by the signal that sets signal_exit
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  }
  return 0;
}

}  // namespace obot
