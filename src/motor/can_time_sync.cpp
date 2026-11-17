#include "can_time_sync.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

namespace obot {

// (paragraphs: the help wraps the lines of each)
const char *const can_time_sync_help =
    "Each period two classic CAN frames of 8 bytes go to all nodes, on id 0x07F "
    "(obot message 0, address 0x7F). They are the SYNC and FUP messages of AUTOSAR's "
    "time synchronization over CAN (CanTSyn), the ones without a CRC.\n"
    "\n"
    "SYNC: the bytes 0x10, 0, domain<<4|sequence, 0, and then the seconds in 4 "
    "bytes, big endian.\n"
    "\n"
    "Follow up: the bytes 0x18, 0, domain<<4|sequence, overflow (0 to 3), and then "
    "the nanoseconds in 4 bytes, big endian.\n"
    "\n"
    "SYNC has the seconds of this host's time (CLOCK_REALTIME, seconds since 1970). "
    "The follow up says when exactly that SYNC was sent: its time is the SYNC's "
    "seconds + overflow + nanoseconds. The sequence counts up with each pair, 0 to 15.\n"
    "\n"
    "A node notes on its own clock when a SYNC arrives. With the follow up of the "
    "same sequence it has the host's time for that moment, and sets its clock by it. "
    "A SYNC without its follow up sets nothing.\n"
    "\n"
    "When the SYNC was sent is the kernel's time for the frame: with most drivers "
    "when it went to the driver, not when it was on the bus.";

namespace {
const canid_t time_id = 0 << 7 | 0x7F;  // the time message, to all nodes
const uint8_t sync_type = 0x10, follow_up_type = 0x18;
const int echo_timeout_ms = 100;
const int64_t second_ns = 1000000000;

int64_t to_ns(const timespec &t) { return t.tv_sec * second_ns + t.tv_nsec; }

int64_t now_ns() {
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return to_ns(t);
}

void big_endian(uint8_t *bytes, uint32_t value) {
  bytes[0] = value >> 24;
  bytes[1] = value >> 16;
  bytes[2] = value >> 8;
  bytes[3] = value;
}
}  // namespace

std::vector<std::string> can_time_sync_interfaces() {
  std::vector<std::string> interfaces;
  struct if_nameindex *names = if_nameindex();
  if (names == nullptr) return interfaces;
  for (struct if_nameindex *i = names; i->if_index != 0; i++) {
    const std::string sys = std::string("/sys/class/net/") + i->if_name;
    int type = 0;
    unsigned flags = 0;
    std::ifstream(sys + "/type") >> type;
    std::ifstream(sys + "/flags") >> std::hex >> flags;
    if (type == ARPHRD_CAN && (flags & IFF_UP)) interfaces.push_back(i->if_name);
  }
  if_freenameindex(names);
  std::sort(interfaces.begin(), interfaces.end());
  return interfaces;
}

CANTimeSync::CANTimeSync(const std::string &interface, uint8_t domain)
    : interface_(interface), domain_(domain & 0xF) {
  const auto fail = [this](const std::string &what) {
    const int saved = errno;
    if (fd_ >= 0) ::close(fd_);
    throw std::runtime_error(interface_ + ": " + what + ": " + strerror(saved));
  };
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) fail("no can socket");
  ifreq ifr = {};
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) fail("no such interface");
  // Only this socket's own SYNC comes back here, with the kernel's time for it
  can_filter filter = {time_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
  const int on = 1;
  const int timestamps = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0 ||
      ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0 ||
      ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPING, &timestamps, sizeof(timestamps)) < 0) {
    fail("socket options");
  }
  sockaddr_can address = {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) fail("bind");
}

CANTimeSync::~CANTimeSync() {
  if (fd_ >= 0) ::close(fd_);
}

CANTimeSync::Result CANTimeSync::send() {
  Result result;
  const auto error = [&result](const std::string &what, int number = 0) {
    result.error = number != 0 ? what + ": " + strerror(number) : what;
    return result;
  };
  const uint8_t sequence = sequence_;
  sequence_ = (sequence_ + 1) & 0xF;
  const int64_t before_ns = now_ns();
  const uint32_t seconds = static_cast<uint32_t>(before_ns / second_ns);

  can_frame sync = {};
  sync.can_id = time_id;
  sync.can_dlc = 8;
  sync.data[0] = sync_type;
  sync.data[2] = domain_ << 4 | sequence;
  big_endian(sync.data + 4, seconds);
  if (::write(fd_, &sync, sizeof(sync)) != sizeof(sync)) return error("SYNC not written", errno);
  const int64_t written_ns = now_ns();

  // The SYNC back from the kernel, with its time. Other hosts' time frames
  // and this one's earlier ones are passed over.
  int64_t time_ns = 0;
  const int64_t deadline_ns = before_ns + echo_timeout_ms * 1000000ll;
  while (time_ns == 0) {
    const int64_t left_ms = (deadline_ns - now_ns()) / 1000000;
    pollfd poll_fd = {fd_, POLLIN, 0};
    const int ready = left_ms <= 0 ? 0 : ::poll(&poll_fd, 1, left_ms);
    if (ready < 0) return error("interrupted");
    if (ready == 0) return error("the SYNC did not come back from the interface");
    can_frame frame;
    char control[256];
    iovec iov = {&frame, sizeof(frame)};
    msghdr message = {};
    message.msg_iov = &iov;
    message.msg_iovlen = 1;
    message.msg_control = control;
    message.msg_controllen = sizeof(control);
    const ssize_t length = ::recvmsg(fd_, &message, 0);
    if (length < 0 && errno != EINTR && errno != EAGAIN) return error("interface", errno);
    if (length != sizeof(frame)) continue;
    if (!(message.msg_flags & MSG_CONFIRM) || std::memcmp(frame.data, sync.data, 8) != 0) continue;
    for (cmsghdr *c = CMSG_FIRSTHDR(&message); c != nullptr; c = CMSG_NXTHDR(&message, c)) {
      if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPING) {
        timespec stamps[3];
        std::memcpy(stamps, CMSG_DATA(c), sizeof(stamps));
        time_ns = to_ns(stamps[0]);  // the kernel's, in CLOCK_REALTIME
      }
    }
    // The kernel starts to keep the times of frames a moment after the first
    // socket asks for them, so the first frames may come back without one
    if (time_ns == 0) time_ns = written_ns;
  }

  // seconds + overflow + nanoseconds is the time; the overflow has two bits
  const int64_t since = time_ns - seconds * second_ns;
  if (since < 0 || since >= 4 * second_ns) return error("the host's time was set meanwhile");
  can_frame follow_up = {};
  follow_up.can_id = time_id;
  follow_up.can_dlc = 8;
  follow_up.data[0] = follow_up_type;
  follow_up.data[2] = domain_ << 4 | sequence;
  follow_up.data[3] = static_cast<uint8_t>(since / second_ns);
  big_endian(follow_up.data + 4, static_cast<uint32_t>(since % second_ns));
  if (::write(fd_, &follow_up, sizeof(follow_up)) != sizeof(follow_up)) {
    return error("follow up not written", errno);
  }
  result.time_ns = time_ns;
  result.delay_ns = time_ns - before_ns;
  return result;
}

int can_time_sync(const std::vector<std::string> &interfaces, double period_s,
                  uint8_t domain, bool verbose, std::atomic<bool> &signal_exit) {
  const bool any = std::find(interfaces.begin(), interfaces.end(), "any") != interfaces.end();
  struct Sender {
    std::unique_ptr<CANTimeSync> sync;
    std::string error;  // of the last time, which is said once
  };
  std::map<std::string, Sender> senders;
  // With "any" the interfaces are looked for again each period: one that
  // comes up joins, and one that is gone or down is left out
  const auto open = [&]() {
    const std::vector<std::string> names = any ? can_time_sync_interfaces() : interfaces;
    for (auto i = senders.begin(); i != senders.end();) {
      if (std::find(names.begin(), names.end(), i->first) == names.end()) {
        std::cout << i->first << ": no longer up" << std::endl;
        i = senders.erase(i);
      } else {
        ++i;
      }
    }
    for (const std::string &name : names) {
      if (senders.count(name)) continue;
      senders[name].sync = std::make_unique<CANTimeSync>(name, domain);
      std::cout << name << ": sending the time every " << period_s << " s" << std::endl;
    }
  };
  if (period_s < 0.001) period_s = 0.001;
  try {
    open();
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  if (senders.empty()) {
    std::cerr << "no CAN interface is up" << (any ? ": waiting for one" : "") << std::endl;
    if (!any) return 1;
  }

  const int64_t period_ns = static_cast<int64_t>(period_s * 1e9);
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  while (!signal_exit) {
    for (auto &[name, sender] : senders) {
      const CANTimeSync::Result result = sender.sync->send();
      if (signal_exit) break;
      // A failure is said once, and so is the end of it
      if (result.error != sender.error) {
        if (!result.error.empty()) {
          std::cerr << name << ": time not sent: " << result.error << std::endl;
        } else {
          std::cerr << name << ": time sent again" << std::endl;
        }
        sender.error = result.error;
      }
      if (verbose && result.error.empty()) {
        std::cout << name << " " << result.time_ns / second_ns << "." << std::setw(9)
                  << std::setfill('0') << result.time_ns % second_ns << std::setfill(' ')
                  << " after " << result.delay_ns / 1000 << " us" << std::endl;
      }
    }
    const int64_t at = to_ns(next) + period_ns;
    next.tv_sec = at / second_ns;
    next.tv_nsec = at % second_ns;
    // interrupted by the signal that sets signal_exit
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    if (any && !signal_exit) {
      try {
        open();
      } catch (const std::runtime_error &) {
        // an interface that went away between the look and the open: next time
      }
    }
  }
  return 0;
}

}  // namespace obot
