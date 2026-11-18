#include "can_time_sync.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

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
    "When the SYNC was sent is the interface's hardware time for the frame, where "
    "its driver gives one for sent frames. Else it falls back to software, and says "
    "so: the kernel's time for the frame, which with most drivers is when it went to "
    "the driver, not when it was on the bus.";

namespace {
const canid_t time_id = 0 << 7 | 0x7F;  // the time message, to all nodes
const uint8_t sync_type = 0x10, follow_up_type = 0x18;
// CANDeviceClock
const size_t max_samples = 48;              // seconds that it looks back
const double max_rate = 500e-6;             // more than two crystals differ by
const int64_t jump_ns = 5000000;            // off by more: a clock was set
const int64_t host_clock_ns = 100000000;    // this close to the kernel's time is no clock of its own
const int echo_timeout_ms = 100;
const int arphrd_can = 280;  // ARPHRD_CAN, which the headers of older C libraries do not have
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
    if (type == arphrd_can && (flags & IFF_UP)) interfaces.push_back(i->if_name);
  }
  if_freenameindex(names);
  std::sort(interfaces.begin(), interfaces.end());
  return interfaces;
}

int64_t CANDeviceClock::difference_at(int64_t device_ns) const {
  // How fast the two run apart. The frames that took least are the ones along
  // the lower edge of all of them, the difference over the time: the rate is
  // the slope of that edge at the middle of the time they span, which is the
  // line below all frames that is closest to them. (The lower convex hull,
  // and its side over the middle.)
  double rate = 0;
  if (samples_.size() >= 2) {
    std::vector<Sample> hull;
    for (const Sample &sample : samples_) {
      while (hull.size() >= 2) {
        const Sample &a = hull[hull.size() - 2], &b = hull[hull.size() - 1];
        const double turn =
            static_cast<double>(b.device_ns - a.device_ns) * static_cast<double>(sample.difference_ns - a.difference_ns) -
            static_cast<double>(b.difference_ns - a.difference_ns) * static_cast<double>(sample.device_ns - a.device_ns);
        if (turn > 0) break;  // b is below the line from a to this one: it stays
        hull.pop_back();
      }
      hull.push_back(sample);
    }
    const int64_t middle = samples_.front().device_ns / 2 + samples_.back().device_ns / 2;
    size_t side = 0;
    while (side + 2 < hull.size() && hull[side + 1].device_ns < middle) side++;
    rate = static_cast<double>(hull[side + 1].difference_ns - hull[side].difference_ns) /
           static_cast<double>(hull[side + 1].device_ns - hull[side].device_ns);
    rate = std::clamp(rate, -max_rate, max_rate);
  }
  // and how far apart they are now: that line, which is the least of the
  // frames, each brought to now at that rate
  int64_t difference = INT64_MAX;
  for (const Sample &sample : samples_) {
    difference = std::min(difference, sample.difference_ns +
        static_cast<int64_t>(rate * static_cast<double>(device_ns - sample.device_ns)));
  }
  return difference;
}

int64_t CANDeviceClock::host_time(int64_t device_ns, int64_t kernel_ns) {
  const int64_t difference = kernel_ns - device_ns;
  // the driver has made it this host's time already
  if (difference > -host_clock_ns && difference < host_clock_ns) return device_ns;
  // one of the clocks was set: from the start
  if (!samples_.empty() && std::llabs(difference - difference_at(device_ns)) > jump_ns) {
    samples_.clear();
  }
  // Of each second on the interface's clock the frame that took least is kept
  if (!samples_.empty() && samples_.back().device_ns / second_ns == device_ns / second_ns) {
    if (difference < samples_.back().difference_ns) samples_.back() = {device_ns, difference};
  } else {
    samples_.push_back({device_ns, difference});
    if (samples_.size() > max_samples) samples_.pop_front();
  }
  return device_ns + difference_at(device_ns);
}

CANTimeSync::CANTimeSync(const std::string &interface, uint8_t domain, bool software)
    : interface_(interface), domain_(domain & 0xF), software_(software) {
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
  // the kernel's time of a frame, and the interface's where it has one
  const int timestamps = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE |
                         SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
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
        const int64_t device_ns = to_ns(stamps[2]);  // the interface's, in its own clock
        if (!software_ && device_ns != 0 && time_ns != 0) {
          time_ns = device_clock_.host_time(device_ns, time_ns);
          result.hardware = true;
        }
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
                  uint8_t domain, bool verbose, std::atomic<bool> &signal_exit,
                  bool software) {
  const bool any = std::find(interfaces.begin(), interfaces.end(), "any") != interfaces.end();
  struct Sender {
    std::unique_ptr<CANTimeSync> sync;
    std::string error;  // of the last time, which is said once
    int hardware = -1;  // whether the last time was the interface's: also said once
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
      senders[name].sync = std::make_unique<CANTimeSync>(name, domain, software);
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
      if (result.error.empty() && result.hardware != sender.hardware && !software) {
        if (result.hardware) {
          std::cout << name << ": hardware timestamps for the times sent" << std::endl;
        } else {
          std::cout << name << ": no hardware timestamp for a sent frame from this "
                    << "interface: falling back to software, the kernel's time for it"
                    << std::endl;
        }
        sender.hardware = result.hardware;
      }
      if (verbose && result.error.empty()) {
        std::cout << name << " " << result.time_ns / second_ns << "." << std::setw(9)
                  << std::setfill('0') << result.time_ns % second_ns << std::setfill(' ')
                  << " after " << result.delay_ns / 1000 << " us"
                  << (result.hardware ? " hardware" : " software") << std::endl;
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
