#include "can_time_sync.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "motor_can.h"

namespace obot {

namespace {
const canid_t time_id = 0 << 7 | 0x7F;  // the time message, to all nodes
const std::chrono::milliseconds echo_timeout{100};
// CANDeviceClock
const size_t max_samples = 48;                  // seconds that it looks back
const double max_rate = 500e-6;                 // more than two crystals differ by
const std::chrono::milliseconds jump{5};        // off by more: a clock was set
const std::chrono::milliseconds host_clock{100};    // this close to the kernel's time is no clock of its own
const int64_t second_ns = 1000000000;

CANTime to_time(const timespec &t) {
    return CANTime(std::chrono::seconds(t.tv_sec) + std::chrono::nanoseconds(t.tv_nsec));
}
}  // namespace

int64_t CANDeviceClock::difference_at(int64_t device_ns) const {
    // How fast the two run apart. The frames that took least are the ones
    // along the lower edge of all of them, the difference over the time: the
    // rate is the slope of that edge at the middle of the time they span,
    // which is the line below all frames that is closest to them. (The lower
    // convex hull, and its side over the middle.)
    double rate = 0;
    if (samples_.size() >= 2) {
        std::vector<Sample> hull;
        for (const Sample &sample : samples_) {
            while (hull.size() >= 2) {
                const Sample &a = hull[hull.size() - 2], &b = hull[hull.size() - 1];
                const double turn =
                    static_cast<double>(b.device_ns - a.device_ns) * static_cast<double>(sample.difference_ns - a.difference_ns) -
                    static_cast<double>(b.difference_ns - a.difference_ns) * static_cast<double>(sample.device_ns - a.device_ns);
                if (turn > 0) break;    // b is below the line from a to this one: it stays
                hull.pop_back();
            }
            hull.push_back(sample);
        }
        const int64_t middle = samples_.front().device_ns / 2 + samples_.back().device_ns / 2;
        size_t side = 0;
        while (side + 2 < hull.size() && hull[side + 1].device_ns < middle) side++;
        rate = static_cast<double>(hull[side + 1].difference_ns - hull[side].difference_ns) /
               static_cast<double>(hull[side + 1].device_ns - hull[side].device_ns);
        rate = std::min(std::max(rate, -max_rate), max_rate);
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

CANTime CANDeviceClock::host_time(std::chrono::nanoseconds device, CANTime kernel) {
    const std::chrono::nanoseconds difference = kernel.time_since_epoch() - device;
    // the driver has made it this host's time already
    if (difference > -host_clock && difference < host_clock) return CANTime(device);
    const int64_t device_ns = device.count(), difference_ns = difference.count();
    // one of the clocks was set: from the start
    if (!samples_.empty() && std::llabs(difference_ns - difference_at(device_ns)) > std::chrono::nanoseconds(jump).count()) {
        samples_.clear();
    }
    // Of each second on the interface's clock the frame that took least is kept
    if (!samples_.empty() && samples_.back().device_ns / second_ns == device_ns / second_ns) {
        if (difference_ns < samples_.back().difference_ns) samples_.back() = {device_ns, difference_ns};
    } else {
        samples_.push_back({device_ns, difference_ns});
        if (samples_.size() > max_samples) samples_.pop_front();
    }
    return CANTime(device + std::chrono::nanoseconds(difference_at(device_ns)));
}

CANTimeSync::CANTimeSync(const std::string &interface)
    : interface_(interface), fd_(MotorCAN::open_socket(interface)) {
    // Of all frames only this socket's own time frames come back to it, with
    // the kernel's time for them and the interface's where it has one. A sent
    // frame comes back as a received one, so these are the receive times.
    const can_filter filter = {time_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
    const int on = 1;
    const int timestamps = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE |
                           SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0 ||
        ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0 ||
        ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPING, &timestamps, sizeof(timestamps)) < 0) {
        const std::string error = strerror(errno);
        ::close(fd_);
        throw std::runtime_error(interface_ + ": socket options: " + error);
    }
}

CANTimeSync::~CANTimeSync() { ::close(fd_); }

CANTime CANTimeSync::send() {
    const auto fail = [this](const std::string &what) {
        return std::runtime_error(interface_ + ": time not sent: " + what);
    };
    TimeSyncFrame time = {};
    time.sequence = sequence_++;
    canfd_frame frame = {};
    frame.can_id = time_id;
    frame.len = sizeof(time);
    frame.flags = CANFD_BRS;
    std::memcpy(frame.data, &time, sizeof(time));
    if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        throw fail(std::string("SYNC: ") + strerror(errno));
    }

    // The SYNC back from the kernel with its time. Earlier ones of this
    // socket and other hosts' are passed over.
    CANTime sent;
    const auto deadline = std::chrono::steady_clock::now() + echo_timeout;
    while (true) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        pollfd poll_fd = {fd_, POLLIN, 0};
        if (left <= 0 || ::poll(&poll_fd, 1, left) <= 0) throw fail("the SYNC did not come back");
        canfd_frame echo;
        char control[CMSG_SPACE(3 * sizeof(timespec))];
        iovec iov = {&echo, sizeof(echo)};
        msghdr message = {};
        message.msg_iov = &iov;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        if (::recvmsg(fd_, &message, 0) != sizeof(echo) || !(message.msg_flags & MSG_CONFIRM) ||
            std::memcmp(echo.data, frame.data, sizeof(time)) != 0) {
            continue;
        }
        // The kernel begins to keep the times of frames a moment after the
        // first socket asks, so the first frames may come back without one
        sent = std::chrono::system_clock::now();
        hardware_ = false;
        const cmsghdr *c = CMSG_FIRSTHDR(&message);
        if (c != nullptr && c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPING) {
            timespec stamps[3];     // the kernel's, unused, the interface's on its own clock
            std::memcpy(stamps, CMSG_DATA(c), sizeof(stamps));
            const CANTime kernel = to_time(stamps[0]), device = to_time(stamps[2]);
            if (kernel != CANTime()) sent = kernel;
            if (kernel != CANTime() && device != CANTime()) {
                sent = device_clock_.host_time(device.time_since_epoch(), kernel);
                hardware_ = true;
            }
        }
        break;
    }

    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(sent);
    time.follow_up = 1;
    time.seconds = static_cast<uint32_t>(seconds.time_since_epoch().count());
    time.nanoseconds = static_cast<uint32_t>((sent - seconds).count());
    std::memcpy(frame.data, &time, sizeof(time));
    if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        throw fail(std::string("follow up: ") + strerror(errno));
    }
    return sent;
}

}  // namespace obot
