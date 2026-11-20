#include "can_time_sync.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "motor_can.h"

namespace obot {

namespace {
const canid_t time_id = 0 << 7 | 0x7F;  // the time message, to all nodes
const std::chrono::milliseconds echo_timeout{100};
const std::chrono::milliseconds host_clock{100};    // an interface's time this close to the kernel's is this host's time

CANTime to_time(const timespec &t) {
    return CANTime(std::chrono::seconds(t.tv_sec) + std::chrono::nanoseconds(t.tv_nsec));
}
}  // namespace

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
            timespec stamps[3];     // the kernel's, one that is no longer filled in, the interface's
            std::memcpy(stamps, CMSG_DATA(c), sizeof(stamps));
            const CANTime kernel = to_time(stamps[0]), device = to_time(stamps[2]);
            if (kernel != CANTime()) sent = kernel;
            hardware_ = device > sent - host_clock && device < sent + host_clock;
            if (hardware_) sent = device;
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
