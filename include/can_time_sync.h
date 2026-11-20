#pragma once
#include <chrono>
#include <cstdint>
#include <string>

namespace obot {

// This host's time to the nodes of a CAN interface: motor_util time_sync.
// Two CAN FD frames to all nodes on id 0x07F, both this struct, little
// endian: a SYNC, then a follow up with the time at which that SYNC was sent.
// A node notes when the SYNC arrived on its own clock, and with the follow up
// of the same sequence has the host's time for that moment (obot-controller
// obot3/control/clock.cppm has the same struct).
struct TimeSyncFrame {
    uint8_t sequence;       // of the pair
    uint8_t follow_up;      // 0: the SYNC, 1: its follow up
    uint8_t reserved[2];
    uint32_t seconds;       // in the follow up: when the SYNC was sent, since 1970
    uint32_t nanoseconds;
};
static_assert(sizeof(TimeSyncFrame) == 12, "a CAN FD frame length");

using CANTime = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

class CANTimeSync {
 public:
    explicit CANTimeSync(const std::string &interface);
    ~CANTimeSync();
    CANTimeSync(const CANTimeSync &) = delete;
    CANTimeSync &operator=(const CANTimeSync &) = delete;

    // One SYNC and its follow up. Returns when the SYNC was sent, by the
    // frame as this socket gets it back: the interface's hardware time for
    // it, where the driver gives one that is this host's time. Else the
    // kernel's, which with most drivers is when the frame was through and
    // with some when it went to the driver. A hardware time on a clock of
    // the interface's own is not used: relating that clock to this host's is
    // left to the driver. Throws a std::runtime_error if a frame could not
    // be sent.
    CANTime send();
    bool hardware() const { return hardware_; }     // the last time was the interface's
    const std::string &interface() const { return interface_; }

 private:
    std::string interface_;
    int fd_;
    uint8_t sequence_ = 0;
    bool hardware_ = false;
};

}  // namespace obot
