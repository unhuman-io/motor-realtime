#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace obot {

// This host's time to the nodes of a CAN interface, in the frames of
// AUTOSAR's time synchronization over CAN (CanTSyn, SWS_CanTSyn_00015 and
// 00016: the SYNC and FUP messages that are not CRC secured): a SYNC with the
// seconds, then a follow up that says when exactly the SYNC was sent. A node
// notes when the SYNC arrived on its own clock, and with the follow up knows
// the host's time for that moment (obot3 control/clock.cppm).
// Both are classic frames of 8 bytes to all nodes, id 0x07F:
//   SYNC        0x10, 0, domain << 4 | sequence, 0, seconds (big endian)
//   follow up   0x18, 0, domain << 4 | sequence, overflow of seconds in bits 1..0, nanoseconds (big endian)
// The time of the SYNC is seconds + overflow + nanoseconds, of CLOCK_REALTIME.
//
// When the SYNC was sent is told by the frame as this socket gets it back:
//   - by the interface's hardware time for it, where the driver gives one
//     with the frames it returns once they are sent. That is in the
//     interface's own clock, which CANDeviceClock relates to this host's.
//   - else by the kernel's software time for it. With a driver that only
//     hands sent frames to the kernel, the PEAK pcan driver for one, that is
//     when the frame went to the driver: some tens of microseconds before it
//     is on the bus, and more when the bus is busy.
// A frame that comes back without any time, as the first ones after the
// socket is opened may, gets the time right after it was written.
//
//   motor_util time_sync --interface can0 --period 1

// The clock of a CAN interface against this host's, from frames that have
// both the interface's time and the kernel's: the kernel's is later by what
// the interrupt took, at least, so the frames that took least tell how far
// the two clocks are apart, and two such frames a while apart how fast they
// run apart. An interface whose times already are this host's, as some
// drivers make them, is taken as it is.
class CANDeviceClock {
    public:
      // A frame's time by the interface and by the kernel, in ns. Returns this
      // host's time for the interface's.
      int64_t host_time(int64_t device_ns, int64_t kernel_ns);

    private:
      struct Sample {           // the frame of a second that took least
        int64_t device_ns;
        int64_t difference_ns;  // the kernel's time minus the interface's
      };
      int64_t difference_at(int64_t device_ns) const;
      std::deque<Sample> samples_;
};

class CANTimeSync {
    public:
      // software: only the kernel's time for a sent frame, also where the
      // interface has one
      explicit CANTimeSync(const std::string &interface, uint8_t domain = 0,
                           bool software = false);
      ~CANTimeSync();
      CANTimeSync(const CANTimeSync &) = delete;
      CANTimeSync &operator=(const CANTimeSync &) = delete;

      struct Result {
        std::string error;        // empty: both frames are sent
        int64_t time_ns = 0;      // of the SYNC, since the epoch
        int64_t delay_ns = 0;     // from before the SYNC was written to that time
        bool hardware = false;    // the time is the interface's, not the kernel's
      };
      // One SYNC and its follow up
      Result send();
      const std::string &interface() const { return interface_; }

    private:
      std::string interface_;
      uint8_t domain_;
      bool software_;
      CANDeviceClock device_clock_;
      uint8_t sequence_ = 0;
      int fd_ = -1;
};

// The CAN interfaces that are up
std::vector<std::string> can_time_sync_interfaces();

// Sends the time on each interface every period_s seconds until signal_exit.
// "any" as an interface is every CAN interface that is up, also those that
// come up later. Returns the process exit code.
int can_time_sync(const std::vector<std::string> &interfaces, double period_s,
                  uint8_t domain, bool verbose, std::atomic<bool> &signal_exit,
                  bool software = false);

// What time_sync sends, for its help
extern const char *const can_time_sync_help;

}  // namespace obot
