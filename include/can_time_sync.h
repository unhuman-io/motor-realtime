#pragma once
#include <atomic>
#include <cstdint>
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
// When the SYNC was sent is the time that the kernel gives for the frame as
// this socket gets it back. With a driver that only hands sent frames to the
// kernel, the PEAK pcan driver for one, that is when the frame went to the
// driver: some tens of microseconds before it is on the bus, and more when
// the bus is busy. A driver that returns frames once they are sent gives the
// time of that. A frame that comes back without a time, as the first ones
// after the socket is opened may, gets the time right after it was written.
//
//   motor_util time_sync --interface can0 --period 1
class CANTimeSync {
    public:
      explicit CANTimeSync(const std::string &interface, uint8_t domain = 0);
      ~CANTimeSync();
      CANTimeSync(const CANTimeSync &) = delete;
      CANTimeSync &operator=(const CANTimeSync &) = delete;

      struct Result {
        std::string error;        // empty: both frames are sent
        int64_t time_ns = 0;      // of the SYNC, since the epoch
        int64_t delay_ns = 0;     // from before the SYNC was written to that time
      };
      // One SYNC and its follow up
      Result send();
      const std::string &interface() const { return interface_; }

    private:
      std::string interface_;
      uint8_t domain_;
      uint8_t sequence_ = 0;
      int fd_ = -1;
};

// The CAN interfaces that are up
std::vector<std::string> can_time_sync_interfaces();

// Sends the time on each interface every period_s seconds until signal_exit.
// "any" as an interface is every CAN interface that is up, also those that
// come up later. Returns the process exit code.
int can_time_sync(const std::vector<std::string> &interfaces, double period_s,
                  uint8_t domain, bool verbose, std::atomic<bool> &signal_exit);

// What time_sync sends, for its help
extern const char *const can_time_sync_help;

}  // namespace obot
