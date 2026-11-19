#pragma once
#include <atomic>
#include <deque>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace obot {

// A gdb remote protocol server for one node, over its text api: gdb's packets
// go to the node as text requests with a leading "$" and without the checksum,
// and the node's replies go back to gdb. The node does the work (obot3
// debug_monitor.h and text_api.h: memory, registers, threads, breakpoints,
// stop, step); what is here is the framing, the target description, the split
// of memory writes that don't fit a request, and the watch for the node
// stopping while gdb waits.
//
// The node is not stopped when gdb connects, though gdb is told so: memory is
// live and the registers are a sample, until gdb continues. `monitor live` in
// gdb gets back to that after a stop. When gdb leaves,
// the node's breakpoints are removed and it continues.
//
// gdb says which threads run with vCont, which the node takes as it is. In
// non-stop mode (set non-stop on, before target remote) the threads stop and
// run one by one, and the node tells of stops by notifications.
//
//   motor_util -f can0:1 --no-list gdbserver
//   (gdb) target remote :3293
class GDBServer {
    public:
      GDBServer(std::function<std::string(std::string_view)> writeread,
                std::atomic<bool> &signal_exit, int port = 3293,
                bool verbose = false);
      ~GDBServer();
      // Serves one gdb after the other until signal_exit
      void start();

    private:
      enum class State {
        LIVE,
        RUNNING,
        STOPPED
      }; // LIVE: running, gdb takes it as stopped
      // What gdb gets for a packet. None: not yet, it waits for the node to
      // stop and the stop reply is its answer (check_target).
      using Reply = std::optional<std::string>;
      struct Handler;
      static const Handler handlers_[];

      // A connection, from gdb's first byte to its last
      void serve();
      bool begin();
      void receive(std::string &in);
      void packet_from_gdb(const std::string &packet, bool intact);
      void interrupt();
      bool wait_readable(int fd, int timeout_ms);
      void send_raw(const std::string &data);
      void send_packet(const std::string &payload);
      void send_notification(const std::string &payload);

      // gdb's packets, see handlers_ for which is which
      Reply handle(const std::string &packet);
      Reply supported(const std::string &packet);
      Reply features(const std::string &packet);
      Reply attached(const std::string &packet);
      Reply symbols(const std::string &packet);
      Reply resume_actions(const std::string &packet);
      Reply set_non_stop(const std::string &packet);
      Reply next_stop(const std::string &packet);
      Reply resume_threads(const std::string &packet);
      Reply stop_reason(const std::string &packet);
      Reply resume(const std::string &packet);
      Reply detach(const std::string &packet);
      Reply kill(const std::string &packet);
      Reply memory_write(const std::string &packet);
      Reply monitor_live(const std::string &packet);
      Reply pass_on(const std::string &packet);

      // The node
      std::string target(const std::string &request);
      void check_target();
      void check_threads();
      uint32_t read_status(uint32_t &event_count);
      std::string take_stop();

      std::function<std::string(std::string_view)> writeread_;
      std::atomic<bool> &signal_exit_;
      bool verbose_;
      int sockfd_ = -1;
      int connfd_ = -1;
      State state_ = State::LIVE;
      bool close_ = false;
      std::string last_packet_;
      // gdb's non-stop mode: each thread stops and runs by itself
      bool non_stop_ = false;
      bool notified_ = false; // a stop notification is out, not acknowledged
      std::deque<std::string> stops_; // stop replies gdb has yet to get
      uint32_t known_stopped_ = 0; // threads gdb knows as stopped, a bit each
      uint32_t event_count_ = 0;
      std::string last_stop_;
      std::chrono::steady_clock::time_point last_check_; // of the node
      std::chrono::steady_clock::time_point last_request_; // of gdb
};

}
