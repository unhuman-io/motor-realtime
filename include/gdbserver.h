#pragma once
#include <atomic>
#include <functional>
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
      bool wait_readable(int fd, int timeout_ms);
      void serve();
      // The reply for gdb, false for none
      bool handle(const std::string &packet, std::string &reply);
      std::string target(const std::string &request);
      std::string memory_write(const std::string &packet);
      void send_raw(const std::string &data);
      void send_packet(const std::string &payload);
      void check_target();

      std::function<std::string(std::string_view)> writeread_;
      std::atomic<bool> &signal_exit_;
      bool verbose_;
      int sockfd_ = -1;
      int connfd_ = -1;
      State state_ = State::LIVE;
      bool close_ = false;
      std::string last_packet_;
};

}
