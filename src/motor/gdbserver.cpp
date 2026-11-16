#include "gdbserver.h"
#include "exception.h"
#include <chrono>
#include <iostream>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <string.h>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace obot {

namespace {

// The registers as the node numbers them: the first 17 are its g packet, gdb
// asks for the others one at a time
std::string target_description() {
  std::string xml = "<?xml version=\"1.0\"?><!DOCTYPE target SYSTEM "
                    "\"gdb-target.dtd\"><target version=\"1.0\">"
                    "<architecture>arm</architecture><feature "
                    "name=\"org.gnu.gdb.arm.m-profile\">";
  for (int i = 0; i < 13; i++) {
    xml += "<reg name=\"r" + std::to_string(i) + "\" bitsize=\"32\" regnum=\"" +
           std::to_string(i) + "\"/>";
  }
  xml += "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\" regnum=\"13\"/>"
         "<reg name=\"lr\" bitsize=\"32\" regnum=\"14\"/>"
         "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\" regnum=\"15\"/>"
         "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"16\"/></feature>"
         "<feature name=\"org.gnu.gdb.arm.vfp\">";
  for (int i = 0; i < 16; i++) {
    xml += "<reg name=\"d" + std::to_string(i) +
           "\" bitsize=\"64\" type=\"ieee_double\" regnum=\"" +
           std::to_string(17 + i) + "\"/>";
  }
  xml += "<reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\" "
         "regnum=\"33\"/></feature></target>";
  return xml;
}

bool starts_with(const std::string &s, std::string_view prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}

const size_t max_request = 63; // characters of a text request, with its $
const int running_poll_ms = 5; // how often the node is asked if it stopped
const int non_stop_poll_ms = 10; // the same in non-stop mode, for any thread
const int keepalive_ms = 500;  // and otherwise, so that it knows of a host
const int idle_read_ms = 50;   // how long gdb is waited for at a time
const uint32_t max_threads = 8;

// The sum that ends a packet, after its #
uint8_t checksum(std::string_view payload) {
  uint8_t sum = 0;
  for (char c : payload)
    sum += c;
  return sum;
}

// A packet as it goes over the connection: $payload#checksum, or % for a
// notification
std::string framed(char start, const std::string &payload) {
  char sum[3];
  snprintf(sum, sizeof(sum), "%02x", checksum(payload));
  return start + payload + "#" + sum;
}

std::string hex(std::string_view text) {
  static const char digits[] = "0123456789abcdef";
  std::string out;
  for (unsigned char c : text) {
    out += digits[c >> 4];
    out += digits[c & 0xF];
  }
  return out;
}

// The stop reply of a thread that is stopped for no reason of its own
std::string stopped_thread(uint32_t thread) {
  return "T00thread:" + std::to_string(thread) + ";";
}

} // namespace

// What is done with each of gdb's packets: the first line that matches it. A
// packet that matches none is not supported, which the empty reply tells gdb.
struct GDBServer::Handler {
  std::string_view packet;
  enum { EXACT, PREFIX } match;
  Reply (GDBServer::*handle)(const std::string &packet);
};

const GDBServer::Handler GDBServer::handlers_[] = {
    // answered here
    {"qSupported", Handler::PREFIX, &GDBServer::supported},
    {"qXfer:features:read:target.xml:", Handler::PREFIX, &GDBServer::features},
    {"qAttached", Handler::EXACT, &GDBServer::attached},
    {"qSymbol", Handler::PREFIX, &GDBServer::symbols},
    {"vCont?", Handler::EXACT, &GDBServer::resume_actions},
    {"vStopped", Handler::EXACT, &GDBServer::next_stop},
    {"D", Handler::PREFIX, &GDBServer::detach},
    {"vKill", Handler::PREFIX, &GDBServer::detach},
    {"k", Handler::PREFIX, &GDBServer::kill},
    // the node's, with something to keep track of here
    {"QNonStop:", Handler::PREFIX, &GDBServer::set_non_stop},
    {"vCont;", Handler::PREFIX, &GDBServer::resume_threads},
    {"?", Handler::PREFIX, &GDBServer::stop_reason},
    {"c", Handler::PREFIX, &GDBServer::resume},
    {"s", Handler::PREFIX, &GDBServer::resume},
    {"M", Handler::PREFIX, &GDBServer::memory_write},
    {"qRcmd,6c697665", Handler::EXACT, &GDBServer::monitor_live}, // "live"
    // the node's, as they are
    {"m", Handler::PREFIX, &GDBServer::pass_on}, // read memory
    {"g", Handler::PREFIX, &GDBServer::pass_on}, // the registers
    {"p", Handler::PREFIX, &GDBServer::pass_on}, // one register
    {"P", Handler::PREFIX, &GDBServer::pass_on}, // write one
    {"H", Handler::PREFIX, &GDBServer::pass_on}, // select a thread
    {"T", Handler::PREFIX, &GDBServer::pass_on}, // is a thread there
    {"Z", Handler::PREFIX, &GDBServer::pass_on}, // a breakpoint or watchpoint
    {"z", Handler::PREFIX, &GDBServer::pass_on}, // and out again
    {"qC", Handler::EXACT, &GDBServer::pass_on}, // the current thread
    {"qfThreadInfo", Handler::EXACT, &GDBServer::pass_on},
    {"qsThreadInfo", Handler::EXACT, &GDBServer::pass_on},
    {"qThreadExtraInfo,", Handler::PREFIX, &GDBServer::pass_on},
    {"qRcmd,", Handler::PREFIX, &GDBServer::pass_on}, // a monitor command
};

GDBServer::Reply GDBServer::handle(const std::string &packet) {
  for (const Handler &handler : handlers_) {
    const bool matches = handler.match == Handler::EXACT
                             ? packet == handler.packet
                             : starts_with(packet, handler.packet);
    if (matches)
      return (this->*handler.handle)(packet);
  }
  return "";
}

// ---------------------------------------------------------------- the node

// A request to the node and its reply. A lost frame is tried again; a packet
// that does not fit a request is not sent.
std::string GDBServer::target(const std::string &request) {
  if (request.size() + 1 > max_request)
    return "E01";
  for (int attempt = 0; attempt < 3; attempt++) {
    try {
      std::string reply = writeread_("$" + request);
      while (!reply.empty() && (reply.back() == '\0' || reply.back() == ' '))
        reply.pop_back(); // padding of long replies
      if (verbose_)
        std::cout << "node: $" << request << " -> " << reply << std::endl;
      return reply;
    } catch (const std::exception &e) {
      if (verbose_)
        std::cout << "node: $" << request << " failed: " << e.what()
                  << std::endl;
    }
  }
  return "E01";
}

// The node's Y: the threads that are stopped, a bit each, the number of
// stops so far and the reply for the last of them
uint32_t GDBServer::read_status(uint32_t &event_count) {
  const std::string status = target("Y");
  const size_t first = status.find(';');
  const size_t second = status.find(';', first + 1);
  if (first == std::string::npos || second == std::string::npos)
    return 0;
  event_count = std::stoul(status.substr(0, first), nullptr, 16);
  last_stop_ = status.substr(second + 1);
  return std::stoul(status.substr(first + 1, second - first - 1), nullptr, 16);
}

// Has the node stopped while gdb waits for it, and a sign of life otherwise
void GDBServer::check_target() {
  last_check_ = std::chrono::steady_clock::now();
  if (non_stop_) {
    check_threads();
    return;
  }
  const std::string status = target("y");
  if (state_ == State::RUNNING && !status.empty() && status[0] == 'T') {
    state_ = State::STOPPED;
    send_packet(status);
  }
}

// Non-stop: gdb is told of each thread that has stopped since, the first by a
// notification and the others when it asks (next_stop)
void GDBServer::check_threads() {
  uint32_t event_count = event_count_;
  const uint32_t stopped = read_status(event_count);
  uint32_t event_thread = 0;
  const size_t at = last_stop_.find("thread:");
  if (at != std::string::npos)
    event_thread = std::stoul(last_stop_.substr(at + 7), nullptr, 16);
  for (uint32_t thread = 1; thread <= max_threads; thread++) {
    const uint32_t bit = 1u << (thread - 1);
    if (!(stopped & bit))
      continue;
    if (thread == event_thread && event_count != event_count_) {
      stops_.push_back(last_stop_);
    } else if (!(known_stopped_ & bit)) {
      stops_.push_back(stopped_thread(thread));
    }
  }
  event_count_ = event_count;
  known_stopped_ = stopped;
  if (!stops_.empty() && !notified_) {
    send_notification("Stop:" + take_stop());
    notified_ = true;
  }
}

std::string GDBServer::take_stop() {
  const std::string stop = stops_.front();
  stops_.pop_front();
  return stop;
}

// ------------------------------------------------------------ gdb's packets

GDBServer::Reply GDBServer::supported(const std::string &) {
  return "PacketSize=800;qXfer:features:read+;QNonStop+;vContSupported+";
}

// qXfer:features:read:target.xml:offset,length: that part of the description
GDBServer::Reply GDBServer::features(const std::string &packet) {
  static const std::string xml = target_description();
  size_t offset = 0, length = 0;
  if (sscanf(packet.c_str() + packet.rfind(':') + 1, "%zx,%zx", &offset,
             &length) != 2)
    return "E01";
  if (offset >= xml.size())
    return "l";
  return (offset + length < xml.size() ? "m" : "l") +
         xml.substr(offset, length);
}

// gdb attached to what was running, it did not start it
GDBServer::Reply GDBServer::attached(const std::string &) { return "1"; }

// no symbols wanted
GDBServer::Reply GDBServer::symbols(const std::string &) { return "OK"; }

GDBServer::Reply GDBServer::resume_actions(const std::string &) {
  return "vCont;c;C;s;S;t";
}

// QNonStop:1 or 0: gdb's mode, which the node needs to know too
GDBServer::Reply GDBServer::set_non_stop(const std::string &packet) {
  const std::string reply = target(packet);
  non_stop_ = reply == "OK" && packet.back() == '1';
  stops_.clear();
  notified_ = false;
  // what is stopped already is for gdb's ? to tell, not for a notification
  // while it is still connecting: a node can have a thread that is always
  // stopped
  known_stopped_ = read_status(event_count_);
  return reply;
}

// vStopped: the next thread that stopped, after the one of the notification
GDBServer::Reply GDBServer::next_stop(const std::string &) {
  if (stops_.empty()) {
    notified_ = false;
    return "OK";
  }
  return take_stop();
}

// vCont;action:thread;...: which threads run is the node's to work out
GDBServer::Reply GDBServer::resume_threads(const std::string &packet) {
  const std::string reply = target(packet);
  if (non_stop_) {
    // what stays stopped is not said again, gdb knows of it. What
    // has stopped by now is yet to be told.
    uint32_t count;
    known_stopped_ &= read_status(count);
    return reply;
  }
  if (reply != "OK")
    return reply;
  state_ = State::RUNNING;
  return std::nullopt;
}

// ?: why it is stopped. In non-stop mode every thread that is: the first
// here, the others by vStopped
GDBServer::Reply GDBServer::stop_reason(const std::string &) {
  if (!non_stop_) {
    if (state_ == State::RUNNING)
      state_ = State::LIVE;
    return target("?");
  }
  stops_.clear();
  known_stopped_ = read_status(event_count_);
  for (uint32_t thread = 1; thread <= max_threads; thread++) {
    if (known_stopped_ & (1u << (thread - 1)))
      stops_.push_back(stopped_thread(thread));
  }
  if (stops_.empty())
    return "OK";
  notified_ = true;
  return take_stop();
}

// c, s: continue and step, from a gdb that does not use vCont
GDBServer::Reply GDBServer::resume(const std::string &packet) {
  target(packet.substr(0, 1));
  state_ = State::RUNNING;
  return std::nullopt;
}

// D, vKill: the node is detached when the connection is closed
GDBServer::Reply GDBServer::detach(const std::string &) {
  close_ = true;
  return "OK";
}

// k: the same, and gdb does not wait for a reply
GDBServer::Reply GDBServer::kill(const std::string &) {
  close_ = true;
  return std::nullopt;
}

// M address,length:hex in as many requests as it takes
GDBServer::Reply GDBServer::memory_write(const std::string &packet) {
  const size_t comma = packet.find(','), colon = packet.find(':');
  if (comma == std::string::npos || colon == std::string::npos || colon < comma)
    return "E01";
  uint32_t address = std::stoul(packet.substr(1, comma - 1), nullptr, 16);
  const std::string data = packet.substr(colon + 1);
  const size_t chunk = 24; // bytes, a multiple of 4: words stay whole
  for (size_t offset = 0; offset < data.size(); offset += 2 * chunk) {
    const std::string part = data.substr(offset, 2 * chunk);
    char header[32];
    snprintf(header, sizeof(header),
             "M%x,%zx:", address + static_cast<uint32_t>(offset / 2),
             part.size() / 2);
    const std::string reply = target(header + part);
    if (reply != "OK")
      return reply.empty() ? "E01" : reply;
  }
  return "OK";
}

// monitor live: the node continues and gdb keeps its prompt, as when it
// connected. gdb has no breakpoints in the node while it has the prompt.
GDBServer::Reply GDBServer::monitor_live(const std::string &) {
  target("c");
  state_ = State::LIVE;
  return hex("node continues: memory is live, the registers are a sample "
             "(maintenance flush register-cache for a new one)\n");
}

GDBServer::Reply GDBServer::pass_on(const std::string &packet) {
  return target(packet);
}

// ----------------------------------------------------------- the connection

GDBServer::GDBServer(std::function<std::string(std::string_view)> writeread,
                     std::atomic<bool> &signal_exit, int port, bool verbose)
    : writeread_(writeread), signal_exit_(signal_exit), verbose_(verbose) {
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ == -1) {
    throw RuntimeException("socket creation failed");
  }
  int option = 1;
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
  struct sockaddr_in servaddr{};
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  servaddr.sin_port = htons(port);
  if (bind(sockfd_, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0) {
    throw RuntimeException("socket bind failed: " +
                           std::string(strerror(errno)));
  }
  if (listen(sockfd_, 1) != 0) {
    throw RuntimeException("listen failed");
  }
  std::cout << "gdbserver listening on port " << port << std::endl;
}

GDBServer::~GDBServer() {
  if (connfd_ >= 0)
    close(connfd_);
  if (sockfd_ >= 0)
    close(sockfd_);
}

bool GDBServer::wait_readable(int fd, int timeout_ms) {
  pollfd tmp{.fd = fd, .events = POLLIN, .revents = 0};
  int retval = poll(&tmp, 1, timeout_ms);
  if (retval < 0 && !signal_exit_ && errno != EINTR) {
    throw RuntimeErrnoException("poll error");
  }
  return retval > 0;
}

void GDBServer::send_raw(const std::string &data) {
  if (verbose_)
    std::cout << "to gdb: " << data << std::endl;
  size_t sent = 0;
  while (sent < data.size()) {
    ssize_t n =
        send(connfd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      close_ = true;
      return;
    }
    sent += n;
  }
}

void GDBServer::send_packet(const std::string &payload) {
  last_packet_ = framed('$', payload);
  send_raw(last_packet_);
}

// Not acknowledged by gdb, and not sent again
void GDBServer::send_notification(const std::string &payload) {
  send_raw(framed('%', payload));
}

// ctrl-c in gdb. In non-stop mode it stops threads with vCont instead.
void GDBServer::interrupt() {
  if (verbose_)
    std::cout << "from gdb: interrupt" << std::endl;
  if (!non_stop_) {
    target("b");
    state_ = State::RUNNING; // the stop reply comes when the node has stopped
  }
}

void GDBServer::packet_from_gdb(const std::string &packet, bool intact) {
  if (verbose_)
    std::cout << "from gdb: " << packet << std::endl;
  send_raw(intact ? "+" : "-");
  if (!intact)
    return;
  if (const Reply reply = handle(packet))
    send_packet(*reply);
  if (state_ == State::RUNNING || non_stop_)
    check_target(); // a step is done by now
}

// What gdb has sent so far: interrupts, requests for the last packet again,
// and packets. Part of a packet stays in for the rest to come.
void GDBServer::receive(std::string &in) {
  while (!in.empty() && !close_) {
    const char first = in[0];
    if (first != '$') {
      in.erase(0, 1);
      if (first == '\x03')
        interrupt();
      else if (first == '-')
        send_raw(last_packet_);
      // + and what is not a packet is dropped
      continue;
    }
    const size_t end = in.find('#');
    if (end == std::string::npos || in.size() < end + 3)
      return;
    const std::string packet = in.substr(1, end - 1);
    const bool intact =
        checksum(packet) == std::stoul(in.substr(end + 1, 2), nullptr, 16);
    in.erase(0, end + 3);
    packet_from_gdb(packet, intact);
  }
}

// A new gdb. False if the node is none for it: without the packets, its
// bootloader for one, it answers nothing gdb can use, and gdb then waits for
// a stop that never comes, at full cpu load.
bool GDBServer::begin() {
  state_ = State::LIVE;
  close_ = false;
  non_stop_ = false;
  stops_.clear();
  notified_ = false;
  target("QNonStop:0"); // whatever an earlier gdb left the node in
  const std::string stop = target("?");
  if (stop.empty() || stop[0] != 'T') {
    std::cerr << "the node does not answer gdb's packets (reply to ?: '" << stop
              << "'): is it in its bootloader? Closing the connection."
              << std::endl;
    return false;
  }
  return true;
}

void GDBServer::serve() {
  if (!begin())
    return;
  std::string in;
  char buf[4096];
  last_check_ = std::chrono::steady_clock::now();
  while (!signal_exit_ && !close_) {
    // gdb waits for a stop: the node is asked often. Otherwise only to keep
    // it knowing of a host.
    const bool waiting = state_ == State::RUNNING || non_stop_;
    const int check_period_ms = non_stop_ ? non_stop_poll_ms
                                : waiting ? running_poll_ms
                                          : keepalive_ms;
    if (wait_readable(connfd_, waiting ? running_poll_ms : idle_read_ms)) {
      ssize_t n = read(connfd_, buf, sizeof(buf));
      if (n <= 0)
        break;
      in.append(buf, n);
    }
    receive(in);
    if (std::chrono::steady_clock::now() - last_check_ >=
        std::chrono::milliseconds(check_period_ms))
      check_target();
  }
  target("D");
}

void GDBServer::start() {
  while (!signal_exit_) {
    if (!wait_readable(sockfd_, 100))
      continue;
    connfd_ = accept(sockfd_, nullptr, nullptr);
    if (connfd_ < 0) {
      throw RuntimeException("server accept failed");
    }
    int option = 1;
    setsockopt(connfd_, IPPROTO_TCP, TCP_NODELAY, &option, sizeof(option));
    std::cout << "gdb connected" << std::endl;
    serve();
    close(connfd_);
    connfd_ = -1;
    std::cout << "gdb disconnected, node continues" << std::endl;
  }
}

} // namespace obot
