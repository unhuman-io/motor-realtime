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
const int keepalive_ms = 500;  // and otherwise, so that it knows of a host

} // namespace

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
  uint8_t checksum = 0;
  for (char c : payload)
    checksum += c;
  char checksum_str[3];
  snprintf(checksum_str, sizeof(checksum_str), "%02x", checksum);
  last_packet_ = "$" + payload + "#" + checksum_str;
  send_raw(last_packet_);
}

// A request to the node and its reply. A lost frame is tried again.
std::string GDBServer::target(const std::string &request) {
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

// M address,length:hex in as many requests as it takes
std::string GDBServer::memory_write(const std::string &packet) {
  const size_t comma = packet.find(','), colon = packet.find(':');
  if (comma == std::string::npos || colon == std::string::npos || colon < comma)
    return "E01";
  uint32_t address = std::stoul(packet.substr(1, comma - 1), nullptr, 16);
  const std::string hex = packet.substr(colon + 1);
  const size_t chunk = 24; // bytes, a multiple of 4: words stay whole
  if (hex.empty())
    return "OK";
  for (size_t offset = 0; offset < hex.size(); offset += 2 * chunk) {
    const std::string part = hex.substr(offset, 2 * chunk);
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

bool GDBServer::handle(const std::string &packet, std::string &reply) {
  reply.clear();
  if (packet.empty())
    return true;
  if (starts_with(packet, "qSupported")) {
    reply = "PacketSize=800;qXfer:features:read+";
  } else if (starts_with(packet, "qXfer:features:read:target.xml:")) {
    static const std::string xml = target_description();
    size_t offset = 0, length = 0;
    if (sscanf(packet.c_str() + packet.rfind(':') + 1, "%zx,%zx", &offset,
               &length) != 2) {
      reply = "E01";
    } else if (offset >= xml.size()) {
      reply = "l";
    } else {
      reply = (offset + length < xml.size() ? "m" : "l") +
              xml.substr(offset, length);
    }
  } else if (packet == "qAttached") {
    reply = "1";
  } else if (starts_with(packet, "qSymbol")) {
    reply = "OK";
  } else if (packet[0] == 'c' || packet[0] == 's') {
    // The stop reply comes when the node has stopped, see check_target
    target(packet.substr(0, 1));
    state_ = State::RUNNING;
    return false;
  } else if (packet[0] == 'D' || packet[0] == 'k' ||
             starts_with(packet, "vKill")) {
    if (packet[0] != 'k')
      reply = "OK";
    close_ = true; // the node is detached when the connection is closed
    return packet[0] != 'k';
  } else if (packet[0] == 'M') {
    reply = memory_write(packet);
  } else if (packet == "qRcmd,6c697665") {
    // monitor live: the node continues and gdb keeps its prompt, as when it
    // connected. gdb has no breakpoints in the node while it has the prompt.
    target("c");
    state_ = State::LIVE;
    const std::string text =
        "node continues: memory is live, the registers are a sample "
        "(maintenance flush register-cache for a new one)\n";
    static const char hex[] = "0123456789abcdef";
    for (unsigned char c : text) {
      reply += hex[c >> 4];
      reply += hex[c & 0xF];
    }
  } else if (packet[0] == '?') {
    reply = target("?");
    if (state_ == State::RUNNING)
      state_ = State::LIVE;
  } else if (std::string_view("mgpPHTZz").find(packet[0]) !=
                 std::string_view::npos ||
             packet == "qC" || packet == "qfThreadInfo" ||
             packet == "qsThreadInfo" ||
             starts_with(packet, "qThreadExtraInfo,") ||
             starts_with(packet, "qRcmd,")) {
    reply = packet.size() + 1 > max_request ? "E01" : target(packet);
  }
  // anything else is not supported: the empty reply
  return true;
}

// Has the node stopped while gdb waits for it, and a sign of life otherwise
void GDBServer::check_target() {
  const std::string status = target("y");
  if (state_ == State::RUNNING && !status.empty() && status[0] == 'T') {
    state_ = State::STOPPED;
    send_packet(status);
  }
}

void GDBServer::serve() {
  std::string in;
  char buf[4096];
  state_ = State::LIVE;
  close_ = false;
  // A node without the packets, its bootloader for one, answers nothing gdb
  // can use: gdb then waits for a stop that never comes, at full cpu load
  const std::string stop = target("?");
  if (stop.empty() || stop[0] != 'T') {
    std::cerr << "the node does not answer gdb's packets (reply to ?: '" << stop
              << "'): is it in its bootloader? Closing the connection."
              << std::endl;
    return;
  }
  auto last_check = std::chrono::steady_clock::now();
  while (!signal_exit_ && !close_) {
    const int period_ms =
        state_ == State::RUNNING ? running_poll_ms : keepalive_ms;
    if (wait_readable(connfd_,
                      state_ == State::RUNNING ? running_poll_ms : 50)) {
      ssize_t n = read(connfd_, buf, sizeof(buf));
      if (n <= 0)
        break;
      in.append(buf, n);
    }
    while (!in.empty() && !close_) {
      if (in[0] == '\x03') {
        in.erase(0, 1);
        if (verbose_)
          std::cout << "from gdb: interrupt" << std::endl;
        target("b");
        if (state_ != State::RUNNING)
          state_ = State::RUNNING;
      } else if (in[0] == '-') {
        in.erase(0, 1);
        send_raw(last_packet_);
      } else if (in[0] == '$') {
        const size_t end = in.find('#');
        if (end == std::string::npos || in.size() < end + 3)
          break; // the rest is yet to come
        const std::string packet = in.substr(1, end - 1);
        uint8_t checksum = 0;
        for (char c : packet)
          checksum += c;
        const bool ok =
            checksum == std::stoul(in.substr(end + 1, 2), nullptr, 16);
        in.erase(0, end + 3);
        if (verbose_)
          std::cout << "from gdb: " << packet << std::endl;
        send_raw(ok ? "+" : "-");
        std::string reply;
        if (ok && handle(packet, reply))
          send_packet(reply);
        if (ok && state_ == State::RUNNING) {
          check_target(); // a step is done by now
          last_check = std::chrono::steady_clock::now();
        }
      } else {
        in.erase(0, 1); // + and what is not a packet
      }
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_check >= std::chrono::milliseconds(period_ms)) {
      last_check = now;
      check_target();
    }
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
