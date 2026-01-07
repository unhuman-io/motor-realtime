#pragma once
#include <string>

namespace obot {

class L2Socket {
  public:
    L2Socket(std::string interface) : interface_(interface) { open(); }
    void open();
    void send(const char *data, std::size_t length);
  private:
    int fd_;
    std::string interface_;
};

} // namespace obot