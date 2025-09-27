#pragma once
#include <stdexcept>
#include <source_location>
#include "terminal.h"
namespace obot {

class RuntimeException : public std::runtime_error {
 public:
    explicit RuntimeException(const std::string& message,
                std::source_location location = std::source_location::current())
        : std::runtime_error(std::string(ANSI_RED_STDERR) + message + std::string(ANSI_RESET_STDERR)),
          location_(location) {}
    std::source_location location() const { return location_;}
 private:
    std::source_location location_;
};

};
