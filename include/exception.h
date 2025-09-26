#pragma once
#include <stdexcept>
#include "terminal.h"
namespace obot {

class RuntimeException : public std::runtime_error {
 public:
    explicit RuntimeException(const std::string& message)
        : std::runtime_error(std::string(ANSI_RED_STDERR) + message + std::string(ANSI_RESET_STDERR)) {}
};

};
