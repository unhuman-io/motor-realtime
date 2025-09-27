#pragma once
#include <stdexcept>
#if __cplusplus >= 202002L
#include <source_location>
#include <format>
#endif
#include "terminal.h"
namespace obot {

#if __cplusplus >= 202002L
class RuntimeException : public std::runtime_error {
 public:
    explicit RuntimeException(const std::string& message,
                std::source_location location = std::source_location::current())
        : std::runtime_error(std::string(ANSI_RED_STDERR) + message + std::string(ANSI_RESET_STDERR)),
          location_(location) {}
    std::source_location location() const { return location_;}
    std::string location_print() const {
        return std::format(" location(): {}({})\n", location().file_name(), location().line()) +
               std::format(" function(): {}", location().function_name());
    }
 private:
    std::source_location location_;
};
#else
class RuntimeException : public std::runtime_error {
 public:
    explicit RuntimeException(const std::string& message)
        : std::runtime_error(std::string(ANSI_RED_STDERR) + message + std::string(ANSI_RESET_STDERR)) {}
    sstd::string location_print() const { return ""; }
};
#endif // c++ 20

};
