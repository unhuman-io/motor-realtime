#pragma once
#include <stdexcept>
#if __cplusplus >= 202002L
#include <source_location>
#endif
#include "terminal.h"
#include <string.h>
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
        return " location(): " + std::string(location().file_name()) + "(" + std::to_string(location().line()) + ")\n" +
               " function(): " + std::string(location().function_name());
    }
 private:
    std::source_location location_;
};

class RuntimeErrnoException : public RuntimeException {
 public:
    explicit RuntimeErrnoException(const std::string& message,
                std::source_location location = std::source_location::current()) :
        RuntimeException(message + ": " + std::to_string(errno) + ": " + strerror(errno), location) {}
};

#else // c++ <20
class RuntimeException : public std::runtime_error {
 public:
    explicit RuntimeException(const std::string& message)
        : std::runtime_error(std::string(ANSI_RED_STDERR) + message + std::string(ANSI_RESET_STDERR)) {}
    std::string location_print() const { return ""; }
};

class RuntimeErrnoException : public RuntimeException {
 public:
    explicit RuntimeErrnoException(const std::string& message) :
        RuntimeException(message + ": " + std::to_string(errno) + ": " + strerror(errno)) {}
};
#endif // c++ <20

};
