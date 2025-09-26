#include "terminal.h"
#include <unistd.h>
#include <stdio.h>
namespace obot {

std::string_view ANSI_RESET = isatty(fileno(stdout)) ? "\033[0m" : "";
std::string_view ANSI_BOLD = isatty(fileno(stdout)) ? "\033[1m" : "";
std::string_view ANSI_RED = isatty(fileno(stdout)) ? "\033[31m" : "";
std::string_view ANSI_GREEN = isatty(fileno(stdout)) ? "\033[32m" : "";
std::string_view ANSI_YELLOW = isatty(fileno(stdout)) ? "\033[33m" : "";
std::string_view ANSI_BLUE = isatty(fileno(stdout)) ? "\033[34m" : "";

};
