file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v1.8.0/CLI11.hpp" 3rdparty/CLI11.hpp)
file(DOWNLOAD "https://github.com/CLIUtils/CLI11/blob/master/LICENSE" 3rdparty/LICENSE/CLI11)
add_library(cli11 INTERFACE)
target_include_directories(cli11 INTERFACE ${CMAKE_BINARY_DIR}/3rdparty/)