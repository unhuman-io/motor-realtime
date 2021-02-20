file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v1.9.1/CLI11.hpp" 3rdparty/CLI11.hpp
    EXPECTED_MD5 05de518a9321e3b6393fe574ac72c6e8)
file(DOWNLOAD "https://raw.githubusercontent.com/CLIUtils/CLI11/master/LICENSE" 3rdparty/LICENSE/CLI11
    EXPECTED_MD5 b73927b18d5c6cd8d2ed28a6ad539733)
add_library(cli11 INTERFACE)
target_include_directories(cli11 INTERFACE ${CMAKE_BINARY_DIR}/3rdparty/)

if(BUILD_PYTHON_API)
    add_subdirectory(pybind11)
endif()
