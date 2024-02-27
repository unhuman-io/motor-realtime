if(BUILD_MOTOR_UTIL)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v2.3.2/CLI11.hpp" 3rdparty/CLI11.hpp
        EXPECTED_MD5 b67d76d9df58edcd8b301cad0c0e323b)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/raw/v2.3.2/LICENSE" 3rdparty/LICENSE/LICENSE-CLI11
        EXPECTED_MD5 afb20fe5f67e21074b10b864d9563c11)
    add_library(cli11 INTERFACE)
    target_include_directories(cli11 SYSTEM INTERFACE ${CMAKE_BINARY_DIR}/3rdparty/)
    install(FILES ${CMAKE_BINARY_DIR}/3rdparty/LICENSE/LICENSE-CLI11 DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE)
endif()

if(BUILD_PYTHON_API)
    add_subdirectory(pybind11)
    install(FILES pybind11/LICENSE DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE
        RENAME LICENSE-pybind11)
endif()

add_library(obot-protocol obot-protocol/protocol_parser.cpp)
target_include_directories(obot-protocol PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/obot-protocol)