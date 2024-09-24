if(BUILD_MOTOR_UTIL)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v2.3.2/CLI11.hpp" 3rdparty/CLI11.hpp
        EXPECTED_MD5 b67d76d9df58edcd8b301cad0c0e323b)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/raw/v2.3.2/LICENSE" 3rdparty/LICENSE/LICENSE-CLI11
        EXPECTED_MD5 afb20fe5f67e21074b10b864d9563c11)
    add_library(cli11 INTERFACE)
    install(FILES ${CMAKE_BINARY_DIR}/3rdparty/LICENSE/LICENSE-CLI11 DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE)
    target_include_directories(cli11 SYSTEM INTERFACE ${CMAKE_BINARY_DIR}/3rdparty/)

    file(DOWNLOAD "https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp" 3rdparty/json.hpp
        EXPECTED_MD5 30b66ababceb852ec47ecf93fd370b8d)
    file(DOWNLOAD https://github.com/nlohmann/json/raw/v3.11.3/LICENSE.MIT 3rdparty/LICENSE/LICENSE-json
        EXPECTED_MD5 f969127d7b7ed0a8a63c2bbeae002588)
    add_library(json INTERFACE)
    install(FILES ${CMAKE_BINARY_DIR}/3rdparty/LICENSE/LICENSE-json DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE)
    target_include_directories(json SYSTEM INTERFACE ${CMAKE_BINARY_DIR}/3rdparty/)
endif()

if(BUILD_PYTHON_API)
    add_subdirectory(pybind11)
    install(FILES pybind11/LICENSE DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE
        RENAME LICENSE-pybind11)
endif()

add_library(obot-protocol obot-protocol/protocol_parser.cpp)
target_include_directories(obot-protocol PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/obot-protocol)
if(BUILD_SHARED_LIBS)
    install(TARGETS obot-protocol
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

