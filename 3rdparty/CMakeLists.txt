if(BUILD_MOTOR_UTIL)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/releases/download/v1.9.1/CLI11.hpp" 3rdparty/CLI11.hpp
        EXPECTED_MD5 05de518a9321e3b6393fe574ac72c6e8)
    file(DOWNLOAD "https://github.com/CLIUtils/CLI11/raw/v1.9.1/LICENSE" 3rdparty/LICENSE/LICENSE-CLI11
        EXPECTED_MD5 b73927b18d5c6cd8d2ed28a6ad539733)
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
