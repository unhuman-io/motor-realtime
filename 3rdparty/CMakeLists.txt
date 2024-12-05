if(BUILD_MOTOR_UTIL)
    add_library(cli11 INTERFACE)
    install(FILES ${PROJECT_SOURCE_DIR}/3rdparty/cli11/LICENSE/LICENSE-CLI11 DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE)
    target_include_directories(cli11 SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/3rdparty/cli11)

    add_library(json INTERFACE)
    install(FILES ${PROJECT_SOURCE_DIR}/3rdparty/json/LICENSE/LICENSE-json DESTINATION 
        ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/LICENSE)
    target_include_directories(json SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/3rdparty/json)
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

