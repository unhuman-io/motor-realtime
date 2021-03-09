#include "rt_version.h"
#include "CLI11.hpp"
#include "motor_manager.h"

int main(int argc, char** argv) {
    CLI::App app{"gRPC network interface to motors"};
    CLI11_PARSE(app, argc, argv);

    MotorManager m;

    std::cout << m.get_connected_motors()[0]->name() << std::endl;


    
    return 0;
}