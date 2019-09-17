#include "motor_manager.h"
#include "motor.h"
#include <iostream>
#include <iomanip>
#include "rt_version.h"
#include "CLI11.hpp"

int main(int argc, char** argv) {
    CLI::App app{"Utility for communicating with motor drivers"};
    bool print = false, list = true, version = false;
    Command command = {};
    auto set = app.add_subcommand("set", "Send data to motor(s)");
    set->add_option("--host_time", command.host_timestamp, "Host time");
    set->add_option("--mode", command.mode_desired, "Mode desired");
    set->add_option("--current", command.current_desired, "Current desired");
    set->add_option("--position", command.position_desired, "Position desired");
    app.add_flag("-p,--print", print, "Print data received from motor(s)");
    app.add_flag("-l,--list", list, "List connected motors");
    app.add_flag("-v,--version", version, "Print version information");
    CLI11_PARSE(app, argc, argv);

    MotorManager m;
    auto motors = m.get_connected_motors();

    if (version) {
        std::cout << "motor_util version: " << RT_VERSION_STRING << std::endl;
    }

    if (list) {
        int name_width = 10;
        int serial_number_width = 15;
        int version_width = 60;
        int path_width = 15;
        int dev_path_width = 12;
        std::cout << motors.size() << " connected motor" << (motors.size() == 1 ? "" : "s") << std::endl;
        if (motors.size() > 0) {
            std::cout << std::setw(dev_path_width) << "Dev" << std::setw(name_width) << "Name"
                        << std::setw(serial_number_width) << " Serial number"
                        << std::setw(version_width) << "Version" << std::setw(path_width) << "Path" << std::endl;
            std::cout << std::setw(dev_path_width + name_width + serial_number_width + version_width + path_width) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
            for (auto m : motors) {
                std::cout << std::setw(dev_path_width) << m->dev_path()
                        << std::setw(name_width) << m->name()
                        << std::setw(serial_number_width) << m->serial_number()
                        << std::setw(version_width) << m->version()
                        << std::setw(path_width) << m->base_path() << std::endl;
            }
        }
    }

    if (*set && motors.size()) {
        auto commands = std::vector<Command>(motors.size(), command);
        std::cout << "Writing commands: \n" << commands << std::endl;
        m.write(commands);
    }

    if (print && motors.size()) {
        m.open();
        while (1) {
            m.poll();
            auto status = m.read();
            std::cout << status << std::endl;
        }
        m.close();
    }

    return 0;
}