#include "motor_manager.h"
#include "motor.h"
#include <iostream>
#include <iomanip>
#include "rt_version.h"

int main() {
    MotorManager m;
    auto motors = m.get_connected_motors();
    int name_width = 10;
    int serial_number_width = 15;
    int version_width = 60;
    int path_width = 15;
    int dev_path_width = 12;
    std::cout << "motor_util version: " << RT_VERSION_STRING << std::endl;
    std::cout << motors.size() << " connected motor" << (motors.size() == 1 ? "" : "s") << std::endl;
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