// A switch pro controller to differential drive two wheel motors
#include "motor_manager.h"

#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <iomanip>

int main (int argc, char **argv) {
    MotorManager motor_manager;
	auto motors = motor_manager.get_motors_by_name({"default"});
    motor_manager.open();
    std::cout << "Connected to " << motors.size() << " motors" << std::endl;

    int fid = open("/dev/hidraw6", O_RDONLY);

    struct Joystick {
        uint16_t buttons[2];
        uint16_t l_stick[2]; 
        uint16_t r_stick[2];
    } joystick;

    double current_gain = 3;

  //  pollfd fds = {fid, POLLIN}; optional
    motor_manager.set_command_mode(2);
    std::vector<float> current_desired = {0};
    uint32_t count = 0;
    while(1) {
        //poll(&fds, 1, 0);
        read(fid, &joystick, sizeof(joystick));
        // uint16_t ud = (joystick.l_stick[2] << 4l) | ((joystick.l_stick[1] & 0xF0) >> 4);
        // uint16_t lr = joystick.l_stick[0] | ((joystick.l_stick[1] & 0xF) << 8l);
        uint16_t ud = joystick.l_stick[1] >> 4;
        uint16_t lr = joystick.l_stick[0] >> 4;
        double ud2 = -1*((int16_t) ud - 2048)/1500.;
        double lr2 = ((int16_t) lr - 2048)/1500.;
        double enable = (joystick.buttons[0] & 0x200) ? 1.0 : 0.0;
        current_desired[0] = enable*current_gain*(ud2 + lr2);
        double j2 = current_gain*(ud2 - lr2);
        std::cout << "ud: " << std::setw(12) << ud2 << ", lr:" << lr2 << std::endl;
        motor_manager.set_command_count(count++);
        motor_manager.set_command_current(current_desired);
        motor_manager.write_saved_commands();
    }

    motor_manager.close();
    close(fid);
}