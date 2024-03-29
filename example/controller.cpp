#include "controller.h"

using namespace obot;

void Controller::set_mode(Mode mode) {
    std::vector<Mode> modes(n_, mode);
    set_mode(modes);
}

void Controller::set_mode(std::vector<Mode> modes) {
    mode_desired_ = modes;
}

void Controller::set_position(double position) {
    std::vector<double> positions(n_, position);
    set_position(positions);
}

void Controller::set_position(std::vector<double> position) {
    position_desired_ = position;
}

void Controller::set_current(double current) {
    std::vector<double> currents(n_, current);
    set_current(currents);
}

void Controller::set_current(std::vector<double> current) {
    current_desired_ = current;
}

void Controller::update(std::vector<Status> statuses, std::vector<Command> &commands) {
    for (int i=0; i<statuses.size(); i++) {
        switch (mode_desired_[i]) {
            case OPEN:
            default:
                commands[i].mode_desired = 0;
                break;
            case BRAKE:
                commands[i].mode_desired = 1;
                break;
            case CURRENT:
                commands[i].mode_desired = 2;
                commands[i].current_desired = current_desired_[i];
                break;
            case POSITION:
                commands[i].mode_desired = 2;
                float current_desired = current_desired_[i] + position_controllers_[i].update(position_desired_[i], 
                                    statuses[i].motor_position, statuses[i].mcu_timestamp);
                commands[i].current_desired = current_desired;
                break;
        }
    }
}

double PositionController::update(double position_desired, double position_measured, int32_t count) {
    double dt = (count - last_count_)/180e6;
    double velocity_measured = (position_measured - position_last_)/dt;
    position_last_ = position_measured;
    last_count_ = count;

    double current_desired = kp*(position_desired - position_measured) - kd*velocity_measured;
    return current_desired;
}
