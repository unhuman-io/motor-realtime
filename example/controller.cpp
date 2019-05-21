#include "controller.h"

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

void Controller::update(std::vector<Status> statuses, std::vector<Command> commands) {
    for (int i; i<statuses.size(); i++) {
        commands[i].mode = mode_desired_[i];
        commands[i].current_desired = current_desired_[i];
        commands[i].position_desired = position_controllers_[i].update(position_desired_[i], 
                                        statuses[i].position_measured, statuses[i].count);
    }
}

double PositionController::update(double position_desired, double position_measured, int32_t count) {

    double kp = 1;
    double kd = 0.01;

    double velocity_measured = (position_measured - position_last_)/(count - last_count_);
    position_last_ = position_measured;
    last_count_ = count;

    double current_desired = kp*(position_desired - position_measured) - kd*velocity_measured;
    return current_desired;
}