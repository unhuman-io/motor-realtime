#include "controller.h"

void Controller::update(std::vector<Status> statuses, std::vector<Command> commands) {
    double position_measured = statuses[0].res[0];

    double kp = 1;
    double kd = 0.01;
    double position_desired = 2;

    double velocity_measured = (position_measured - position_last_)/(statuses[0].count - last_count_);
    position_last_ = position_measured;
    last_count_ = statuses[0].count;

    double current_desired = kp*(position_desired - position_measured) - kd*velocity_measured;
    commands[0].current_desired = current_desired;
}