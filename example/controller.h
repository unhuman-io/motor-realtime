#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "motor.h"
#include <vector>

class PositionController {
 public:
    double update(double position_desired, double position_measured, int32_t count);
 private:
    int32_t last_count_;
    double position_last_;

    double kp = .1;
    double kd = 0.00001;
};

class Controller {
 public:
    Controller(int n) : n_(n), position_controllers_(n), 
         position_desired_(n, 0), current_desired_(n, 0),
         mode_desired_(n, OPEN) {}
    enum Mode{OPEN, BRAKE, CURRENT, POSITION};
    void set_mode(Mode);
    void set_mode(std::vector<Mode> modes);
    std::vector<Mode> get_mode() const { return mode_desired_; }
    void set_position(double position);
    void set_position(std::vector<double> position);
    void set_current(double current);
    void set_current(std::vector<double> current);
    void update(std::vector<Status> statuses, std::vector<Command> &commands);
 private:
    std::vector<double> position_desired_;
    std::vector<double> current_desired_;
    std::vector<Mode> mode_desired_;
    std::vector<PositionController> position_controllers_;
    int n_;
};



#endif
