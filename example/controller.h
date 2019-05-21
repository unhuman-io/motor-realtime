#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "motor.h"
#include <vector>

class Controller {
 public:
    void update(std::vector<Status> statuses, std::vector<Command> commands);
 private:
    int32_t last_count_;
    double position_last_;
};

#endif
