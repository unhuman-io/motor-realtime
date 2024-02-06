#pragma once

#include "motor.h"

namespace obot {

class MotorUSB : public Motor {
   public:
    MotorUSB() = default;

    MotorUSB(std::string dev_path);

    virtual ~MotorUSB();

    ssize_t read() final;

    ssize_t write() final;

    int lock() final;

    int set_nonblock() final;

    int clear_nonblock() final;

    ssize_t aread() final;

   protected:
    /// @brief Open the file descriptor
    /// @return file descriptor on success, -1 on failure
    int open() final;

    /// @brief Close the file descriptor
    /// @return 0 on success, -1 on failure
    int close() final;
};

}  // namespace obot