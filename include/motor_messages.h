#pragma once

#include <cstdint>
#include "motor_messages/motor_messages.h"
namespace obot {
// TODO probably remove these typedefs
typedef MotorCommand Command;
typedef MotorStatus Status;
typedef MotorMode ModeDesired;
}  // namespace obot
#define RESET   BOARD_RESET
