#pragma once

#include "config/pid_controller.hh"

namespace td::k::auton {

// Autonomous PID coefficients

cfg::pid_coefficients move_coefficients { .p = 5.0, .i = 0.0, .d = 0.0 };
cfg::pid_coefficients turn_coefficients { .p = 1.75, .i = 0.0, .d = 0.0 };
} // namespace td::k::auton
