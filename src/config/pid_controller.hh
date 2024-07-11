#pragma once

#include <frc/controller/PIDController.h>

namespace td::cfg {

/// @brief PID coefficient config structure
struct pid_coefficients {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
};

/// @brief PID output config structure
struct pid_output_parameters {
    double tolerance = 0.001;

    bool   ci_enabled = false;
    double ci_min     = 0.0;
    double ci_max     = 0.0;
};

/// @brief PID glob config structure
struct pid_config {
    pid_coefficients      coefficients;
    pid_output_parameters output_parameters;
};

/// @brief Function to apply PID config parameters
/// @param controller The PID controller
/// @param config The config structure
/// @return Nothing
auto
configure_pid_controller(frc::PIDController *controller, pid_config const &config) noexcept -> void;

} // namespace td::cfg
