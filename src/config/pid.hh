#pragma once

#include <frc/controller/PIDController.h>

namespace td::config {

struct pid_controller {
    double p         = 0.0;
    double i         = 0.0;
    double d         = 0.0;
    double tolerance = 0.001;

    bool   ci_enabled = false;
    double ci_min     = 0.0;
    double ci_max     = 0.0;
};

[[nodiscard]] auto
mkpid_controller(pid_controller const &config) noexcept -> frc::PIDController &&;

} // namespace td::config
