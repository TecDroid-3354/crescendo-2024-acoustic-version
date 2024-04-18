#pragma once

#include <units/angular_velocity.h>
#include "config/rev.hh"
#include "motors.hh"

namespace td::k::intake {

constexpr double target_percentage = 0.6;

constexpr cfg::spark_max_config controller_config = {
    .identity = { .id = 1, .motor_type = rev::CANSparkBase::MotorType::kBrushless },

    .behavior = { .idle_mode        = rev::CANSparkBase::IdleMode::kBrake,
                 .open_ramp_rate   = 0.5_s,
                 .closed_ramp_rate = 0.5_s,
                 .is_inverted      = true }
};

} // namespace td::k::intake
