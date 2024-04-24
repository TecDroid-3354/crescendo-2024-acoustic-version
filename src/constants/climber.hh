#pragma once

#include <units/length.h>

#include "config/encoder.hh"
#include "config/rev.hh"

namespace td::k::climber {

constexpr units::meter_t bottom_boundary = 0.0_m;
constexpr units::meter_t top_boundary    = 4.75_m;

constexpr double gear_ratio = 4.0 * 4.0 * 3.0;

constexpr double speed = 0.8;

constexpr cfg::spark_max_config left_controller_config = {
    .identity = { .id = 7, .motor_type = rev::CANSparkMax::MotorType::kBrushless },
    .behavior { .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
                 .open_ramp_rate   = 0.0_s,
                 .closed_ramp_rate = 0.0_s,
                 .is_inverted      = true }
};

constexpr cfg::spark_max_config right_controller_config = {
    .identity = { .id = 8, .motor_type = rev::CANSparkMax::MotorType::kBrushless },
    .behavior { .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
                 .open_ramp_rate   = 0.0_s,
                 .closed_ramp_rate = 0.0_s,
                 .is_inverted      = false }
};

constexpr cfg::encoder_output_parameters encoder_config = { .position_conversion_factor = 1.0 / gear_ratio,
                                                            .velocity_conversion_factor = 1.0,
                                                            .is_inverted                = false };

} // namespace td::k::climber
