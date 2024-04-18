#pragma once

#include "config/swerve.hh"

namespace td::k::dt::azim {

constexpr units::degree_t full_turn        = 360_deg;
constexpr units::degree_t module_angle_min = -180.0_deg;
constexpr units::degree_t module_angle_max = +180.0_deg;

constexpr units::second_t ramp_rate = 0.25_s;

constexpr cfg::pid_config pid_config = {
    .coefficients      = { .p = 0.01, .i = 0.0, .d = 0.00035 },
    .output_parameters = { .tolerance  = 0.01,
                          .ci_enabled = true,
                          .ci_min     = module_angle_min.value(),
                          .ci_max     = module_angle_max.value() }
};

// Unused due to using cancoder
// constexpr units::dimensionless::scalar_t gear_ratio = 150.0 / 7.0;
// constexpr units::dimensionless::scalar_t output_position_conversion_factor = full_turn.value() / gear_ratio;

constexpr cfg::spark_max_config front_right_controller_config = {
    .identity = {
        .id               = 12,
        .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    }, 
    
    .behavior = {
        .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate   = ramp_rate,
        .closed_ramp_rate = ramp_rate,
        .is_inverted      = false
    }
};

constexpr cfg::spark_max_config front_left_controller_config = {
    .identity = {
        .id               = 22,
        .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    },
    
    .behavior = {
        .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate   = ramp_rate,
        .closed_ramp_rate = ramp_rate,
        .is_inverted      = false
    }
};

constexpr cfg::spark_max_config back_left_controller_config = {
    .identity = {
        .id         = 32,
        .motor_type = rev::CANSparkMax::MotorType::kBrushless,
    },

    .behavior = {
        .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate   = ramp_rate,
        .closed_ramp_rate = ramp_rate,
        .is_inverted      = false
    }
};

constexpr cfg::spark_max_config back_right_controller_config = {
    .identity = {
        .id         = 42,
        .motor_type = rev::CANSparkMax::MotorType::kBrushless,
    },
    
    .behavior = {
        .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate   = ramp_rate,
        .closed_ramp_rate = ramp_rate,
        .is_inverted      = false
    }
};

constexpr cfg::cancoder front_right_cancoder_config { .id = 13 };
constexpr cfg::cancoder front_left_cancoder_config { .id = 23 };
constexpr cfg::cancoder back_left_cancoder_config { .id = 33 };
constexpr cfg::cancoder back_right_cancoder_config { .id = 43 };

} // namespace td::k::dt::azim
