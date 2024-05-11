#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "config/swerve.hh"
#include "constants/motors.hh"
#include "measurements.hh"
#include "propulsion.hh"

namespace td::k::dt::prop {

namespace ms {

/// @brief Swevre wheel diameter
constexpr units::meter_t wheel_diameter = 4_in;

/// @brief Swerve wheel circumference
constexpr units::meter_t wheel_circumference = wheel_diameter * std::numbers::pi;
} // namespace ms

/// @brief Propulsion ramp rate
constexpr units::second_t ramp_rate = 0.25_s;

/// @brief Global PID config
constexpr cfg::spark_pid_config pid_config = {
    .coefficients       = { .p = 0.0, .i = 0.0, .d = 0.0 },
    .spark_coefficients = { .ff = 0.2275 },
    .output_parameters { .tolerance = 0.001, .ci_enabled = false, .ci_min = -1.0, .ci_max = +1.0 },
    .spark_output_parameters = { .min_output = -1.0, .max_output = +1.0 }
};

/// @brief Propulsion gear ratio
constexpr units::dimensionless::scalar_t gear_ratio = 6.12;

/// @brief Max revs per second of any propulsion motor
constexpr units::turns_per_second_t output_max_rps = k::motors::neo_max_rpm / gear_ratio;

/// @brief Max meters per second of any propulsion motor
constexpr units::meters_per_second_t max_linear_velocity =
        units::meters_per_second_t { ms::wheel_circumference.value() * output_max_rps.value() };

/// @brief Min meters per second of any propulsion motor
constexpr units::meters_per_second_t min_linear_velocity = -max_linear_velocity;

/// @brief PCF per encoder pulse
constexpr units::dimensionless::scalar_t output_position_conversion_factor =
        (ms::wheel_circumference / gear_ratio).value();

/// @brief VCF per encoder pulse per update rate
constexpr units::dimensionless::scalar_t output_velocity_conversion_factor =
        max_linear_velocity.value() / k::motors::neo_max_rpm.value();

constexpr cfg::spark_max_config front_right_controller_config = {
    .identity = {
        .id         = 11,
        .motor_type = rev::CANSparkMax::MotorType::kBrushless,
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
        .id         = 21,
        .motor_type = rev::CANSparkMax::MotorType::kBrushless,
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
        .id         = 31,
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
        .id         = 41,
        .motor_type = rev::CANSparkMax::MotorType::kBrushless,
    }, 
    
    .behavior = {
        .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate   = ramp_rate,
        .closed_ramp_rate = ramp_rate,
        .is_inverted      = false
    }
};

constexpr cfg::encoder_output_parameters encoder_config = {
    .position_conversion_factor = output_position_conversion_factor,
    .velocity_conversion_factor = output_velocity_conversion_factor,
    .is_inverted                = false // ignored due to spark encoder
};

} // namespace td::k::dt::prop
