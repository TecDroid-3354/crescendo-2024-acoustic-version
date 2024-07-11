#pragma once

#include <map>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "config/encoder.hh"
#include "config/pid_controller.hh"
#include "config/rev.hh"
#include "constants/motors.hh"

namespace td::k::shooter {

namespace position {

/// @brief Min angle that the shooter can adopt (With offset applied)
constexpr units::degree_t min_possible_angle = 0.0_deg;

/// @brief Max angle that the shooter can adopt (With offset applied)
constexpr units::degree_t max_possible_angle = 37_deg;

/// @brief When the encoder is in the desired 0-position, this the offset to apply
/// to actually make it zero
constexpr units::degree_t zero_offset = 344.8_deg;

/// @brief Angle at which to raise the shooter for: Amp
constexpr units::degree_t amp_target_angle = 31_deg;

/// @brief Angle at which to raise the shooter for: Speaker
constexpr units::degree_t speaker_target_angle = 27_deg;

/// @brief Angle at which to raise the shooter for: High Passes
constexpr units::degree_t pass_target_angle = 31_deg;

/// @brief Angle at which to raise the shooter for: Low Passes
constexpr units::degree_t lowpass_target_angle = 0.0_deg;

/// @brief Angle at which to raise the shooter for: Amp
constexpr units::degree_t home_angle = 0.0_deg;

/// @brief Angle to add to each measurement as a slight global offset
constexpr units::degree_t angle_offset = 0.5_deg;

/// @brief Speed factor at which to raise or lower the shooter
constexpr double speed_factor = 1.0;

/// @brief Theshold needed to activate shooter auto-targetting
constexpr double activation_trigger_threshold = 0.1;

constexpr cfg::spark_max_config controller_config_a = {
    .identity = {
        .id             = 5,
        .motor_type     = rev::CANSparkMax::MotorType::kBrushless,
    },

    .behavior = {
        .idle_mode      = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate = 0_s,
        .closed_ramp_rate = 0_s,
        .is_inverted      = true
    }
};

constexpr cfg::spark_max_config controller_config_b = {
    .identity = {
        .id             = 6,
        .motor_type     = rev::CANSparkMax::MotorType::kBrushless,
    },

    .behavior = {
        .idle_mode      = rev::CANSparkMax::IdleMode::kBrake,
        .open_ramp_rate = 0_s,
        .closed_ramp_rate = 0_s,
        .is_inverted      = false
    }
};

constexpr cfg::pid_config pid_controller_config = {
    .coefficients { .p = 0.045, .i = 0.0, .d = 0.0 },
    .output_parameters = { .tolerance = 0.0, .ci_enabled = false, .ci_min = 0.0, .ci_max = 0.0 }
};

constexpr cfg::encoder_output_parameters encoder_config = { .position_conversion_factor = 360.0,
                                                            .velocity_conversion_factor = 60,
                                                            .is_inverted                = true };

} // namespace position

namespace spin {

/// @brief Percentage at which to operate both motors to shoot at the speaker
constexpr double speaker_target_percentage = 0.9;

/// @brief Percentage at which to operate the top motor to shoot at the amp
constexpr double amp_top_target_percentage = 0.05;

/// @brief Percentage at which to operate the bottom motor to shoot at the amp
constexpr double amp_bottom_target_percentage = 0.315;

constexpr double pass_percentage = 0.45;

/// @brief Controller threshold at which to activate shooting
constexpr double activation_trigger_threshold = 0.5;

constexpr cfg::spark_max_config controller_config_bottom = {
    .identity = { .id = 3, .motor_type = rev::CANSparkBase::MotorType::kBrushless },

    .behavior = { .idle_mode        = rev::CANSparkBase::IdleMode::kBrake,
                 .open_ramp_rate   = 1.0_s,
                 .closed_ramp_rate = 1.0_s,
                 .is_inverted      = true }
};

constexpr cfg::spark_max_config controller_config_top = {
    .identity = { .id = 4, .motor_type = rev::CANSparkBase::MotorType::kBrushless },

    .behavior = { .idle_mode        = rev::CANSparkBase::IdleMode::kBrake,
                 .open_ramp_rate   = 1.0_s,
                 .closed_ramp_rate = 1.0_s,
                 .is_inverted      = false }
};

} // namespace spin

} // namespace td::k::shooter
