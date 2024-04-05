#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "config/swerve.hh"

namespace units {
using turns_per_minute_t = units::unit_t<units::compound_unit<units::turn, units::inverse<units::minute>>>;
} // namespace units

namespace td::k::swerve {

constexpr units::dimensionless::scalar_t linear_velocity_factor  = 1.0;
constexpr units::dimensionless::scalar_t angular_velocity_factor = 1.0;

constexpr units::degree_t full_turn = 360_deg;

constexpr units::meter_t module_forwards_offset = 10.875_in;
constexpr units::meter_t module_sideways_offset = 10.875_in;

constexpr units::turns_per_minute_t neo_motor_max_rpm = units::turns_per_minute_t { 5676.0 };

constexpr units::dimensionless::scalar_t propulsion_gear_ratio = 6.12;
constexpr units::dimensionless::scalar_t azimuthal_gear_ratio  = 150.0 / 7.0;

constexpr units::meter_t wheel_diameter      = 4_in;
constexpr units::meter_t wheel_circumference = wheel_diameter * std::numbers::pi;

constexpr units::turns_per_second_t propulsion_output_max_rps = neo_motor_max_rpm / propulsion_gear_ratio;

constexpr units::meters_per_second_t max_linear_velocity =
        units::meters_per_second_t { wheel_circumference.value() * propulsion_output_max_rps.value() };

constexpr units::meters_per_second_t target_linear_velocity = max_linear_velocity * linear_velocity_factor;

constexpr units::dimensionless::scalar_t azimuthal_output_position_conversion_factor =
        full_turn.value() / azimuthal_gear_ratio;

constexpr units::dimensionless::scalar_t propulsion_output_position_conversion_factor =
        wheel_circumference.value() / propulsion_gear_ratio.value();

constexpr units::dimensionless::scalar_t propulsion_output_velocity_conversion_factor =
        max_linear_velocity.value() / neo_motor_max_rpm.value();

constexpr units::degrees_per_second_t target_angular_velocity = 360.0_deg_per_s;

constexpr units::second_t propulsion_ramp_rate = 0.25_s;
constexpr units::second_t azimuthal_ramp_rate  = 0.25_s;

constexpr config::pid_controller propulsion_pid_config = { .p          = 0.55,
                                                           .i          = 0.0,
                                                           .d          = 0.0,
                                                           .tolerance  = 0.001,
                                                           .ci_enabled = false

};

constexpr config::pid_controller azimuthal_pid_config = { .p          = 0.0,
                                                          .i          = 0.0,
                                                          .d          = 0.0,
                                                          .tolerance  = 0.1,
                                                          .ci_enabled = true,
                                                          .ci_min     = -180,
                                                          .ci_max     = +180

};

constexpr config::spark_max propulsion_front_right_controller_config = { //
    .id               = 31,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = propulsion_ramp_rate,
    .closed_ramp_rate = propulsion_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max propulsion_front_left_controller_config = { //
    .id               = 41,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = propulsion_ramp_rate,
    .closed_ramp_rate = propulsion_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max propulsion_back_left_controller_config = { //
    .id               = 11,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = propulsion_ramp_rate,
    .closed_ramp_rate = propulsion_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max propulsion_back_right_controller_config = { //
    .id               = 21,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = propulsion_ramp_rate,
    .closed_ramp_rate = propulsion_ramp_rate,
    .is_inverted      = false
};

constexpr config::neo_encoder propulsion_encoder_config = { //
    .position_conversion_factor = propulsion_output_position_conversion_factor,
    .velocity_conversion_factor = propulsion_output_velocity_conversion_factor
};

constexpr config::spark_max azimuthal_front_right_controller_config = { //
    .id               = 32,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = azimuthal_ramp_rate,
    .closed_ramp_rate = azimuthal_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max azimuthal_front_left_controller_config = { //
    .id               = 42,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = azimuthal_ramp_rate,
    .closed_ramp_rate = azimuthal_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max azimuthal_back_left_controller_config = { //
    .id               = 12,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = azimuthal_ramp_rate,
    .closed_ramp_rate = azimuthal_ramp_rate,
    .is_inverted      = false
};

constexpr config::spark_max azimuthal_back_right_controller_config = { //
    .id               = 22,
    .motor_type       = rev::CANSparkMax::MotorType::kBrushless,
    .idle_mode        = rev::CANSparkMax::IdleMode::kBrake,
    .open_ramp_rate   = azimuthal_ramp_rate,
    .closed_ramp_rate = azimuthal_ramp_rate,
    .is_inverted      = false
};

constexpr config::cancoder front_right_cancoder_config { .id = 13 };
constexpr config::cancoder front_left_cancoder_config { .id = 23 };
constexpr config::cancoder back_left_cancoder_config { .id = 33 };
constexpr config::cancoder back_right_cancoder_config { .id = 43 };

constexpr config::swerve_drive swerve_drive_config = {
    .front_right = {.azimuth_controller_config    = azimuthal_front_right_controller_config,
                    .propulsion_controller_config = propulsion_front_right_controller_config,
                    .cancoder_config              = front_right_cancoder_config,
                    .propulsion_encoder_config    = propulsion_encoder_config,
                    .azimuth_pid_config           = azimuthal_pid_config,
                    .propulsion_pid_config        = propulsion_pid_config,
                    .offset = frc::Translation2d { module_forwards_offset, -module_sideways_offset } }

    ,

    .front_left = { .azimuth_controller_config    = azimuthal_front_left_controller_config,
                    .propulsion_controller_config = propulsion_front_left_controller_config,
                    .cancoder_config              = front_left_cancoder_config,
                    .propulsion_encoder_config    = propulsion_encoder_config,
                    .azimuth_pid_config           = azimuthal_pid_config,
                    .propulsion_pid_config        = propulsion_pid_config,
                    .offset = frc::Translation2d { module_forwards_offset, module_sideways_offset }  },

    .back_left = { .azimuth_controller_config    = azimuthal_back_left_controller_config,
                    .propulsion_controller_config = propulsion_back_left_controller_config,
                    .cancoder_config              = back_left_cancoder_config,
                    .propulsion_encoder_config    = propulsion_encoder_config,
                    .azimuth_pid_config           = azimuthal_pid_config,
                    .propulsion_pid_config        = propulsion_pid_config,
                    .offset = frc::Translation2d { -module_forwards_offset, module_sideways_offset } },

    .back_right = { .azimuth_controller_config    = azimuthal_back_right_controller_config,
                    .propulsion_controller_config = propulsion_back_right_controller_config,
                    .cancoder_config              = back_right_cancoder_config,
                    .propulsion_encoder_config    = propulsion_encoder_config,
                    .azimuth_pid_config           = azimuthal_pid_config,
                    .propulsion_pid_config        = propulsion_pid_config,
                    .offset = frc::Translation2d { -module_forwards_offset, -module_sideways_offset }}
};
} // namespace td::k::swerve
