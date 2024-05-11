#pragma once

#include "azimuthal.hh"
#include "config/swerve.hh"
#include "propulsion.hh"

namespace td::k::dt::swerve {

/// @brief PID for chassis turn-to-angle
constexpr cfg::pid_config angle_turn_pid_config = {
    .coefficients      = { .p = 0.02, .i = 0.0, .d = 0.0015 },
    .output_parameters = { .tolerance = 0.001, .ci_enabled = true, .ci_min = -180.0, .ci_max = +180.0 }
};

/// @brief PID for limelight targetting
constexpr cfg::pid_config align_pid_config = {
    .coefficients      = { .p = 0.025, .i = 0.01, .d = 0.0015 },
    .output_parameters = { .tolerance = 0.001, .ci_enabled = false, .ci_min = 0.0, .ci_max = 0.0 }
};

constexpr cfg::swerve_drive_config swerve_drive_config = {
    .front_right = {.azimuth_controller_config    = azim::front_right_controller_config,
                    .propulsion_controller_config = prop::front_right_controller_config,
                    .cancoder_config              = azim::front_right_cancoder_config,
                    .propulsion_encoder_config    = prop::encoder_config,
                    .azimuth_pid_config           = azim::pid_config,
                    .propulsion_pid_config        = prop::pid_config,
                    .offset = frc::Translation2d { ms::module_forwards_offset, -ms::module_sideways_offset } },
    .front_left  = { .azimuth_controller_config    = azim::front_left_controller_config,
                    .propulsion_controller_config = prop::front_left_controller_config,
                    .cancoder_config              = azim::front_left_cancoder_config,
                    .propulsion_encoder_config    = prop::encoder_config,
                    .azimuth_pid_config           = azim::pid_config,
                    .propulsion_pid_config        = prop::pid_config,
                    .offset = frc::Translation2d { ms::module_forwards_offset, ms::module_sideways_offset }  },
    .back_left   = { .azimuth_controller_config    = azim::back_left_controller_config,
                    .propulsion_controller_config = prop::back_left_controller_config,
                    .cancoder_config              = azim::back_left_cancoder_config,
                    .propulsion_encoder_config    = prop::encoder_config,
                    .azimuth_pid_config           = azim::pid_config,
                    .propulsion_pid_config        = prop::pid_config,
                    .offset = frc::Translation2d { -ms::module_forwards_offset, ms::module_sideways_offset } },
    .back_right  = { .azimuth_controller_config    = azim::back_right_controller_config,
                    .propulsion_controller_config = prop::back_right_controller_config,
                    .cancoder_config              = azim::back_right_cancoder_config,
                    .propulsion_encoder_config    = prop::encoder_config,
                    .azimuth_pid_config           = azim::pid_config,
                    .propulsion_pid_config        = prop::pid_config,
                    .offset = frc::Translation2d { -ms::module_forwards_offset, -ms::module_sideways_offset }},

    .angle_pid_config = angle_turn_pid_config,
    .align_pid_config = align_pid_config
};
} // namespace td::k::dt::swerve
