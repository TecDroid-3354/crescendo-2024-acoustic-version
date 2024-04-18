#pragma once

#include <array>

#include <frc/geometry/Translation2d.h>

#include "ctre.hh"
#include "pid_controller.hh"
#include "rev.hh"

namespace td::cfg {
struct swerve_module_config {
    spark_max_config azimuth_controller_config;
    spark_max_config propulsion_controller_config;

    cancoder                  cancoder_config;
    encoder_output_parameters propulsion_encoder_config;

    pid_config       azimuth_pid_config;
    spark_pid_config propulsion_pid_config;

    frc::Translation2d offset;
};

struct swerve_drive_config {
    swerve_module_config front_right;
    swerve_module_config front_left;
    swerve_module_config back_left;
    swerve_module_config back_right;
    pid_config           angle_pid_config;
    pid_config           align_pid_config;
};
} // namespace td::cfg
