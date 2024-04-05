#pragma once

#include <array>

#include <frc/geometry/Translation2d.h>

#include "cancoder.hh"
#include "pid.hh"
#include "rev.hh"

namespace td::config {
struct swerve_module {
    spark_max azimuth_controller_config;
    spark_max propulsion_controller_config;

    cancoder    cancoder_config;
    neo_encoder propulsion_encoder_config;

    pid_controller azimuth_pid_config;
    pid_controller propulsion_pid_config;

    frc::Translation2d offset;
};

struct swerve_drive {
    swerve_module front_right;
    swerve_module front_left;
    swerve_module back_left;
    swerve_module back_right;
};
} // namespace td::config
