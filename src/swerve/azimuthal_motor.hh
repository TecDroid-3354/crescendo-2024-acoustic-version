#pragma once

#include <functional>
#include <units/angle.h>

#include <ctre/phoenix6/CANcoder.hpp>

#include "config/ctre.hh"
#include "config/pid_controller.hh"
#include "config/rev.hh"

namespace td::sub::swerve {

class azimuthal_motor {
public:

    explicit azimuthal_motor(
            cfg::spark_max_config controller_config,
            cfg::cancoder         cancoder_config,
            cfg::pid_config       pid_config);

    auto
    update() noexcept -> void;

    auto
    set_current_angle(units::degree_t angle) noexcept -> void;

    auto
    set_target_angle(units::degree_t angle) noexcept -> void;

    [[nodiscard]] auto
    get_current_angle() noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;


private:

    rev::CANSparkMax                   controller;
    ctre::phoenix6::hardware::CANcoder encoder;
    frc::PIDController                 pid_controller;

    units::degree_t target_angle;
};

} // namespace td::sub::swerve
