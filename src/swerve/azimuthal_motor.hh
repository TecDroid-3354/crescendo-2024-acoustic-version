#pragma once

#include <functional>
#include <units/angle.h>

#include <ctre/phoenix6/CANcoder.hpp>

#include "config/cancoder.hh"
#include "config/pid.hh"
#include "config/rev.hh"

namespace td::swerve {

class azimuthal_motor {
public:

    explicit azimuthal_motor(
            config::spark_max      spark_max_config,
            config::cancoder       encoder_config,
            config::pid_controller pid_controller_config);

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

    [[nodiscard]] auto
    expose_pid_controller() noexcept -> frc::PIDController *;


private:

    rev::CANSparkMax                   controller;
    ctre::phoenix6::hardware::CANcoder encoder;
    frc::PIDController                 pid_controller;

    units::degree_t target_angle;
};

} // namespace td::swerve
