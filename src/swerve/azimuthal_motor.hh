#pragma once

#include <functional>
#include <units/angle.h>

#include "config/pid.hh"
#include "config/rev.hh"
#include "util/sm_observable.hh"

namespace td::swerve {

class azimuthal_motor {
public:

    explicit azimuthal_motor(
            config::spark_max      spark_max_config,
            config::neo_encoder    neo_encoder_config,
            config::pid_controller pid_controller_config);

    auto
    update() noexcept -> void;

    auto
    set_current_angle(units::degree_t angle) noexcept -> void;

    auto
    set_target_angle(units::degree_t angle) noexcept -> void;

    [[nodiscard]] auto
    get_current_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    expose_pid_controller() noexcept -> frc::PIDController &;

    auto
    log() const noexcept -> void;


private:

    rev::CANSparkMax          controller;
    rev::SparkRelativeEncoder encoder;
    frc::PIDController        pid_controller;

    units::degree_t target_angle;
};

} // namespace td::swerve
