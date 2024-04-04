#pragma once

#include <functional>

#include <units/velocity.h>

#include "config/pid.hh"
#include "config/rev.hh"

namespace td::swerve {

class propulsion_motor {
public:

    explicit propulsion_motor(
            config::spark_max      spark_max_config,
            config::neo_encoder    neo_encoder_config,
            config::pid_controller pid_controller_config);

    auto
    update() noexcept -> void;

    auto
    set_target_velocity(units::meters_per_second_t const &velocity) noexcept -> void;

    [[nodiscard]] auto
    get_current_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    get_target_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    expose_pid_controller() noexcept -> frc::PIDController &;

    auto
    log() const noexcept -> void;


private:

    rev::CANSparkMax          controller;
    rev::SparkRelativeEncoder encoder;
    frc::PIDController        pid_controller;

    units::meters_per_second_t target_velocity;
};

} // namespace td::swerve
