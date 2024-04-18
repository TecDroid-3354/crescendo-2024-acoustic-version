#pragma once

#include <functional>

#include <units/velocity.h>

#include "config/pid_controller.hh"
#include "config/rev.hh"

namespace td::sub::swerve {

class propulsion_motor {
public:

    explicit propulsion_motor(
            cfg::spark_max_config          controller_config,
            cfg::encoder_output_parameters encoder_config,
            cfg::spark_pid_config          pid_controller_config);

    auto
    update() noexcept -> void;

    auto
    set_target_velocity(units::meters_per_second_t const &velocity) noexcept -> void;

    [[nodiscard]] auto
    get_current_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    get_target_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    get_travelled_distance() const noexcept -> units::meter_t;


private:

    rev::CANSparkMax          controller;
    rev::SparkRelativeEncoder encoder;
    rev::SparkPIDController   pid_controller;

    units::meters_per_second_t target_velocity;
};

} // namespace td::sub::swerve
