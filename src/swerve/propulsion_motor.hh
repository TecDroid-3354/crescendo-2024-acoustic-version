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

    /// @brief Update's the system's PIDF controller to reach the target velocity
    /// @return Nothing
    auto
    update() noexcept -> void;

    /// @brief Set the system's target velocity
    /// @param velocity The target velocity
    /// @return Nothing
    auto
    set_target_velocity(units::meters_per_second_t const &velocity) noexcept -> void;

    /// @brief Returns the wheel's current velocity
    /// @return The wheel's velocity
    [[nodiscard]] auto
    get_current_velocity() const noexcept -> units::meters_per_second_t;

    /// @brief Returns the wheel's target velocity
    /// @return The velocity at which the wheel should be
    [[nodiscard]] auto
    get_target_velocity() const noexcept -> units::meters_per_second_t;

    /// @brief Returns the wheel's travelled distance
    /// @return The travelled distance of this wheel
    [[nodiscard]] auto
    get_travelled_distance() const noexcept -> units::meter_t;


private:

    rev::CANSparkMax          controller;
    rev::SparkRelativeEncoder encoder;
    rev::SparkPIDController   pid_controller;

    units::meters_per_second_t target_velocity;
};

} // namespace td::sub::swerve
