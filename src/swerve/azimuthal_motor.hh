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

    /// @brief Updates the system's PID controller to reach the target angle
    /// @return Nothing
    auto
    update() noexcept -> void;

    /// @brief Sets the current angle.
    /// DOES NOT MAKE THE WHEEL REACH THIS ANGLE.
    /// It merely overrides the encoder's position
    /// @param angle
    /// @return
    auto
    set_current_angle(units::degree_t angle) noexcept -> void;

    /// @brief Sets the angle at which the wheel should be positioned
    /// @param angle
    /// @return
    auto
    set_target_angle(units::degree_t angle) noexcept -> void;

    /// @brief Returns the current wheel angle
    /// @return The wheel's current angle
    [[nodiscard]] auto
    get_current_angle() noexcept -> units::degree_t;

    /// @brief Returns the target wheel angle
    /// @return The wheel's target angle
    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;


private:

    rev::CANSparkMax                   controller;
    ctre::phoenix6::hardware::CANcoder encoder;
    frc::PIDController                 pid_controller;

    units::degree_t target_angle;
};

} // namespace td::sub::swerve
