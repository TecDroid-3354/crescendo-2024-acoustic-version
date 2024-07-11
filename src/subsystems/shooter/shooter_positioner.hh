#pragma once

#include <functional>

#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>

#include <rev/CANSparkMax.h>

#include "config/pid_controller.hh"
#include "config/rev.hh"

namespace td::sub {

class shooter_positioner: public frc2::SubsystemBase {
public:

    explicit shooter_positioner(
            cfg::spark_max_config const          &controller_config_a,
            cfg::spark_max_config const          &controller_config_b,
            cfg::encoder_output_parameters const &encoder_config,
            cfg::pid_config const                &pid_controller_config);

    /// @brief JUST ENABLES target tracking. Calculations are done elsewhere
    /// @return The command to just ENABLE the tracking
    [[nodiscard]] auto
    enable_target_tracking() noexcept -> frc2::CommandPtr;

    /// @brief Disables target tracking
    /// @return The command to disable tracking
    [[nodiscard]] auto
    disable_target_tracking() noexcept -> frc2::CommandPtr;

    /// @brief Assigns a callback source that should return at what angle the shooter should position itself
    /// @param function The callback
    /// @return nothing
    auto
    set_target_angle_source(std::function<units::degree_t()> function) noexcept -> void;

    /// @brief JUST ENABLES fixed targetting
    /// This causes the shooter to reach a fixed angle when
    /// target tracking is enabled
    /// @return The command to just ENABLE the targetting
    auto
    enable_fixed_targetting() noexcept -> frc2::CommandPtr;

    /// @brief Disables fixed targetting
    /// @return The command to disable fixed targetting
    auto
    disable_fixed_targetting() noexcept -> frc2::CommandPtr;

    /// @brief Sets the fixed target for fixed targetting mode
    /// @param angle The angle to reach
    /// @return The command to execute
    auto
    set_fixed_target(units::degree_t angle) noexcept -> frc2::CommandPtr;

    /// @brief Returns the target angle
    /// @return The target angle
    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;

    /// @brief Returns the shooter's current angle
    /// @return The current angle
    [[nodiscard]] auto
    get_current_angle() const noexcept -> units::degree_t;

    /// @brief Sets the percentage of both motors (one is inverted)
    /// @param percentage The percentage at which to operate the motors
    /// @return Nothing
    auto
    set_percentage(double percentage) -> void;

    /// @brief Makes the motors coast as no-power behavior
    /// @return The command to execute
    auto
    md_coast() -> frc2::CommandPtr;

    /// @brief Makes the motors break as no-power behavior
    /// @return The command to execute
    auto
    md_break() -> frc2::CommandPtr;

    /// @brief Stops the subsystem
    /// @return The command to execute
    auto
    stop() -> frc2::CommandPtr;

private:

    rev::CANSparkMax      controller_a;
    rev::CANSparkMax      controller_b;
    frc::PIDController    pid_controller;
    frc::DutyCycleEncoder encoder;

    bool            with_fixed_target = false;
    units::degree_t fixed_target      = 0.0_deg;

    std::function<units::degree_t()> target_angle_source;
};

} // namespace td::sub
