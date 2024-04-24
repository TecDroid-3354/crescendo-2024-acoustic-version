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

    [[nodiscard]] auto
    enable_target_tracking() noexcept -> frc2::CommandPtr;

    [[nodiscard]] auto
    disable_target_tracking() noexcept -> frc2::CommandPtr;

    auto
    set_target_angle_source(std::function<units::degree_t()> function) noexcept -> void;

    auto
    enable_fixed_targetting() noexcept -> frc2::CommandPtr;

    auto
    disable_fixed_targetting() noexcept -> frc2::CommandPtr;

    auto reach_fixed_target() noexcept -> frc2::CommandPtr;

    auto
    set_fixed_target(units::degree_t angle) noexcept -> frc2::CommandPtr;

    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_current_angle() const noexcept -> units::degree_t;

    auto
    set_zero_offset(double offset) noexcept -> void;

    auto
    set_percentage(double percentage) -> void;

    auto
    md_coast() -> frc2::CommandPtr;

    auto
    md_break() -> frc2::CommandPtr;

    auto stop() -> frc2::CommandPtr;

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
