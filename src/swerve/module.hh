#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "azimuthal_motor.hh"
#include "propulsion_motor.hh"

#include "config/swerve.hh"

namespace td::swerve {

class individual_module {
public:

    explicit individual_module(config::swerve_module const &config);

    auto
    update() noexcept -> void;

    auto
    optimize_state(frc::SwerveModuleState const &state) const noexcept -> frc::SwerveModuleState;

    auto
    set_target_state(frc::SwerveModuleState const &state) noexcept -> void;

    [[nodiscard]] auto
    current_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    current_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    target_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    target_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    center_offset() const noexcept -> frc::Translation2d;

    [[nodiscard]] auto
    expose_azimuth_pid() noexcept -> frc::PIDController &;

    [[nodiscard]] auto
    expose_propulsion_pid() noexcept -> frc::PIDController &;

    auto
    log() const noexcept -> void;

private:

    frc::SwerveModuleState target_state;
    frc::Translation2d     offset;

    azimuthal_motor  azimuth;
    propulsion_motor propulsion;
};

} // namespace td::swerve
