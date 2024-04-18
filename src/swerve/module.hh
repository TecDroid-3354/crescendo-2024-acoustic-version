#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "azimuthal_motor.hh"
#include "propulsion_motor.hh"

#include "config/swerve.hh"

namespace td::sub::swerve {

class individual_module {
public:

    explicit individual_module(cfg::swerve_module_config const &config);

    auto
    update() noexcept -> void;

    auto
    optimize_state(frc::SwerveModuleState const &state) const noexcept -> frc::SwerveModuleState;

    auto
    set_target_state(frc::SwerveModuleState const &state) noexcept -> void;

    [[nodiscard]] auto
    get_current_angle() noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_current_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    get_target_velocity() const noexcept -> units::meters_per_second_t;

    [[nodiscard]] auto
    get_travelled_distance() const noexcept -> units::meter_t;

    [[nodiscard]] auto
    get_module_position() noexcept -> frc::SwerveModulePosition;

    [[nodiscard]] auto
    get_module_state() noexcept -> frc::SwerveModuleState;

    [[nodiscard]] auto
    get_offset_from_center() const noexcept -> frc::Translation2d;

private:

    frc::SwerveModuleState target_state;
    frc::Translation2d     offset;

    azimuthal_motor  azimuth;
    propulsion_motor propulsion;
};

} // namespace td::sub::swerve
