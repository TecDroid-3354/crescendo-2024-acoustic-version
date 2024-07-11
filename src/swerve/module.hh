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

    /// @brief Update the state of both motors
    /// @return Nothing
    auto
    update() noexcept -> void;

    /// @brief Optimize a given state according to the module's current settings in order to make it travel the least
    /// distance possible
    /// @param state The state to optimize
    /// @return The optimized state
    auto
    optimize_state(frc::SwerveModuleState const &state) const noexcept -> frc::SwerveModuleState;

    /// @brief Sets the module's target state
    /// @param state The state to assign
    /// @return Nothing
    auto
    set_target_state(frc::SwerveModuleState const &state) noexcept -> void;

    /// @brief Returns the module's current wheel angle
    /// @return The angle of the module's wheel
    [[nodiscard]] auto
    get_current_angle() noexcept -> units::degree_t;

    /// @brief Returns the module's target wheel angle
    /// @return The target angle of the module's wheel
    [[nodiscard]] auto
    get_target_angle() const noexcept -> units::degree_t;

    /// @brief Returns the module's current wheel velocity
    /// @return The velocity of the module's wheel
    [[nodiscard]] auto
    get_current_velocity() const noexcept -> units::meters_per_second_t;

    /// @brief Returns the module's target wheel velocity
    /// @return The velocity of the module's wheel
    [[nodiscard]] auto
    get_target_velocity() const noexcept -> units::meters_per_second_t;

    /// @brief Returns the distance travelled by this module
    /// @return The distance
    [[nodiscard]] auto
    get_travelled_distance() const noexcept -> units::meter_t;

    /// @brief Returns this module's position (Travelled distance + Angle)
    /// @return The module's position
    [[nodiscard]] auto
    get_module_position() noexcept -> frc::SwerveModulePosition;

    /// @brief Returns this module's state (Velocity + Angle)
    /// @return The module's position
    [[nodiscard]] auto
    get_module_state() noexcept -> frc::SwerveModuleState;

    /// @brief Returns this module's offset from the center of the robot
    /// @return The module's center offset
    [[nodiscard]] auto
    get_offset_from_center() const noexcept -> frc::Translation2d;

private:

    frc::SwerveModuleState target_state;
    frc::Translation2d     offset;

    azimuthal_motor  azimuth;
    propulsion_motor propulsion;
};

} // namespace td::sub::swerve
