#pragma once

#include <cstdint>
#include <functional>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include "hardware/gyroscope.hh"
#include "module.hh"

namespace td::sub::swerve {

enum class swerve_mode {
    ROBOT_ORIENTED,
    FIELD_ORIENTED
};

constexpr uint8_t front_right_idx = 0;
constexpr uint8_t front_left_idx  = 1;
constexpr uint8_t back_left_idx   = 2;
constexpr uint8_t back_right_idx  = 3;

class swerve_drive: public frc2::SubsystemBase {
public:

    explicit swerve_drive(cfg::swerve_drive_config const &config);

    auto
    Periodic() -> void override;

    /// @brief Causes the robot to drive with the given speeds
    /// @param speed The speeds
    /// @return Nothing
    auto
    drive(frc::ChassisSpeeds const &speed) noexcept -> void;

    /// @brief Turns by the given angle relative to the robot's current angle
    /// @param angle The angle to rotate by
    /// @return The command to execute
    [[nodiscard]] auto
    turn_by_angle(units::degree_t angle) noexcept -> frc2::CommandPtr;

    /// @brief Causes the robot to align to an object such that the horizontal
    /// offset is 0
    /// @param target_cb The offset callback
    /// @param fallback_condition Do not align if this condition is met
    /// @return The command to execute
    [[nodiscard]] auto
    align_with(std::function<units::degree_t()> target_cb, std::function<bool()> fallback_condition)
            -> frc2::CommandPtr;

    /// @brief Sets the callback that will return the forwards velocity coefficient (-1.0 to 1.0)
    /// @param forwards_motion_source_cb The callback
    /// @return Nothing
    auto
    set_forwards_motion_source(std::function<double()> forwards_motion_source_cb) noexcept -> void;

    /// @brief Sets the callback that will return the sideways velocity coefficient (-1.0 to 1.0)
    /// @param forwards_motion_source_cb The callback
    /// @return Nothing
    auto
    set_sideways_motion_source(std::function<double()> sideways_motion_source_cb) noexcept -> void;

    /// @brief Sets the callback that will return the angular velocity coefficient (-1.0 to 1.0)
    /// @param forwards_motion_source_cb The callback
    /// @return Nothing
    auto
    set_rotation_motion_source(std::function<double()> rotation_motion_source_cb) noexcept -> void;

    /// @brief Sets the backup callback that will return the angular velocity coefficient (-1.0 to 1.0)
    /// @param forwards_motion_source_cb The backup callback
    /// @return Nothing
    auto
    set_rotation_motion_fallback_source(std::function<double()> rotation_motion_fallback_source_cb) noexcept -> void;

    /// @brief Returns the robot's pose on the field
    /// @return The pose
    [[nodiscard]] auto
    get_pose() noexcept -> frc::Pose2d;

    /// @brief Sets the robot's pose on the field
    /// @return Nothing
    auto
    set_pose(frc::Pose2d pose) noexcept -> void;

    /// @brief Gets the robot's yaw
    /// @return The yaw
    [[nodiscard]] auto
    get_yaw() noexcept -> units::degree_t;

    /// @brief Resets the robot's yaw
    /// @return Nothing
    auto
    reset_yaw() noexcept -> void;

    /// @brief Get the robot's curent chassis speeds
    /// @return The current speeds
    [[nodiscard]] auto
    get_current_speeds() noexcept -> frc::ChassisSpeeds;

    /// @brief The position of each module (travelled distance)
    /// @return
    [[nodiscard]] auto
    get_module_position_array() noexcept -> wpi::array<frc::SwerveModulePosition, 4>;

    /// @brief The state of each module (angle + velocity)
    /// @return
    [[nodiscard]] auto
    get_module_state_array() noexcept -> wpi::array<frc::SwerveModuleState, 4>;

private:

    individual_module front_right;
    individual_module front_left;
    individual_module back_left;
    individual_module back_right;

    std::function<double()> forwards_motion_source;
    std::function<double()> sideways_motion_source;
    std::function<double()> rotation_motion_source;

    std::function<double()> rotation_motion_fallback_source;

    hardware::gyroscope gyro;

    frc::SwerveDriveKinematics<4> kinematics;
    frc::SwerveDriveOdometry<4>   odometry;

    frc::PIDController angle_pid_controller;
    frc::PIDController align_pid_controller;
    frc::Field2d       field;
    swerve_mode        current_mode;
};

} // namespace td::sub::swerve
