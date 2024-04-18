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

    auto
    drive(frc::ChassisSpeeds const &speed) noexcept -> void;

    [[nodiscard]] auto
    turn_by_angle(units::degree_t angle) noexcept -> frc2::CommandPtr;

    [[nodiscard]] auto
    align_with(std::function<units::degree_t()> target_cb, std::function<bool()> fallback_condition)
            -> frc2::CommandPtr;

    auto
    set_forwards_motion_source(std::function<double()> forwards_motion_source_cb) noexcept -> void;

    auto
    set_sideways_motion_source(std::function<double()> sideways_motion_source_cb) noexcept -> void;

    auto
    set_rotation_motion_source(std::function<double()> rotation_motion_source_cb) noexcept -> void;

    auto
    set_rotation_motion_fallback_source(std::function<double()> rotation_motion_fallback_source_cb) noexcept -> void;

    [[nodiscard]] auto
    get_pose() noexcept -> frc::Pose2d;

    auto
    set_pose(frc::Pose2d pose) noexcept -> void;

    [[nodiscard]] auto
    get_yaw() noexcept -> units::degree_t;

    auto
    reset_yaw() noexcept -> void;

    [[nodiscard]] auto
    get_current_speeds() noexcept -> frc::ChassisSpeeds;

    [[nodiscard]] auto
    get_module_position_array() noexcept -> wpi::array<frc::SwerveModulePosition, 4>;

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
