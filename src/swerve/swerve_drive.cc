#include "swerve_drive.hh"

#include <cmath>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "constants/control.hh"
#include "constants/drivetrain/targets.hh"
#include "constants/strings.hh"

namespace td::sub::swerve {

swerve_drive::swerve_drive(cfg::swerve_drive_config const &config)
    : front_right { config.front_right }
    , front_left { config.front_left }
    , back_left { config.back_left }
    , back_right { config.back_right }
    , forwards_motion_source{[](){return 0.0;}}
    , sideways_motion_source{[](){return 0.0;}}
    , rotation_motion_source{[](){return 0.0;}}
    , gyro { }
    , kinematics { wpi::array<frc::Translation2d, 4> { front_right.get_offset_from_center(), front_left.get_offset_from_center(), back_left.get_offset_from_center(), back_right.get_offset_from_center() } }
    , odometry {kinematics, get_yaw(), get_module_position_array()}
    , angle_pid_controller{0.0, 0.0, 0.0}
    , align_pid_controller{0.0, 0.0, 0.0}
    , field{ }
    , current_mode{swerve_mode::FIELD_ORIENTED}
{
    cfg::configure_pid_controller(&angle_pid_controller, config.angle_pid_config);
    cfg::configure_pid_controller(&align_pid_controller, config.align_pid_config);

    // Setpoint to align to a target
    align_pid_controller.SetSetpoint(0.0);

    // Data logging
    frc::ShuffleboardTab &swerve_pid_logs = frc::Shuffleboard::GetTab(k::str::swerve_pid_manipulation_tab);

    swerve_pid_logs.Add("Turning PID", angle_pid_controller);
    swerve_pid_logs.Add("Align PID", align_pid_controller);

    // If the drivetrain is not doing any other command, run this
    this->SetDefaultCommand(frc2::cmd::Run(
            [this]() {
                // Get coefficients
                double normalized_forwards =
                        std::clamp(this->forwards_motion_source(), k::ctrl::pid_output_min, k::ctrl::pid_output_max);

                double normalized_sideways =
                        std::clamp(this->sideways_motion_source(), k::ctrl::pid_output_min, k::ctrl::pid_output_max);

                double normalized_angular =
                        std::clamp(this->rotation_motion_source(), k::ctrl::pid_output_min, k::ctrl::pid_output_max);

                frc::ChassisSpeeds chassis_speeds;

                // Apply coefficients based on robot mode
                switch (current_mode) {
                case swerve_mode::ROBOT_ORIENTED :
                    // Direct factor-in
                    chassis_speeds = { k::dt::tgt::linear_velocity * normalized_forwards,
                                       k::dt::tgt::linear_velocity * normalized_sideways,
                                       k::dt::tgt::angular_velocity * normalized_angular };

                    break;
                case swerve_mode::FIELD_ORIENTED :
                    // Factor in, then apply field-relative rotation to speeds
                    units::degree_t current_angle = get_yaw();
                    chassis_speeds                = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            k::dt::tgt::linear_velocity * normalized_forwards,
                            k::dt::tgt::linear_velocity * normalized_sideways,
                            k::dt::tgt::angular_velocity * normalized_angular,
                            std::move(current_angle));
                    break;
                }

                this->drive(std::move(chassis_speeds));
            },
            { this }));
}

auto
swerve_drive::Periodic() -> void {
    // Constantly update modules & odometry
    front_right.update();
    front_left.update();
    back_left.update();
    back_right.update();

    odometry.Update(get_yaw(), get_module_position_array());
    field.SetRobotPose(odometry.GetPose());
}

auto
swerve_drive::drive(frc::ChassisSpeeds const &speed) noexcept -> void {
    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(speed * -1.0);

    // Assigns the target state to each swerve module. Optimizes it to rotate as least as possible
    front_right.set_target_state(front_right.optimize_state(states[front_right_idx]));
    front_left.set_target_state(front_left.optimize_state(states[front_left_idx]));
    back_left.set_target_state(back_left.optimize_state(states[back_left_idx]));
    back_right.set_target_state(back_right.optimize_state(states[back_right_idx]));
}

auto
swerve_drive::turn_by_angle(units::degree_t angle) noexcept -> frc2::CommandPtr {
    units::degree_t target_angle = get_yaw() + angle;

    frc::SmartDashboard::PutNumber("TARGET", target_angle.value());

    // Build turn command using a PID on the current yaw
    return frc2::cmd::RunOnce([this, target_angle]() {
        set_rotation_motion_source([this, target_angle]() {
            double output         = angle_pid_controller.Calculate(get_yaw().value(), std::move(target_angle).value());
            double clamped_output = std::clamp(std::move(output), k::ctrl::pid_output_min, k::ctrl::pid_output_max);
            return std::move(clamped_output);
        });
    });
}

[[nodiscard]] auto
swerve_drive::align_with(std::function<units::degree_t()> target_cb, std::function<bool()> fallback_condition)
        -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this, target_cb, fallback_condition]() {
        set_rotation_motion_source([this, target_cb, fallback_condition]() {
            if (fallback_condition()) return rotation_motion_fallback_source();

            double output         = align_pid_controller.Calculate(target_cb().value());
            double clamped_output = std::clamp(std::move(output), k::ctrl::pid_output_min, k::ctrl::pid_output_max);
            return std::move(clamped_output);
        });
    });
}

auto
swerve_drive::set_forwards_motion_source(std::function<double()> forwards_motion_source_cb) noexcept -> void {
    this->forwards_motion_source = std::move(forwards_motion_source_cb);
}

auto
swerve_drive::set_sideways_motion_source(std::function<double()> sideways_motion_source_cb) noexcept -> void {
    this->sideways_motion_source = std::move(sideways_motion_source_cb);
}

auto
swerve_drive::set_rotation_motion_source(std::function<double()> rotation_motion_source_cb) noexcept -> void {
    this->rotation_motion_source = std::move(rotation_motion_source_cb);
}

auto
swerve_drive::set_rotation_motion_fallback_source(std::function<double()> rotation_motion_fallback_source_cb) noexcept
        -> void {
    this->rotation_motion_fallback_source = std::move(rotation_motion_fallback_source_cb);
}

auto
swerve_drive::get_pose() noexcept -> frc::Pose2d {
    return odometry.GetPose();
}

auto
swerve_drive::set_pose(frc::Pose2d pose) noexcept -> void {
    odometry.ResetPosition(get_yaw(), get_module_position_array(), std::move(pose));
}

auto
swerve_drive::get_yaw() noexcept -> units::degree_t {
    return gyro.get_z_angle();
}

auto
swerve_drive::reset_yaw() noexcept -> void {
    gyro.reset_z_angle();
}

auto
swerve_drive::get_current_speeds() noexcept -> frc::ChassisSpeeds {
    return kinematics.ToChassisSpeeds(get_module_state_array());
}

auto
swerve_drive::get_module_position_array() noexcept -> wpi::array<frc::SwerveModulePosition, 4> {
    return { front_right.get_module_position(),
             front_left.get_module_position(),
             back_left.get_module_position(),
             back_right.get_module_position() };
}

auto
swerve_drive::get_module_state_array() noexcept -> wpi::array<frc::SwerveModuleState, 4> {
    return { front_right.get_module_state(),
             front_left.get_module_state(),
             back_left.get_module_state(),
             back_right.get_module_state() };
}

} // namespace td::sub::swerve
