#include "swerve_drive.hh"

#include <frc2/command/Commands.h>

#include "constants/drive.hh"

namespace td::swerve {

swerve_drive::swerve_drive(config::swerve_drive const &config)
    : front_right { config.front_right }
    , front_left { config.front_left }
    , back_left { config.back_left }
    , back_right { config.back_right }
    , kinematics { wpi::array<frc::Translation2d, 4> { front_right.center_offset(), front_left.center_offset(), back_left.center_offset(), back_right.center_offset() } }
    , motion_provider { nullptr }
    , data_provider { nullptr }
    , current_mode { swerve_mode::FIELD_ORIENTED }

{ }

auto
swerve_drive::init() noexcept -> void { }

auto
swerve_drive::Periodic() -> void {
    front_right.update();
    front_left.update();
    back_left.update();
    back_right.update();
}

auto
swerve_drive::engage_motion_provider(
        std::shared_ptr<provider::drivetrain_motion_provider> motion_provider,
        std::shared_ptr<provider::drivetrain_data_provider>   data_provider) noexcept -> status::error_code {
    if (motion_provider.get() == nullptr || data_provider.get() == nullptr) { return status::error_code::NULL_POINTER; }

    this->motion_provider = motion_provider;
    this->data_provider   = data_provider;

    this->SetDefaultCommand(frc2::cmd::Run(
            [this]() {
                double normalized_forwards =
                        std::clamp(this->motion_provider->get_forwards_velocity_output(), -1.0, 1.0);

                double normalized_sideways =
                        std::clamp(this->motion_provider->get_sideways_velocity_output(), -1.0, 1.0);

                double normalized_angular = std::clamp(this->motion_provider->get_angular_velocity_output(), -1.0, 1.0);

                frc::ChassisSpeeds chassis_speeds;

                switch (current_mode) {
                case swerve_mode::ROBOT_ORIENTED :
                    chassis_speeds = { k::swerve::target_linear_velocity * normalized_forwards,
                                       k::swerve::target_linear_velocity * normalized_sideways,
                                       k::swerve::target_angular_velocity * normalized_angular };

                    break;
                case swerve_mode::FIELD_ORIENTED :
                    chassis_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            k::swerve::target_linear_velocity * normalized_forwards,
                            k::swerve::target_linear_velocity * normalized_sideways,
                            k::swerve::target_angular_velocity * normalized_angular,
                            this->data_provider->get_current_angle_output());
                    break;
                }

                this->drive(chassis_speeds);
            },
            { this }));

    return status::error_code::NONE;
}

auto
swerve_drive::drive(frc::ChassisSpeeds const &speed) noexcept -> void {
    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(speed);
    front_right.set_target_state(front_right.optimize_state(states[front_right_idx]));
    front_left.set_target_state(front_left.optimize_state(states[front_left_idx]));
    back_left.set_target_state(back_left.optimize_state(states[back_left_idx]));
    back_right.set_target_state(back_right.optimize_state(states[back_right_idx]));
}

auto
swerve_drive::expose_front_right_module() noexcept -> individual_module * {
    return &front_right;
}

auto
swerve_drive::expose_front_left_module() noexcept -> individual_module * {
    return &front_left;
}

auto
swerve_drive::expose_back_left_module() noexcept -> individual_module * {
    return &back_left;
}

auto
swerve_drive::expose_back_right_module() noexcept -> individual_module * {
    return &back_right;
}

} // namespace td::swerve
