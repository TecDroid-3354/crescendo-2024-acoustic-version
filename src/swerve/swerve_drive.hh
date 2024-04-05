#pragma once

#include <cstdint>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/SubsystemBase.h>

#include "module.hh"
#include "providers/drivetrain_data_provider.hh"
#include "providers/drivetrain_motion_provider.hh"
#include "status/error_code.hh"

namespace td::swerve {

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

    explicit swerve_drive(config::swerve_drive const &config);

    auto
    init() noexcept -> void;

    auto
    Periodic() -> void override;

    auto
    engage_motion_provider(
            std::shared_ptr<provider::drivetrain_motion_provider> motion_provider,
            std::shared_ptr<provider::drivetrain_data_provider>   data_provider) noexcept -> status::error_code;

    auto
    drive(frc::ChassisSpeeds const &speed) noexcept -> void;

    auto
    expose_front_right_module() noexcept -> individual_module *;

    auto
    expose_front_left_module() noexcept -> individual_module *;

    auto
    expose_back_left_module() noexcept -> individual_module *;

    auto
    expose_back_right_module() noexcept -> individual_module *;

private:

    individual_module front_right;
    individual_module front_left;
    individual_module back_left;
    individual_module back_right;

    frc::SwerveDriveKinematics<4> kinematics;

    std::shared_ptr<provider::drivetrain_motion_provider> motion_provider;
    std::shared_ptr<provider::drivetrain_data_provider>   data_provider;

    swerve_mode current_mode;
};

} // namespace td::swerve
