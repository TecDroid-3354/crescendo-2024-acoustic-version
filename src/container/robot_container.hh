#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "config/auto/autonomous_config.hh"
#include "constants/climber.hh"
#include "constants/drivetrain/swerve.hh"
#include "constants/indexer.hh"
#include "constants/intake.hh"
#include "constants/numeric.hh"
#include "constants/port.hh"
#include "constants/shooter.hh"
#include "frc2/command/button/Trigger.h"
#include "lib/limelight.hh"
#include "status/robot_mode.hh"
#include "subsystems/climber/climber.hh"
#include "subsystems/indexer/indexer.hh"
#include "subsystems/intake/intake.hh"
#include "subsystems/shooter/shooter.hh"
#include "subsystems/shooter/shooter_positioner.hh"
#include "swerve/swerve_drive.hh"
#include "tejuino/TejuinoBoard.hh"

namespace td {

class robot_container {

public:

    explicit robot_container();

    auto
    robot_init() noexcept -> void;

    auto
    robot_periodic() noexcept -> void;

    auto
    disabled_init() noexcept -> void;

    auto
    disabled_periodic() noexcept -> void;

    auto
    disabled_exit() noexcept -> void;

    auto
    autonomous_init() noexcept -> void;

    auto
    autonomous_periodic() noexcept -> void;

    auto
    autonomous_exit() noexcept -> void;

    auto
    teleop_init() noexcept -> void;

    auto
    teleop_periodic() noexcept -> void;

    auto
    teleop_exit() noexcept -> void;

    auto
    test_init() noexcept -> void;

    auto
    test_periodic() noexcept -> void;

    auto
    test_exit() noexcept -> void;

    [[nodiscard]] auto
    get_autonomous_command() -> frc2::Command *;

    [[nodiscard]] auto
    mode() const noexcept -> status::robot_mode;

    auto
    set_mode(status::robot_mode const &mode) noexcept -> void;

    [[nodiscard]] auto
    toggle_mode() -> frc2::CommandPtr;

    auto
    configure_keybinds() noexcept -> void;

    auto
    configure_providers() noexcept -> void;

    auto
    configure_auto() noexcept -> void;

private:

    status::robot_mode current_mode;
    TejuinoBoard       led_controller;

    frc2::CommandXboxController controller_a { k::port::controller_a };

    sub::swerve::swerve_drive drivetrain { k::dt::swerve::swerve_drive_config };
    auton::autonomous_config  auto_config { &drivetrain };

    sub::shooter_positioner shooter_positioner { k::shooter::position::controller_config_a,
                                                 k::shooter::position::controller_config_b,
                                                 k::shooter::position::encoder_config,
                                                 k::shooter::position::pid_controller_config };
    sub::shooter shooter { k::shooter::spin::controller_config_bottom, k::shooter::spin::controller_config_top };

    sub::intake  intake { k::intake::controller_config };
    sub::indexer indexer { k::indexer::controller_config };

    sub::climber climber { k::climber::left_controller_config,
                           k::climber::right_controller_config,
                           k::climber::encoder_config,
                           k::climber::encoder_config };

    std::function<double()> controlled_forwards_motion_source = [this]() {
        if (controller_a.GetPOV() == 0) return 0.5;
        if (controller_a.GetPOV() == 180) return -0.5;

        return -this->controller_a.GetLeftY();
    };

    std::function<double()> controlled_sideways_motion_source = [this]() {
        if (controller_a.GetPOV() == 90) return -0.5;
        if (controller_a.GetPOV() == 270) return 0.5;
        return -this->controller_a.GetLeftX();
    };

    std::function<double()> controlled_angular_motion_source = [this]() {
        return -this->controller_a.GetRightX();
    };

    std::function<bool()> is_climbing_cb = [this]() {
        return mode() == status::robot_mode::CLIMBING;
    };

    frc::DigitalInput lswitch { k::port::lswitch_dio_pin };
    frc2::Trigger     lswitch_trigger { [this] {
        return this->lswitch.Get();
    } };
};

} // namespace td
