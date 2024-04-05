#pragma once

#include <frc2/command/button/CommandXboxController.h>

#include "constants/drive.hh"
#include "constants/numeric.hh"
#include "constants/port.hh"
#include "status/robot_mode.hh"
#include "swerve/swerve_drive.hh"
#include "util/sb_pid_observer.hh"

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
    mode() const noexcept -> status::robot_mode;

    auto
    set_mode(status::robot_mode const &mode) noexcept -> void;

private:

    status::robot_mode current_mode;

    frc2::CommandXboxController controller_a { k::controller_a_port };
    // frc2::CommandXboxController controller_b { k::controller_b_port };

    swerve::swerve_drive drivetrain { k::swerve::swerve_drive_config };

    std::shared_ptr<provider::drivetrain_motion_provider> motion_provider {
        std::make_shared<provider::drivetrain_motion_provider>()
    };
    std::shared_ptr<provider::drivetrain_data_provider> data_provider {
        std::make_shared<provider::drivetrain_data_provider>()
    };

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// TESTING /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // util::sb_pid_observer prop_pid {
    //     "propulsion",
    //     {
    //       drivetrain.expose_front_right_module()->expose_propulsion_pid(),
    //       drivetrain.expose_front_left_module()->expose_propulsion_pid(),
    //       drivetrain.expose_back_left_module()->expose_propulsion_pid(),
    //       drivetrain.expose_back_right_module()->expose_propulsion_pid(),
    //       }
    // };

    // util::sb_pid_observer azim_pid {
    //     "azimuth",
    //     {
    //       drivetrain.expose_front_right_module()->expose_azimuth_pid(),
    //       drivetrain.expose_front_left_module()->expose_azimuth_pid(),
    //       drivetrain.expose_back_left_module()->expose_azimuth_pid(),
    //       drivetrain.expose_back_right_module()->expose_azimuth_pid(),
    //       }
    // };
};

} // namespace td
