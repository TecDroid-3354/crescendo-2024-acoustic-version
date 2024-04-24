#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "container/robot_container.hh"

namespace td {

/**
 * Robot Class
 * Contains the robot's main functionality
 *
 * This class should not be modified in order to preserve core functionality.
 * That's what robot_container is for.
 */
class robot: public frc::TimedRobot {
public:

    explicit robot();

    auto
    RobotInit() -> void override;

    auto
    RobotPeriodic() -> void override;

    auto
    DisabledInit() -> void override;

    auto
    DisabledPeriodic() -> void override;

    auto
    DisabledExit() -> void override;

    auto
    AutonomousInit() -> void override;

    auto
    AutonomousPeriodic() -> void override;

    auto
    AutonomousExit() -> void override;

    auto
    TeleopInit() -> void override;

    auto
    TeleopPeriodic() -> void override;

    auto
    TeleopExit() -> void override;

    auto
    TestInit() -> void override;

    auto
    TestPeriodic() -> void override;

    auto
    TestExit() -> void override;

private:

    robot_container container;
    frc2::Command* auto_cmd;
};
} // namespace td
