#include "robot.hh"

#include <frc2/command/CommandScheduler.h>

namespace td {

robot::robot() = default;

auto
robot::RobotInit() -> void {
    _container.robot_init();
}

auto
robot::RobotPeriodic() -> void {
    frc2::CommandScheduler::GetInstance().Run();
    _container.robot_periodic();
}

auto
robot::DisabledInit() -> void {
    _container.disabled_init();
}

auto
robot::DisabledPeriodic() -> void {
    _container.disabled_periodic();
}

auto
robot::DisabledExit() -> void {
    _container.disabled_exit();
}

auto
robot::AutonomousInit() -> void {
    _container.autonomous_init();
}

auto
robot::AutonomousPeriodic() -> void {
    _container.autonomous_periodic();
}

auto
robot::AutonomousExit() -> void {
    _container.autonomous_exit();
}

auto
robot::TeleopInit() -> void {
    _container.teleop_init();
}

auto
robot::TeleopPeriodic() -> void {
    _container.teleop_periodic();
}

auto
robot::TeleopExit() -> void {
    _container.teleop_exit();
}

auto
robot::TestInit() -> void {
    _container.test_init();
}

auto
robot::TestPeriodic() -> void {
    _container.test_periodic();
}

auto
robot::TestExit() -> void {
    _container.test_exit();
}
} // namespace td
