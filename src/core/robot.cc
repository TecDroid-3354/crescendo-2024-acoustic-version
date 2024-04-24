#include "robot.hh"

#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

namespace td {

robot::robot() = default;

auto
robot::RobotInit() -> void {
    container.robot_init();
}

auto
robot::RobotPeriodic() -> void {
    frc2::CommandScheduler::GetInstance().Run();
    container.robot_periodic();
}

auto
robot::DisabledInit() -> void {
    container.disabled_init();
}

auto
robot::DisabledPeriodic() -> void {
    container.disabled_periodic();
}

auto
robot::DisabledExit() -> void {
    container.disabled_exit();
}

auto
robot::AutonomousInit() -> void {
    container.autonomous_init();

    auto_cmd = container.get_autonomous_command();
    if (auto_cmd == nullptr) return;

    auto_cmd->Schedule();

}

auto
robot::AutonomousPeriodic() -> void {
    container.autonomous_periodic();
}

auto
robot::AutonomousExit() -> void {
    container.autonomous_exit();
}

auto
robot::TeleopInit() -> void {
    if (auto_cmd != nullptr && auto_cmd->IsScheduled()) {
        auto_cmd->Cancel();
    }

    container.teleop_init();
}

auto
robot::TeleopPeriodic() -> void {
    container.teleop_periodic();
}

auto
robot::TeleopExit() -> void {
    container.teleop_exit();
}

auto
robot::TestInit() -> void {
    container.test_init();
}

auto
robot::TestPeriodic() -> void {
    container.test_periodic();
}

auto
robot::TestExit() -> void {
    container.test_exit();
}
} // namespace td
