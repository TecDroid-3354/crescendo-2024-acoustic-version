#include "robot_container.hh"

#include <frc2/command/Commands.h>

#include <memory>

#include "constants/numeric.hh"

namespace td {

robot_container::robot_container() { }

auto
robot_container::robot_init() noexcept -> void {
    drivetrain.init();
}

auto
robot_container::robot_periodic() noexcept -> void { }

auto
robot_container::disabled_init() noexcept -> void { }

auto
robot_container::disabled_periodic() noexcept -> void { }

auto
robot_container::disabled_exit() noexcept -> void { }

auto
robot_container::autonomous_init() noexcept -> void { }

auto
robot_container::autonomous_periodic() noexcept -> void { }

auto
robot_container::autonomous_exit() noexcept -> void { }

auto
robot_container::teleop_init() noexcept -> void {
    motion_provider->set_forwards_velocity_provider([this]() {
        return -this->controller_a.GetLeftY();
    });

    motion_provider->set_sideways_velocity_provider([this]() {
        return -this->controller_a.GetLeftX();
    });

    motion_provider->set_angular_velocity_provider([this]() {
        return -this->controller_a.GetRightX();
    });

    data_provider->set_current_angle_provider([this]() {
        return 0.0_deg;
    });

    this->drivetrain.engage_motion_provider(motion_provider, data_provider);
}

auto
robot_container::teleop_periodic() noexcept -> void {
    // azim_pid.update();
    // prop_pid.update();
}

auto
robot_container::teleop_exit() noexcept -> void { }

auto
robot_container::test_init() noexcept -> void { }

auto
robot_container::test_periodic() noexcept -> void { }

auto
robot_container::test_exit() noexcept -> void { }

auto
robot_container::mode() const noexcept -> status::robot_mode {
    return this->current_mode;
}

auto
robot_container::set_mode(status::robot_mode const &mode) noexcept -> void {
    this->current_mode = mode;
}

} // namespace td
