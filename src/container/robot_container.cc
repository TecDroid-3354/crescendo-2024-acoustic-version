#include "robot_container.hh"

#include <memory>

#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/PIDCommand.h>

#include <pathplanner/lib/auto/NamedCommands.h>

#include "constants/drivetrain/thresholds.hh"
#include "constants/numeric.hh"
#include "constants/strings.hh"
#include "lib/limelight.hh"

namespace td {

robot_container::robot_container() { }

auto
robot_container::robot_init() noexcept -> void {
    // init limelight
    ll::init();

    frc::ShuffleboardTab &dt_tab = frc::Shuffleboard::GetTab(k::str::drive_team_tab);

    dt_tab.AddBoolean("Climbing", is_climbing_cb);

    configure_keybinds();
    configure_providers();
    configure_auto();
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
robot_container::teleop_init() noexcept -> void { }

auto
robot_container::teleop_periodic() noexcept -> void { }

auto
robot_container::teleop_exit() noexcept -> void { }

auto
robot_container::test_init() noexcept -> void { }

auto
robot_container::test_periodic() noexcept -> void { }

auto
robot_container::test_exit() noexcept -> void { }

auto
robot_container::get_autonomous_command() -> frc2::Command * {
    return auto_config.get_selected_auto();
}

auto
robot_container::mode() const noexcept -> status::robot_mode {
    return this->current_mode;
}

auto
robot_container::set_mode(status::robot_mode const &mode) noexcept -> void {
    this->current_mode = mode;
}

auto
robot_container::toggle_mode() -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this]() {
        switch (mode()) {
        case status::robot_mode::SHOOTING : {
            this->set_mode(status::robot_mode::CLIMBING);
        } break;
        case status::robot_mode::CLIMBING : {
            this->set_mode(status::robot_mode::SHOOTING);
        } break;
        }
    });
}

auto
robot_container::configure_keybinds() noexcept -> void {
    // >> Drivetrain << //

    // Reset heading
    controller_a.Start().OnTrue(frc2::cmd::RunOnce([this]() {
        drivetrain.reset_yaw();
    }));

    // Rotate 180
    controller_a.RightStick().ToggleOnTrue(drivetrain.turn_by_angle(180_deg));

    // >> Shooter position << //

    // Enable targetting
    controller_a.RightTrigger(k::shooter::position::activation_trigger_threshold)
            .OnTrue(frc2::cmd::Sequence(
                    shooter_positioner.disable_fixed_targetting(),
                    shooter_positioner.enable_target_tracking()))
            .OnFalse(frc2::cmd::Sequence(
                    shooter_positioner.disable_fixed_targetting(),
                    shooter_positioner.disable_target_tracking()));

    // Set fixed amp target
    controller_a.Y()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    frc2::cmd::Sequence(
                            shooter_positioner.set_fixed_target(k::shooter::position::amp_target_angle),
                            shooter_positioner.enable_fixed_targetting(),
                            shooter_positioner.enable_target_tracking()),
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Sequence(
                    shooter_positioner.disable_target_tracking(),
                    shooter_positioner.stop(),
                    shooter_positioner.disable_fixed_targetting()));

    // Set fixed hi-pass angle
    controller_a.B()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    frc2::cmd::Sequence(
                            shooter_positioner.set_fixed_target(k::shooter::position::pass_target_angle),
                            shooter_positioner.enable_fixed_targetting(),
                            shooter_positioner.enable_target_tracking()),
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Sequence(
                    shooter_positioner.disable_target_tracking(),
                    shooter_positioner.stop(),
                    shooter_positioner.disable_fixed_targetting()));

    // Set fixed home angle
    controller_a.A()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    frc2::cmd::Sequence(
                            shooter_positioner.set_fixed_target(k::shooter::position::home_angle),
                            shooter_positioner.enable_fixed_targetting(),
                            shooter_positioner.enable_target_tracking())

                            ,
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Sequence(
                    shooter_positioner.disable_target_tracking(),
                    shooter_positioner.stop(),
                    shooter_positioner.disable_fixed_targetting()));

    // Set fixed lo-pass angle
    controller_a.X()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    frc2::cmd::Sequence(
                            shooter_positioner.set_fixed_target(k::shooter::position::lowpass_target_angle),
                            shooter_positioner.enable_fixed_targetting(),
                            shooter_positioner.enable_target_tracking())

                            ,
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Sequence(
                    shooter_positioner.disable_target_tracking(),
                    shooter_positioner.stop(),
                    shooter_positioner.disable_fixed_targetting()));

    // >> Shooter Spin << //

    // Set speeds to shoot at the speaker
    controller_a.RightTrigger(k::shooter::spin::activation_trigger_threshold)
            .OnTrue(shooter.set_velocity(
                                   k::shooter::spin::speaker_target_percentage,
                                   k::shooter::spin::speaker_target_percentage)
                            .AlongWith(frc2::cmd::Parallel()))
            .OnFalse(shooter.stop());

    // Set speeds to shoot at the amp
    controller_a.Y()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    shooter.set_velocity(
                            k::shooter::spin::amp_bottom_target_percentage,
                            k::shooter::spin::amp_top_target_percentage),
                    is_climbing_cb))
            .OnFalse(shooter.stop());

    // Set speeds to hi-pass
    controller_a.B()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    shooter.set_velocity(k::shooter::spin::pass_percentage, k::shooter::spin::pass_percentage),
                    is_climbing_cb))
            .OnFalse(shooter.stop());

    // Set speeds to lo-pass
    controller_a.X()
            .OnTrue(frc2::cmd::Either(
                    frc2::cmd::None(),
                    shooter.set_velocity(k::shooter::spin::pass_percentage, k::shooter::spin::pass_percentage),
                    is_climbing_cb))
            .OnFalse(shooter.stop());

    // >> Drivetrain << //

    // Begin apriltag targetting when trigger passes threshold. Hate this method.
    controller_a.RightTrigger(k::shooter::position::activation_trigger_threshold)
            .OnTrue(drivetrain.align_with(ll::get_horizontal_angle_to_target, ll::has_no_targets))
            .OnFalse(frc2::cmd::RunOnce([this]() {
                drivetrain.set_rotation_motion_source(controlled_angular_motion_source);
            }));

    // >> Intake, Indexer & Hooks << //

    // lswitch.OnTrue(intake.stop().AlongWith(indexer.stop()));

    // Intake in
    controller_a.RightBumper()
            .OnTrue(frc2::cmd::Either(
                    climber.lower_hooks(),
                    frc2::cmd::Parallel(
                            intake.set_percentage(k::intake::target_percentage),
                            indexer.set_percentage(k::indexer::target_percentage)),
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Parallel(climber.stop(), intake.stop(), indexer.stop()));

    // Intake out
    controller_a.LeftBumper()
            .OnTrue(frc2::cmd::Either(
                    climber.raise_hooks(),
                    frc2::cmd::Parallel(
                            intake.set_percentage(-k::intake::target_percentage),
                            indexer.set_percentage(-k::indexer::target_percentage)),
                    is_climbing_cb))
            .OnFalse(frc2::cmd::Parallel(climber.stop(), intake.stop(), indexer.stop()));

    // Toggle shooting & climbing
    controller_a.Back().OnTrue(toggle_mode());

    lswitch_trigger.OnTrue(frc2::cmd::Parallel(intake.stop(), indexer.stop()));
}

auto
robot_container::configure_providers() noexcept -> void {
    /// Have the drivetrain move according to callbacks from the controller
    drivetrain.set_forwards_motion_source(controlled_forwards_motion_source);
    drivetrain.set_sideways_motion_source(controlled_sideways_motion_source);
    drivetrain.set_rotation_motion_source(controlled_angular_motion_source);
    drivetrain.set_rotation_motion_fallback_source(controlled_angular_motion_source);

    // Function to calculate target angle. Hate this method.
    shooter_positioner.set_target_angle_source([this]() {
        if (ll::has_no_targets()) { return shooter_positioner.get_current_angle(); }

        units::meter_t target_height = ll::get_height_of_target_type();
        units::meter_t diff          = target_height - ll::mount_height;
        units::meter_t dtt           = ll::get_distance_to_target();

        if (dtt == 0.0_m) { return 0.0_deg; }

        // units::degree_t is ideally not necessary but I am paranoid. Has no runtime penalty
        return units::degree_t { units::radian_t { std::atan2(diff.value(), dtt.value()) } }
        - k::shooter::position::angle_offset;
    });
}

auto
robot_container::configure_auto() noexcept -> void {
    // Pathplanner commands
    pathplanner::NamedCommands::registerCommand(
            "intake",
            frc2::cmd::Parallel(
                    intake.set_percentage(k::intake::target_percentage),
                    indexer.set_percentage(k::indexer::target_percentage))
                    .WithTimeout(1.5_s)
                    .AndThen(frc2::cmd::Parallel(intake.stop(), indexer.stop())));

    // Command sequence to shoot
    pathplanner::NamedCommands::registerCommand(
            "shoot",
            frc2::cmd::Parallel(
                    drivetrain.align_with(ll::get_horizontal_angle_to_target, ll::has_no_targets)
                            .AndThen(frc2::cmd::Wait(2_s))
                            .AndThen(frc2::cmd::RunOnce([this]() {
                                this->drivetrain.set_rotation_motion_source(controlled_angular_motion_source);
                            })),

                    shooter.set_velocity(
                                   k::shooter::spin::speaker_target_percentage,
                                   k::shooter::spin::speaker_target_percentage)
                            .AndThen(frc2::cmd::Wait(2_s))
                            .AndThen(shooter.stop()),

                    frc2::cmd::Sequence(
                            shooter_positioner.disable_fixed_targetting(),
                            shooter_positioner.enable_target_tracking())
                            .AndThen(frc2::cmd::Wait(2_s))
                            .AndThen(shooter_positioner.disable_target_tracking()),

                    frc2::cmd::Wait(1.25_s)
                            .AndThen(frc2::cmd::Parallel(
                                    intake.set_percentage(k::intake::target_percentage),
                                    indexer.set_percentage(k::indexer::target_percentage)))
                            .AndThen(frc2::cmd::Wait(0.75_s))
                            .AndThen(frc2::cmd::Parallel(intake.stop(), indexer.stop())))
                    .WithTimeout(2.25_s));

    // begin limelight allignment
    pathplanner::NamedCommands::registerCommand(
            "drivetrain.llalign",
            drivetrain.align_with(ll::get_horizontal_angle_to_target, ll::has_no_targets));

    // stop limelight allignment
    pathplanner::NamedCommands::registerCommand("drivetrain.llstop", frc2::cmd::RunOnce([this]() {
                                                    this->drivetrain.set_rotation_motion_source(
                                                            controlled_angular_motion_source);
                                                }));

    auto_config.load_existing_commands();
}

} // namespace td
