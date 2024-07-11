#include "propulsion_motor.hh"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "constants/drivetrain/propulsion.hh"
#include "constants/strings.hh"
#include "constants/widgets/swerve.hh"

namespace td::sub::swerve {

propulsion_motor::propulsion_motor(
        cfg::spark_max_config          controller_config,
        cfg::encoder_output_parameters encoder_config,
        cfg::spark_pid_config          pid_controller_config)
    : controller { controller_config.identity.id, controller_config.identity.motor_type }
    , encoder { controller.GetEncoder() }
    , pid_controller { controller.GetPIDController() } {
    cfg::configure_spark_max(&controller, controller_config);
    cfg::configure_relative_encoder(&encoder, encoder_config);
    cfg::configure_spark_pid(&pid_controller, pid_controller_config);

    // Data logging
    frc::ShuffleboardTab &swerve_drive_logs = frc::Shuffleboard::GetTab(k::str::swerve_subsystem_tab);

    swerve_drive_logs
            .AddNumber(
                    fmt::format("[{}] Propulsion Velocity: Current", controller.GetDeviceId()),
                    [this]() {
                        return get_current_velocity().value();
    })
            .WithWidget(frc::BuiltInWidgets::kNumberBar)
            .WithProperties({
                    { "min", nt::Value::MakeDouble(k::dt::prop::min_linear_velocity.value()) },
                    { "max", nt::Value::MakeDouble(k::dt::prop::max_linear_velocity.value()) },
            });

    swerve_drive_logs
            .AddNumber(
                    fmt::format("[{}] Propulsion Velocity: Target", controller.GetDeviceId()),
                    [this]() {
                        return get_target_velocity().value();
    })
            .WithWidget(frc::BuiltInWidgets::kNumberBar)
            .WithProperties({
                    { "min", nt::Value::MakeDouble(k::dt::prop::min_linear_velocity.value()) },
                    { "max", nt::Value::MakeDouble(k::dt::prop::max_linear_velocity.value()) },
            });
}

auto
propulsion_motor::update() noexcept -> void {
    // Set the PIDF target, have the API do its thing
    pid_controller.SetReference(get_target_velocity().value(), rev::CANSparkLowLevel::ControlType::kVelocity);
}

auto
propulsion_motor::set_target_velocity(units::meters_per_second_t const &velocity) noexcept -> void {
    target_velocity = std::move(velocity);
}

auto
propulsion_motor::get_current_velocity() const noexcept -> units::meters_per_second_t {
    return units::meters_per_second_t { encoder.GetVelocity() };
}

auto
propulsion_motor::get_target_velocity() const noexcept -> units::meters_per_second_t {
    return target_velocity;
}

auto
propulsion_motor::get_travelled_distance() const noexcept -> units::meter_t {
    // Inverted because of planar conventions (TODO: Evaluate the effect when motor movement is backwards)
    return units::meter_t { -encoder.GetPosition() };
}

} // namespace td::sub::swerve
