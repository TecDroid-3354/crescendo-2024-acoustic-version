#include "propulsion_motor.hh"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

namespace td::swerve {

propulsion_motor::propulsion_motor(
        config::spark_max      spark_max_config,
        config::neo_encoder    neo_encoder_config,
        config::pid_controller pid_controller_config)
    : controller { spark_max_config.id, spark_max_config.motor_type }
    , encoder { controller.GetEncoder() }
    , pid_controller { mkpid_controller(pid_controller_config) } {
    configure_spark(controller, spark_max_config);
    configure_neo_encoder(encoder, neo_encoder_config);
}

auto
propulsion_motor::update() noexcept -> void {
    pid_controller.SetSetpoint(get_target_velocity().value());
    double const controller_output = pid_controller.Calculate(get_current_velocity().value());
    double const clamped_output    = std::clamp(controller_output, -1.0, 1.0);

    controller.Set(clamped_output);
}

auto
propulsion_motor::set_target_velocity(units::meters_per_second_t const &velocity) noexcept -> void {
    target_velocity = velocity;
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
propulsion_motor::expose_pid_controller() noexcept -> frc::PIDController & {
    return pid_controller;
}

auto
propulsion_motor::log() const noexcept -> void {
    frc::SmartDashboard::PutNumber(
            fmt::format("[{}] Propulsion Velocity: Target", controller.GetDeviceId()),
            get_target_velocity().value());

    frc::SmartDashboard::PutNumber(
            fmt::format("[{}] Propulsion Velocity: Current", controller.GetDeviceId()),
            get_current_velocity().value());
}

} // namespace td::swerve
