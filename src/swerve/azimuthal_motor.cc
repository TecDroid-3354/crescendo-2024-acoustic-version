#include "azimuthal_motor.hh"

#include <fmt/format.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

namespace td::swerve {

azimuthal_motor::azimuthal_motor(
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
azimuthal_motor::update() noexcept -> void {
    pid_controller.SetSetpoint(get_target_angle().value());
    double const controller_output = pid_controller.Calculate(get_current_angle().value());
    double const clamped_output    = std::clamp(controller_output, -1.0, 1.0);

    controller.Set(clamped_output);
}

auto
azimuthal_motor::set_current_angle(units::degree_t angle) noexcept -> void {
    encoder.SetPosition(angle.value());
}

auto
azimuthal_motor::set_target_angle(units::degree_t angle) noexcept -> void {
    target_angle = angle;
}

auto
azimuthal_motor::get_current_angle() const noexcept -> units::degree_t {
    return units::degree_t { encoder.GetPosition() };
}

auto
azimuthal_motor::get_target_angle() const noexcept -> units::degree_t {
    return target_angle;
}

auto
azimuthal_motor::expose_pid_controller() noexcept -> frc::PIDController & {
    return pid_controller;
}

auto
azimuthal_motor::log() const noexcept -> void {
    frc::SmartDashboard::PutNumber(
            fmt::format("[{}] Azimuthal Angle: Target", controller.GetDeviceId()),
            get_target_angle().value());

    frc::SmartDashboard::PutNumber(
            fmt::format("[{}] Azimuthal Angle: Current", controller.GetDeviceId()),
            get_current_angle().value());
}

} // namespace td::swerve
