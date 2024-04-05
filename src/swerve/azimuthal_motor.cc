#include "azimuthal_motor.hh"

#include <fmt/format.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>

#include "constants/strings.hh"

namespace td::swerve {

azimuthal_motor::azimuthal_motor(
        config::spark_max      spark_max_config,
        config::cancoder       encoder_config,
        config::pid_controller pid_controller_config)
    : controller { spark_max_config.id, spark_max_config.motor_type }
    , encoder { encoder_config.id }
    , pid_controller { mkpid_controller(pid_controller_config) } {
    configure_spark(controller, spark_max_config);

    frc::ShuffleboardTab &dt_logs = frc::Shuffleboard::GetTab(k::str::DRIVETRAIN_LOGS_TAB);

    dt_logs.AddNumber(
                   fmt::format("[{}] Azimuthal Angle: Target", controller.GetDeviceId()),
                   [this]() {
                       return get_target_angle().value();
    })
            .WithWidget(frc::BuiltInWidgets::kDial)
            .WithProperties({ { "min", nt::Value::MakeDouble(-180.0) }, { "max", nt::Value::MakeDouble(-180.0) } });

    dt_logs.AddNumber(
                   fmt::format("[{}] Azimuthal Angle: Current", controller.GetDeviceId()),
                   [this]() {
                       return get_current_angle().value();
    })
            .WithWidget(frc::BuiltInWidgets::kDial)
            .WithProperties({ { "min", nt::Value::MakeDouble(-180.0) }, { "max", nt::Value::MakeDouble(-180.0) } });
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
    encoder.SetPosition(angle);
}

auto
azimuthal_motor::set_target_angle(units::degree_t angle) noexcept -> void {
    target_angle = angle;
}

auto
azimuthal_motor::get_current_angle() noexcept -> units::degree_t {
    return encoder.GetAbsolutePosition().GetValue();
}

auto
azimuthal_motor::get_target_angle() const noexcept -> units::degree_t {
    return target_angle;
}

auto
azimuthal_motor::expose_pid_controller() noexcept -> frc::PIDController * {
    return &pid_controller;
}

} // namespace td::swerve
