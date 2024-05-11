#include "azimuthal_motor.hh"

#include <fmt/format.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "constants/control.hh"
#include "constants/drivetrain/azimuthal.hh"
#include "constants/strings.hh"
#include "constants/widgets/swerve.hh"

namespace td::sub::swerve {

azimuthal_motor::azimuthal_motor(
        cfg::spark_max_config spark_max_config,
        cfg::cancoder         cancoder_config,
        cfg::pid_config       pid_config)
    : controller { spark_max_config.identity.id, spark_max_config.identity.motor_type }
    , encoder { cancoder_config.id }
    , pid_controller { 0.0, 0.0, 0.0 } {
    cfg::configure_spark_max(&controller, spark_max_config);
    cfg::configure_pid_controller(&pid_controller, pid_config);

    // Data logging
    frc::ShuffleboardTab &swerve_drive_logs = frc::Shuffleboard::GetTab(k::str::swerve_subsystem_tab);
    frc::ShuffleboardTab &swerve_pid_logs   = frc::Shuffleboard::GetTab(k::str::swerve_pid_manipulation_tab);

    swerve_pid_logs.Add(fmt::format("[{}] azimuthal pid", controller.GetDeviceId()), pid_controller);

    swerve_drive_logs
            .AddNumber(
                    fmt::format("[{}] Azimuthal Angle: Current", controller.GetDeviceId()),
                    [this]() {
                        return get_current_angle().value();
    })
            .WithWidget(frc::BuiltInWidgets::kDial)
            .WithProperties({ { "min", nt::Value::MakeDouble(k::dt::azim::module_angle_min.value()) },
                              { "max", nt::Value::MakeDouble(k::dt::azim::module_angle_max.value()) } });

    swerve_drive_logs
            .AddNumber(
                    fmt::format("[{}] Azimuthal Angle: Target", controller.GetDeviceId()),
                    [this]() {
                        return get_target_angle().value();
    })
            .WithWidget(frc::BuiltInWidgets::kDial)
            .WithProperties({ { "min", nt::Value::MakeDouble(k::dt::azim::module_angle_min.value()) },
                              { "max", nt::Value::MakeDouble(k::dt::azim::module_angle_max.value()) } });
}

auto
azimuthal_motor::update() noexcept -> void {
    // Calculate the percentage at which the motor should be set to reach
    // The target angle
    double const target_angle      = get_target_angle().value();
    double const current_angle     = get_current_angle().value();
    double const controller_output = pid_controller.Calculate(std::move(current_angle), std::move(target_angle));

    double const clamped_output =
            std::clamp(std::move(controller_output), k::ctrl::pid_output_min, k::ctrl::pid_output_max);

    controller.Set(-std::move(clamped_output));
}

auto
azimuthal_motor::set_current_angle(units::degree_t angle) noexcept -> void {
    encoder.SetPosition(std::move(angle));
}

auto
azimuthal_motor::set_target_angle(units::degree_t angle) noexcept -> void {
    target_angle = std::move(angle);
}

auto
azimuthal_motor::get_current_angle() noexcept -> units::degree_t {
    return encoder.GetAbsolutePosition().GetValue();
}

auto
azimuthal_motor::get_target_angle() const noexcept -> units::degree_t {
    return target_angle;
}

} // namespace td::sub::swerve
