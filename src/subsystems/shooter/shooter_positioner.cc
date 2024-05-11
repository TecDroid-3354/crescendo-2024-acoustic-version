#include "shooter_positioner.hh"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>

#include "constants/control.hh"
#include "constants/shooter.hh"
#include "constants/strings.hh"

namespace td::sub {

shooter_positioner::shooter_positioner(
        cfg::spark_max_config const          &controller_config_a,
        cfg::spark_max_config const          &controller_config_b,
        cfg::encoder_output_parameters const &encoder_config,
        cfg::pid_config const                &pid_controller_config)
    : controller_a { controller_config_a.identity.id, controller_config_a.identity.motor_type }
    , controller_b { controller_config_b.identity.id, controller_config_b.identity.motor_type }
    , pid_controller { 0.0, 0.0, 0.0 }
    , encoder { 0 }
    , target_angle_source { []() {
        return 0.0_deg;
    } } {
    cfg::configure_spark_max(&controller_a, controller_config_a);
    cfg::configure_spark_max(&controller_b, controller_config_b);

    cfg::configure_pid_controller(&pid_controller, pid_controller_config);

    controller_b.Follow(controller_a, true);

    frc::ShuffleboardTab &shooter_log_tab = frc::Shuffleboard::GetTab(k::str::shooter_subsystem_tab);

    shooter_log_tab.Add("Target PID", pid_controller);

    shooter_log_tab.AddDouble("Absolute Angle", [this]() {
        return this->get_current_angle().value();
    });

    shooter_log_tab.AddDouble("Target Angle", [this]() {
        return this->get_target_angle().value();
    });
}

auto
shooter_positioner::enable_target_tracking() noexcept -> frc2::CommandPtr {
    return md_coast().AndThen(frc2::cmd::Run(
            [this]() {
                units::degree_t target_angle = this->fixed_target;

                if (!this->with_fixed_target) { target_angle = get_target_angle(); }

                double clamped_target_angle = std::clamp(
                        std::move(target_angle).value(),
                        k::shooter::position::min_possible_angle.value(),
                        k::shooter::position::max_possible_angle.value());

                double target_output =
                        pid_controller.Calculate(get_current_angle().value(), std::move(clamped_target_angle));

                double modded_output =
                        std::clamp(std::move(target_output), k::ctrl::pid_output_min, k::ctrl::pid_output_max);

                modded_output *= k::shooter::position::speed_factor;

                if (modded_output < 0 && get_current_angle() > k::shooter::position::max_possible_angle) {
                    controller_a.Set(0.0);
                    return;
                }
                if (modded_output > 0 && get_current_angle() < k::shooter::position::min_possible_angle) {
                    controller_a.Set(0.0);
                    return;
                }

                controller_a.Set(std::move(modded_output) * 0.25);
            },
            { this }));
}

auto
shooter_positioner::disable_target_tracking() noexcept -> frc2::CommandPtr {
    return md_break().AndThen(frc2::cmd::RunOnce(
            [this]() {
                controller_a.Set(0.0);
            },
            { this }));
}

auto
shooter_positioner::set_target_angle_source(std::function<units::degree_t()> function) noexcept -> void {
    this->target_angle_source = std::move(function);
}

auto
shooter_positioner::enable_fixed_targetting() noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this]() {
                this->with_fixed_target = true;
            },
            { this });
}

auto
shooter_positioner::disable_fixed_targetting() noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this]() {
                this->with_fixed_target = false;
            },
            { this });
}

auto
shooter_positioner::set_fixed_target(units::degree_t angle) noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this, angle]() {
                this->fixed_target = std::move(angle);
            },
            { this });
}

auto
shooter_positioner::get_target_angle() const noexcept -> units::degree_t {
    return target_angle_source();
}

auto
shooter_positioner::get_current_angle() const noexcept -> units::degree_t {
    return units::degree_t { -(
            units::degree_t { units::turn_t { encoder.GetAbsolutePosition() } - k::shooter::position::zero_offset }
                    .value()) };
}

auto
shooter_positioner::set_percentage(double percenatge) -> void {
    this->controller_a.Set(percenatge);
}

auto
shooter_positioner::md_coast() -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this]() {
                controller_a.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
                controller_b.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
            },
            { this });
}

auto
shooter_positioner::md_break() -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this]() {
                controller_a.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
                controller_b.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
            },
            { this });
}

auto
shooter_positioner::stop() -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this]() {
        controller_a.StopMotor();
    });
}

} // namespace td::sub
