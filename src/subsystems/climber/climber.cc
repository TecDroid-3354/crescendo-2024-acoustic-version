#include "climber.hh"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>

#include "constants/climber.hh"
#include "constants/strings.hh"

namespace td::sub {

climber::climber(
        cfg::spark_max_config const          &left_config,
        cfg::spark_max_config const          &right_config,
        cfg::encoder_output_parameters const &left_encoder_config,
        cfg::encoder_output_parameters const &right_encoder_config)
    : left_controller(left_config.identity.id, left_config.identity.motor_type)
    , right_controller(right_config.identity.id, right_config.identity.motor_type)
    , left_encoder(left_controller.GetEncoder())
    , right_encoder(right_controller.GetEncoder()) {
    cfg::configure_spark_max(&left_controller, left_config);
    cfg::configure_spark_max(&right_controller, right_config);
    cfg::configure_relative_encoder(&left_encoder, left_encoder_config);
    cfg::configure_relative_encoder(&right_encoder, right_encoder_config);

    frc::ShuffleboardTab &climber_tab = frc::Shuffleboard::GetTab(k::str::climber_subsystem_tab);

    climber_tab.AddDouble("Right Encoder Position", [this]() {
        return get_right_hook_position().value();
    });

    climber_tab.AddDouble("Left Encoder Position", [this]() {
        return get_left_hook_position().value();
    });
}

auto
climber::set_percentage(double left_percentage, double right_percentage) -> void {
    this->left_controller.Set(left_percentage);
    this->right_controller.Set(right_percentage);
}

auto
climber::lower_hooks() -> frc2::CommandPtr {
    return frc2::cmd::Run(
            [this]() {
                // Basic bang-bang style limits
                if (get_right_hook_position() > k::climber::bottom_boundary) {
                    right_controller.Set(-k::climber::speed);
                }
                else
                    right_controller.Set(0.0);

                // Basic bang-bang style limits
                if (get_left_hook_position() > k::climber::bottom_boundary) { left_controller.Set(-k::climber::speed); }
                else
                    left_controller.Set(0.0);
            },
            { this });
}

auto
climber::raise_hooks() -> frc2::CommandPtr {
    return frc2::cmd::Run(
            [this]() {
                // Basic bang-bang style limits
                if (get_right_hook_position() < k::climber::top_boundary) { right_controller.Set(k::climber::speed); }
                else
                    right_controller.Set(0.0);

                // Basic bang-bang style limits
                if (get_left_hook_position() < k::climber::top_boundary) { left_controller.Set(k::climber::speed); }
                else
                    left_controller.Set(0.0);
            },
            { this });
}

auto
climber::stop() -> frc2::CommandPtr {
    return frc2::cmd::RunOnce(
            [this]() {
                right_controller.Set(0.0);
                left_controller.Set(0.0);
            },
            { this });
}

auto
climber::get_left_hook_position() -> units::meter_t {
    return units::meter_t { left_encoder.GetPosition() };
}

auto
climber::get_right_hook_position() -> units::meter_t {
    return units::meter_t { right_encoder.GetPosition() };
}

} // namespace td::sub
