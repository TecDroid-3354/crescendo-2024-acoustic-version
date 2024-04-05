#include "drivetrain_motion_provider.hh"

#include <frc/shuffleboard/Shuffleboard.h>

#include "constants/strings.hh"

namespace td::provider {

drivetrain_motion_provider::drivetrain_motion_provider() {
    frc::ShuffleboardTab &dt_logs = frc::Shuffleboard::GetTab(k::str::DRIVETRAIN_LOGS_TAB);

    dt_logs.AddNumber("[DTP] Normalized forwards output", [this]() {
        return get_forwards_velocity_output();
    });

    dt_logs.AddNumber("[DTP] Normalized sideways output", [this]() {
        return get_sideways_velocity_output();
    });

    dt_logs.AddNumber("[DTP] Normalized angular output", [this]() {
        return get_angular_velocity_output();
    });
}

auto
drivetrain_motion_provider::get_forwards_velocity_output() const noexcept -> double {
    return normalized_forwards_velocity_provider();
}

auto
drivetrain_motion_provider::get_sideways_velocity_output() const noexcept -> double {
    return normalized_sideways_velocity_provider();
}

auto
drivetrain_motion_provider::get_angular_velocity_output() const noexcept -> double {
    return normalized_angular_velocity_provider();
}

auto
drivetrain_motion_provider::set_forwards_velocity_provider(std::function<double()> const &provider) noexcept -> void {
    this->normalized_forwards_velocity_provider = provider;
}

auto
drivetrain_motion_provider::set_sideways_velocity_provider(std::function<double()> const &provider) noexcept -> void {
    this->normalized_sideways_velocity_provider = provider;
}

auto
drivetrain_motion_provider::set_angular_velocity_provider(std::function<double()> const &provider) noexcept -> void {
    this->normalized_angular_velocity_provider = provider;
}

} // namespace td::provider
