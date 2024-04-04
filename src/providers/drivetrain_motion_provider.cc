#include "drivetrain_motion_provider.hh"

#include <frc/smartdashboard/SmartDashboard.h>

namespace td::provider {

drivetrain_motion_provider::drivetrain_motion_provider() = default;

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

auto
drivetrain_motion_provider::log() const noexcept -> void {
    frc::SmartDashboard::PutNumber("[DTP] Normalized forwards output", get_forwards_velocity_output());
    frc::SmartDashboard::PutNumber("[DTP] Normalized sideways output", get_sideways_velocity_output());
    frc::SmartDashboard::PutNumber("[DTP] Normalized angular output", get_angular_velocity_output());
}

} // namespace td::provider
