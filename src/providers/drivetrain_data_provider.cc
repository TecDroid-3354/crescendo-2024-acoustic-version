#include "drivetrain_data_provider.hh"

#include <frc/smartdashboard/SmartDashboard.h>

namespace td::provider {

drivetrain_data_provider ::drivetrain_data_provider() noexcept { }

auto
drivetrain_data_provider::get_current_angle_output() const noexcept -> units::degree_t {
    return current_angle_provider();
}

auto
drivetrain_data_provider::set_current_angle_provider(std::function<units::degree_t()> const &provider) noexcept
        -> void {
    this->current_angle_provider = provider;
}

auto
drivetrain_data_provider::log() const noexcept -> void {
    frc::SmartDashboard::PutNumber("[DTP] Current angle", get_current_angle_output().value());
}

} // namespace td::provider
