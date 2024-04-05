#include "drivetrain_data_provider.hh"

#include <frc/shuffleboard/Shuffleboard.h>

#include "constants/strings.hh"

namespace td::provider {

drivetrain_data_provider ::drivetrain_data_provider() noexcept {
    frc::ShuffleboardTab &dt_logs = frc::Shuffleboard::GetTab(k::str::DRIVETRAIN_LOGS_TAB);

    dt_logs.AddNumber("[DTP] Current angle", [this]() {
        return get_current_angle_output().value();
    });
}

auto
drivetrain_data_provider::get_current_angle_output() const noexcept -> units::degree_t {
    return current_angle_provider();
}

auto
drivetrain_data_provider::set_current_angle_provider(std::function<units::degree_t()> const &provider) noexcept
        -> void {
    this->current_angle_provider = provider;
}

} // namespace td::provider
