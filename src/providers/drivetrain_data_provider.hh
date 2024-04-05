#pragma once

#include <functional>

#include <units/angle.h>

namespace td::provider {
class drivetrain_data_provider {
public:

    explicit drivetrain_data_provider() noexcept;

    auto
    get_current_angle_output() const noexcept -> units::degree_t;

    auto
    set_current_angle_provider(std::function<units::degree_t()> const &provider) noexcept -> void;

private:

    std::function<units::degree_t()> current_angle_provider = [this]() {
        return 0.0_deg;
    };
};
} // namespace td::provider
