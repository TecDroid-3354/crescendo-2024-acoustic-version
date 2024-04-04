#pragma once

#include <functional>
#include <units/angle.h>

namespace td::provider {

class angle_provider {
public:

    explicit angle_provider() noexcept;

    auto
    get_target_angle_output() const noexcept -> units::degree_t;

    auto
    get_current_angle_output() const noexcept -> units::degree_t;

    auto
    set_target_angle_provider(std::function<units::degree_t()> const &provider) noexcept -> void;

    auto
    set_current_angle_provider(std::function<units::degree_t()> const &provider) noexcept -> void;

private:

    std::function<units::degree_t()> target_angle_provider;
    std::function<units::degree_t()> current_angle_provider;
};

} // namespace td::provider
