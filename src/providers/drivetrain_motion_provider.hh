#pragma once

#include <functional>

namespace td::provider {

class drivetrain_motion_provider {
public:

    explicit drivetrain_motion_provider();

    auto
    get_forwards_velocity_output() const noexcept -> double;

    auto
    get_sideways_velocity_output() const noexcept -> double;

    auto
    get_angular_velocity_output() const noexcept -> double;

    auto
    set_forwards_velocity_provider(std::function<double()> const &provider) noexcept -> void;

    auto
    set_sideways_velocity_provider(std::function<double()> const &provider) noexcept -> void;

    auto
    set_angular_velocity_provider(std::function<double()> const &provider) noexcept -> void;

    auto
    log() const noexcept -> void;

private:

    std::function<double()> normalized_forwards_velocity_provider;
    std::function<double()> normalized_sideways_velocity_provider;
    std::function<double()> normalized_angular_velocity_provider;
};

} // namespace td::provider
