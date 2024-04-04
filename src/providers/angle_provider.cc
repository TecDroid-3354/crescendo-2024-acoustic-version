#include "angle_provider.hh"

namespace td::provider {

angle_provider::angle_provider() noexcept { }

auto
angle_provider::get_target_angle_output() const noexcept -> units::degree_t {
    return target_angle_provider();
}

auto
angle_provider::get_current_angle_output() const noexcept -> units::degree_t {
    return current_angle_provider();
}

auto
angle_provider::set_target_angle_provider(std::function<units::degree_t()> const &provider) noexcept -> void {
    this->target_angle_provider = provider;
}

auto
angle_provider::set_current_angle_provider(std::function<units::degree_t()> const &provider) noexcept -> void {
    this->current_angle_provider = provider;
}

} // namespace td::provider
