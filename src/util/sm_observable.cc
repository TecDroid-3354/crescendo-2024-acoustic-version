#include "sm_observable.hh"

#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace td::util {

sm_observable::sm_observable(
        std::string_view                   entity_name,
        std::function<void(double)> const &callback,
        std::function<double(void)> const &source)
    : name(fmt::format("obs_{}", std::move(entity_name)))
    , update_callback(callback)
    , value_source(source) { }

auto
sm_observable::update() noexcept -> void {
    double src_value = read_source();
    frc::SmartDashboard::PutNumber(name, src_value);

    if (double value = frc::SmartDashboard::GetNumber(name, 0.0); value != src_value) { update_callback(value); }
}

auto
sm_observable::read_source() const noexcept -> double {
    return value_source();
}

auto
sm_observable::set_callback(std::function<void(double)> const &callback) noexcept -> void {
    this->update_callback = callback;
}

auto
sm_observable::set_source(std::function<double(void)> const &source) noexcept -> void {
    this->value_source = source;
}

auto
sm_observable::log() const noexcept -> void {
    frc::SmartDashboard::PutNumber(fmt::format("[OBS] {}", name), read_source());
}

} // namespace td::util
