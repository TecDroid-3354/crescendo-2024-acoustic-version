#pragma once

#include <functional>
#include <string_view>

namespace td::util {

class sm_observable {
public:

    explicit sm_observable(
            std::string_view                   entity_name,
            std::function<void(double)> const &callback,
            std::function<double(void)> const &source);

    auto
    update() noexcept -> void;

    auto
    read_source() const noexcept -> double;

    auto
    set_callback(std::function<void(double)> const &callback) noexcept -> void;

    auto
    set_source(std::function<double(void)> const &source) noexcept -> void;

    auto
    log() const noexcept -> void;

private:

    std::string_view            name;
    std::function<void(double)> update_callback;
    std::function<double(void)> value_source;
};

} // namespace td::util
