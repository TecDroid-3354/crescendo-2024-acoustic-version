#pragma once

#include <cstdint>

namespace td::k::port {

/// @brief Main controller port
constexpr uint8_t controller_a { 0U };

/// @brief Secondary controller port
constexpr uint8_t controller_b { 1U };

/// @brief Limit Switch port
constexpr uint8_t lswitch_dio_pin { 1U };

} // namespace td::k::port
