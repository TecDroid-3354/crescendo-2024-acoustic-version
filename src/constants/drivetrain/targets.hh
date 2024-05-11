#pragma once

#include "propulsion.hh"

namespace td::k::dt::tgt {

/// @brief Linear velocity lead coefficient
constexpr units::dimensionless::scalar_t linear_velocity_factor = 1.0;

/// @brief Angular velocity lead coefficient
constexpr units::dimensionless::scalar_t angular_velocity_factor = 0.5;

/// @brief Angular velocity target
constexpr units::degrees_per_second_t angular_velocity = 360.0_deg_per_s * angular_velocity_factor;

/// @brief Linear velocity target
constexpr units::meters_per_second_t linear_velocity = prop::max_linear_velocity * linear_velocity_factor;
} // namespace td::k::dt::tgt
