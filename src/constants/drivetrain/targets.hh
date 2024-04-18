#pragma once

#include "propulsion.hh"

namespace td::k::dt::tgt {

constexpr units::dimensionless::scalar_t linear_velocity_factor  = 1.0;
constexpr units::dimensionless::scalar_t angular_velocity_factor = 0.5;

constexpr units::degrees_per_second_t angular_velocity = 180.0_deg_per_s;
constexpr units::meters_per_second_t  linear_velocity  = prop::max_linear_velocity * linear_velocity_factor;
} // namespace td::k::dt::tgt
