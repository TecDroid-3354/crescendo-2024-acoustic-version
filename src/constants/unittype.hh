#pragma once

#include <units/angular_velocity.h>
#include <units/time.h>

namespace units {
/// @brief Definition of a new unit in order to have automatic conversion
using turns_per_minute_t = units::unit_t<units::compound_unit<units::turn, units::inverse<units::minute>>>;
} // namespace units
