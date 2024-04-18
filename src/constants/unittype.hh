#pragma once

#include <units/angular_velocity.h>
#include <units/time.h>

namespace units {
using turns_per_minute_t = units::unit_t<units::compound_unit<units::turn, units::inverse<units::minute>>>;
} // namespace units
