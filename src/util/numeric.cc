#include "numeric.hh"

#include <cmath>

namespace td::util {

constexpr auto
apply_absolute_deadband(double value, double deadband_val) -> double {
    if (std::abs(value < deadband_val)) { return 0.0; }
    return value;
}

} // namespace td::util
