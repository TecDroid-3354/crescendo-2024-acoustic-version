#pragma once

namespace td::util {

[[nodiscard]] constexpr auto
apply_absolute_deadband(double value, double deadband_val) -> double;

} // namespace td::util
