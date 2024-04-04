#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/button/CommandXboxController.h>

namespace td::util {

[[nodiscard]] constexpr auto
normalized_input_to_chassis_speeds(double ix, double iy, double ir) noexcept -> frc::ChassisSpeeds;

} // namespace td::util
