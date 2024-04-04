#include "drive.hh"

#include "constants/drive.hh"

namespace td::util {
constexpr auto
normalized_input_to_chassis_speeds(double ix, double iy, double ir) noexcept -> frc::ChassisSpeeds {
    return frc::ChassisSpeeds { ix * k::swerve::target_linear_velocity,
                                iy * k::swerve::target_linear_velocity,
                                ir * k::swerve::target_angular_velocity };
}
} // namespace td::util
