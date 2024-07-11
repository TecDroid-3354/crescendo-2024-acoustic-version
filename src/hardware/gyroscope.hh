#pragma once

#include <units/angle.h>

#include <AHRS.h>

namespace td::hardware {

enum class gyroscope_convention {
    LEFT_IS_POSITIVE  = 1,
    RIGHT_IS_POSITIVE = -1
};

/// @brief Gyroscope wraper class
class gyroscope {
public:

    /// @brief Create a new gyroscope
    explicit gyroscope();

    /// @brief Get the robot's roll (x-axis)
    /// @return The roll angle, in degrees
    [[nodiscard]] auto
    get_x_angle() noexcept -> units::degree_t;

    /// @brief Get the robot's pitch (y-axis)
    /// @return The pitch angle, in degrees
    [[nodiscard]] auto
    get_y_angle() noexcept -> units::degree_t;

    /// @brief Get the robot's yaw (z-axis)
    /// @return The yaw angle, in degrees
    [[nodiscard]] auto
    get_z_angle() noexcept -> units::degree_t;

    /// @brief Resets the robot's yaw
    /// @return Nothing
    auto
    reset_z_angle() noexcept -> void;
private:

    AHRS                 gyro;
    gyroscope_convention convention;

    /// @brief Get the correct sign accoring to the gyroscope's convention
    /// @return The correct sign as either +1.0 or -1.0
    [[nodiscard]] auto
    get_sgn() noexcept -> double;
};

} // namespace td::hardware
