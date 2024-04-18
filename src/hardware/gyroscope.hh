#pragma once

#include <units/angle.h>

#include <AHRS.h>

namespace td::hardware {

enum class gyroscope_convention {
    LEFT_IS_POSITIVE  = 1,
    RIGHT_IS_POSITIVE = -1
};

class gyroscope {
public:

    explicit gyroscope();

    [[nodiscard]] auto
    get_x_angle() noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_y_angle() noexcept -> units::degree_t;

    [[nodiscard]] auto
    get_z_angle() noexcept -> units::degree_t;

    auto
    reset_z_angle() noexcept -> void;
private:

    AHRS                 gyro;
    gyroscope_convention convention;

    [[nodiscard]] auto
    get_sgn() noexcept -> double;
};

} // namespace td::hardware
