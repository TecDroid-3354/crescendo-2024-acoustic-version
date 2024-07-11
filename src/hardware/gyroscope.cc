#include "gyroscope.hh"

#include <frc/shuffleboard/Shuffleboard.h>

#include "constants/strings.hh"
#include "constants/widgets/data.hh"

namespace td::hardware {
gyroscope::gyroscope()
    : gyro(frc::SerialPort::kMXP)
    , convention(gyroscope_convention::RIGHT_IS_POSITIVE) {
    /// Add data to dashboard
    frc::ShuffleboardTab &data_tab = frc::Shuffleboard::GetTab(k::str::general_data_tab);
    data_tab.AddDouble(
                    "Gyroscope X",
                    [this]() {
                        return get_x_angle().value();
                    })
            .WithWidget(frc::BuiltInWidgets::kGyro);
    data_tab.AddDouble(
                    "Gyroscope Y",
                    [this]() {
                        return get_y_angle().value();
                    })
            .WithWidget(frc::BuiltInWidgets::kGyro);
    data_tab.AddDouble(
                    "Gyroscope Z",
                    [this]() {
                        return get_z_angle().value();
                    })
            .WithWidget(frc::BuiltInWidgets::kGyro);
    gyro.ZeroYaw();
}

auto
gyroscope::get_x_angle() noexcept -> units::degree_t {
    return units::degree_t { gyro.GetRoll() * get_sgn() };
}

auto
gyroscope::get_y_angle() noexcept -> units::degree_t {
    return units::degree_t { gyro.GetPitch() * get_sgn() };
}

auto
gyroscope::get_z_angle() noexcept -> units::degree_t {
    return units::degree_t { gyro.GetYaw() * get_sgn() };
}

auto
gyroscope::reset_z_angle() noexcept -> void {
    gyro.ZeroYaw();
    gyro.Reset();
}

auto
gyroscope::get_sgn() noexcept -> double {
    // If the gyroscope reports positive values as it moves left,
    // leave it as-is. Else, invert the sign
    switch (convention) {
    case gyroscope_convention::LEFT_IS_POSITIVE : {
        return 1.0;
    }
    case gyroscope_convention::RIGHT_IS_POSITIVE : {
        return -1.0;
    }
    }

    return 0.0;
}

} // namespace td::hardware
