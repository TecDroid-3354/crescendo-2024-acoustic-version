#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "config/rev.hh"
#include "constants/unittype.hh"

namespace td::sub {

class shooter: public frc2::SubsystemBase {
public:

    explicit shooter(
            cfg::spark_max_config const &controller_config_bottom,
            cfg::spark_max_config const &controller_config_top);

    /// @brief Set the wheel velocities
    /// @param percentage_bottom Velocity percentage of the bottom wheel
    /// @param percentage_top Velocity percentage of the top wheel
    /// @return The command to execute
    [[nodiscard]] auto
    set_velocity(double percentage_bottom, double percentage_top) noexcept -> frc2::CommandPtr;

    /// @brief Stop the subsystem
    /// @return The command to execute
    [[nodiscard]] auto
    stop() noexcept -> frc2::CommandPtr;

private:

    rev::CANSparkMax controller_bottom;
    rev::CANSparkMax controller_top;
};

} // namespace td::sub
