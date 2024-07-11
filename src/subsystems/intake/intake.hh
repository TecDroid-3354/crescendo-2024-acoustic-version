#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "config/encoder.hh"
#include "config/rev.hh"
#include "constants/unittype.hh"

namespace td::sub {

class intake: public frc2::SubsystemBase {
public:

    explicit intake(cfg::spark_max_config const &controller_config);

    /// @brief Set operation percentage
    /// @param percentage The percentage to put the motors in
    /// @return The command to execute
    [[nodiscard]] auto
    set_percentage(double percentage) noexcept -> frc2::CommandPtr;

    /// @brief Stop the subsystem
    /// @return The command to execute
    [[nodiscard]] auto
    stop() noexcept -> frc2::CommandPtr;

private:

    rev::CANSparkMax controller;
};

} // namespace td::sub
