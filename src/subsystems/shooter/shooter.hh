#pragma once

#include <frc2/command/CommandPtr.h>

#include "config/rev.hh"
#include "constants/unittype.hh"

namespace td::sub {

class shooter {
public:

    explicit shooter(
            cfg::spark_max_config const &controller_config_bottom,
            cfg::spark_max_config const &controller_config_top);

    [[nodiscard]] auto
    set_velocity(std::function<double()> percentage_bottom, std::function<double()> percentage_top) noexcept
            -> frc2::CommandPtr;

    [[nodiscard]] auto
    stop() noexcept -> frc2::CommandPtr;

private:

    rev::CANSparkMax controller_bottom;
    rev::CANSparkMax controller_top;
};

} // namespace td::sub
