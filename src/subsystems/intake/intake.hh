#pragma once

#include <frc2/command/CommandPtr.h>

#include "config/encoder.hh"
#include "config/rev.hh"
#include "constants/unittype.hh"

namespace td::sub {

class intake {
public:

    explicit intake(cfg::spark_max_config const &controller_config);

    [[nodiscard]] auto
    set_percentage(double percentage) noexcept -> frc2::CommandPtr;

    [[nodiscard]] auto
    stop() noexcept -> frc2::CommandPtr;

private:

    rev::CANSparkMax controller;
};

} // namespace td::sub
