#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>

#include <rev/CANSparkMax.h>

#include "config/encoder.hh"
#include "config/pid_controller.hh"
#include "config/rev.hh"

namespace td::sub {

class climber: public frc2::SubsystemBase {
public:

    explicit climber(
            cfg::spark_max_config const          &left_config,
            cfg::spark_max_config const          &right_config,
            cfg::encoder_output_parameters const &left_encoder_config,
            cfg::encoder_output_parameters const &right_encoder_config);

    auto
    set_percentage(double left_percentage, double right_percentage) -> void;

    [[nodiscard]] auto
    lower_hooks() -> frc2::CommandPtr;

    [[nodiscard]] auto
    raise_hooks() -> frc2::CommandPtr;

    [[nodiscard]] auto
    lower_right(double percentage) -> frc2::CommandPtr;

    [[nodiscard]] auto
    lower_left(double percentage) -> frc2::CommandPtr;

    [[nodiscard]] auto
    stop() -> frc2::CommandPtr;

    [[nodiscard]] auto
    get_left_hook_position() -> units::meter_t;

    [[nodiscard]] auto
    get_right_hook_position() -> units::meter_t;

private:

    rev::CANSparkMax          left_controller;
    rev::CANSparkMax          right_controller;
    rev::SparkRelativeEncoder left_encoder;
    rev::SparkRelativeEncoder right_encoder;
};

} // namespace td::sub
