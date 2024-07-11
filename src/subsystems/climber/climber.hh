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

    /// @brief Set individual percentages
    /// @param left_percentage Percentage for the left motor
    /// @param right_percentage Percentge for the right motor
    /// @return Nothing
    auto
    set_percentage(double left_percentage, double right_percentage) -> void;

    /// @brief Builds a command to lower both hooks
    /// @return The command to execute
    [[nodiscard]] auto
    lower_hooks() -> frc2::CommandPtr;

    /// @brief Builds a command to raise both hooks
    /// @return The command to execute
    [[nodiscard]] auto
    raise_hooks() -> frc2::CommandPtr;

    /// @brief Stop the subsytstem
    /// @return The command to execute
    [[nodiscard]] auto
    stop() -> frc2::CommandPtr;

    /// @brief Get the travelled distance of the left hook string
    /// @return The traveled distance of the string, in meters
    [[nodiscard]] auto
    get_left_hook_position() -> units::meter_t;

    /// @brief Get the travelled distance of the right hook string
    /// @return The traveled distance of the string, in meters
    [[nodiscard]] auto
    get_right_hook_position() -> units::meter_t;

private:

    rev::CANSparkMax          left_controller;
    rev::CANSparkMax          right_controller;
    rev::SparkRelativeEncoder left_encoder;
    rev::SparkRelativeEncoder right_encoder;
};

} // namespace td::sub
