#pragma once

#include <cstdint>

#include <rev/CANSparkMax.h>

#include <units/time.h>

#include "config/encoder.hh"
#include "config/pid_controller.hh"

namespace td::cfg {

/// @brief Spark identity config structure
struct spark_max_identity {
    uint8_t                     id;
    rev::CANSparkMax::MotorType motor_type;
};

/// @brief Spark behavior config structure
struct spark_max_behavior {
    rev::CANSparkMax::IdleMode idle_mode;
    units::second_t            open_ramp_rate;
    units::second_t            closed_ramp_rate;
    bool                       is_inverted;
};

/// @brief Spark config glob structure
struct spark_max_config {
    spark_max_identity identity;
    spark_max_behavior behavior;
};

/// @brief Spark PID config extension structure
struct spark_pid_coefficients {
    double ff;
};

/// @brief Spark PID output config structure
struct spark_pid_output_parameters {
    double min_output;
    double max_output;
};

/// @brief Spark PID config glob structure
struct spark_pid_config {
    pid_coefficients            coefficients;
    spark_pid_coefficients      spark_coefficients;
    pid_output_parameters       output_parameters;
    spark_pid_output_parameters spark_output_parameters;
};

/// @brief Configures a spark max through the given config
/// @param spark The spark to configue
/// @param config The config to apply
/// @return Nothing
auto
configure_spark_max(rev::CANSparkMax *spark, spark_max_config const &config) noexcept -> void;

/// @brief Configures a spark relative encoder through the given config
/// @param spark The encoder to configue
/// @param config The config to apply
/// @return Nothing
auto
configure_relative_encoder(rev::SparkRelativeEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void;

/// @brief Configures a spark absolute encoder through the given config
/// @param spark The encoder to configue
/// @param config The config to apply
/// @return Nothing
auto
configure_absolute_encoder(rev::SparkAbsoluteEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void;

/// @brief Configures a spark PID encoder through the given config
/// @param spark The PID to configue
/// @param config The config to apply
/// @return Nothing
auto
configure_spark_pid(rev::SparkPIDController *controller, spark_pid_config const &config) noexcept -> void;

} // namespace td::cfg
