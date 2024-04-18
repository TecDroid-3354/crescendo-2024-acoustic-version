#pragma once

#include <cstdint>

#include <rev/CANSparkMax.h>

#include <units/time.h>

#include "config/encoder.hh"
#include "config/pid_controller.hh"

namespace td::cfg {

struct spark_max_identity {
    uint8_t                     id;
    rev::CANSparkMax::MotorType motor_type;
};

struct spark_max_behavior {
    rev::CANSparkMax::IdleMode idle_mode;
    units::second_t            open_ramp_rate;
    units::second_t            closed_ramp_rate;
    bool                       is_inverted;
};

struct spark_max_config {
    spark_max_identity identity;
    spark_max_behavior behavior;
};

struct spark_pid_coefficients {
    double ff;
};

struct spark_pid_output_parameters {
    double min_output;
    double max_output;
};

struct spark_pid_config {
    pid_coefficients            coefficients;
    spark_pid_coefficients      spark_coefficients;
    pid_output_parameters       output_parameters;
    spark_pid_output_parameters spark_output_parameters;
};

auto
configure_spark_max(rev::CANSparkMax *spark, spark_max_config const &config) noexcept -> void;

auto
configure_relative_encoder(rev::SparkRelativeEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void;

auto
configure_absolute_encoder(rev::SparkAbsoluteEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void;

auto
configure_spark_pid(rev::SparkPIDController *controller, spark_pid_config const &config) noexcept -> void;

} // namespace td::cfg
