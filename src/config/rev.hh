#pragma once

#include <cstdint>
#include <rev/CANSparkMax.h>
#include <units/time.h>

namespace td::config {

struct spark_max {
    uint8_t                     id;
    rev::CANSparkMax::MotorType motor_type       = rev::CANSparkMax::MotorType::kBrushless;
    rev::CANSparkMax::IdleMode  idle_mode        = rev::CANSparkMax::IdleMode::kBrake;
    units::second_t             open_ramp_rate   = 0_s;
    units::second_t             closed_ramp_rate = 0_s;
    bool                        is_inverted      = false;
};

struct neo_encoder {
    double position_conversion_factor = 1.0;
    double velocity_conversion_factor = 1.0;
};

auto
configure_spark(rev::CANSparkMax &spark, spark_max const &config) noexcept -> void;

auto
configure_neo_encoder(rev::SparkRelativeEncoder &encoder, neo_encoder const &config) noexcept -> void;

} // namespace td::config
