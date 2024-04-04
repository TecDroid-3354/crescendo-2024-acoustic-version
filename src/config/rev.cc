#include "rev.hh"

namespace td::config {

auto
configure_spark(rev::CANSparkMax &spark, spark_max const &config) noexcept -> void {
    spark.SetInverted(config.is_inverted);
    spark.SetIdleMode(config.idle_mode);
    spark.SetOpenLoopRampRate(config.open_ramp_rate.value());
    spark.SetClosedLoopRampRate(config.closed_ramp_rate.value());
}

auto
configure_neo_encoder(rev::SparkRelativeEncoder &encoder, neo_encoder const &config) noexcept -> void {
    encoder.SetPositionConversionFactor(config.position_conversion_factor);
    encoder.SetVelocityConversionFactor(config.velocity_conversion_factor);
}

} // namespace td::config
