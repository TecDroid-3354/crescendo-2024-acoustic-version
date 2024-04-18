#include "rev.hh"

namespace td::cfg {

auto
configure_spark_max(rev::CANSparkMax *spark, spark_max_config const &config) noexcept -> void {
    spark->SetIdleMode(config.behavior.idle_mode);
    spark->SetOpenLoopRampRate(config.behavior.open_ramp_rate.value());
    spark->SetClosedLoopRampRate(config.behavior.closed_ramp_rate.value());
    spark->SetInverted(config.behavior.is_inverted);
}

auto
configure_relative_encoder(rev::SparkRelativeEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void {
    encoder->SetPositionConversionFactor(config.position_conversion_factor);
    encoder->SetVelocityConversionFactor(config.velocity_conversion_factor);
}

auto
configure_absolute_encoder(rev::SparkAbsoluteEncoder *encoder, encoder_output_parameters const &config) noexcept
        -> void {
    encoder->SetPositionConversionFactor(config.position_conversion_factor);
    encoder->SetVelocityConversionFactor(config.velocity_conversion_factor);
    encoder->SetInverted(config.is_inverted);
}

auto
configure_spark_pid(rev::SparkPIDController *controller, spark_pid_config const &config) noexcept -> void {
    controller->SetP(config.coefficients.p);
    controller->SetI(config.coefficients.i);
    controller->SetD(config.coefficients.d);
    controller->SetFF(config.spark_coefficients.ff);

    controller->SetPositionPIDWrappingMinInput(config.output_parameters.ci_min);
    controller->SetPositionPIDWrappingMaxInput(config.output_parameters.ci_max);

    controller->SetPositionPIDWrappingEnabled(config.output_parameters.ci_enabled);

    controller->SetOutputRange(config.spark_output_parameters.min_output, config.spark_output_parameters.max_output);
}

} // namespace td::cfg
