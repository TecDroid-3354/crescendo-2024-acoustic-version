#include "pid_controller.hh"

namespace td::cfg {

[[nodiscard]] auto
configure_pid_controller(frc::PIDController *controller, pid_config const &config) noexcept -> void {
    controller->SetPID(config.coefficients.p, config.coefficients.i, config.coefficients.d);
    controller->SetTolerance(config.output_parameters.tolerance);

    if (config.output_parameters.ci_enabled) {
        controller->EnableContinuousInput(config.output_parameters.ci_min, config.output_parameters.ci_max);
    }
}

} // namespace td::cfg
