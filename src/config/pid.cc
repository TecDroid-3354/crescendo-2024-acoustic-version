#include "pid.hh"

namespace td::config {
auto
mkpid_controller(pid_controller const &config) noexcept -> frc::PIDController && {
    frc::PIDController controller { config.p, config.i, config.d };
    controller.SetTolerance(config.tolerance);

    if (config.ci_enabled) { controller.EnableContinuousInput(config.ci_min, config.ci_max); }

    return std::move(controller);
}

} // namespace td::config
