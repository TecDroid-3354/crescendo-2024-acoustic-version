#include "intake.hh"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>

#include "constants/strings.hh"

namespace td::sub {
intake::intake(cfg::spark_max_config const &controller_config)
    : controller { controller_config.identity.id, controller_config.identity.motor_type } {
    cfg::configure_spark_max(&controller, controller_config);
}

auto
intake::set_percentage(double percentage) noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this, percentage]() {
        this->controller.Set(percentage);
    });
}

auto
intake::stop() noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this]() {
        this->controller.StopMotor();
    });
}

} // namespace td::sub
