
#include "shooter.hh"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>

#include "constants/strings.hh"

namespace td::sub {
shooter::shooter(
        cfg::spark_max_config const &controller_config_bottom,
        cfg::spark_max_config const &controller_config_top)
    : controller_bottom { controller_config_bottom.identity.id, controller_config_bottom.identity.motor_type }
    , controller_top { controller_config_top.identity.id, controller_config_top.identity.motor_type } {
    cfg::configure_spark_max(&controller_bottom, controller_config_bottom);
    cfg::configure_spark_max(&controller_top, controller_config_top);

}

auto
shooter::set_velocity(double percentage_bottom, double percentage_top) noexcept
        -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this, percentage_bottom, percentage_top]() {
        this->controller_bottom.Set(percentage_bottom);
        this->controller_top.Set(percentage_top);
    });
}

auto
shooter::stop() noexcept -> frc2::CommandPtr {
    return frc2::cmd::RunOnce([this]() {
        this->controller_bottom.Set(0.0);
        this->controller_top.Set(0.0);
    });
}

} // namespace td::sub
