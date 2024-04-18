#include "module.hh"

namespace td::sub::swerve {

individual_module::individual_module(cfg::swerve_module_config const &config)
    : offset { config.offset }
    , azimuth { config.azimuth_controller_config, config.cancoder_config, config.azimuth_pid_config }
    , propulsion { config.propulsion_controller_config,
                   config.propulsion_encoder_config,
                   config.propulsion_pid_config } { }

auto
individual_module::update() noexcept -> void {
    azimuth.update();
    propulsion.update();
}

auto
individual_module::optimize_state(frc::SwerveModuleState const &state) const noexcept -> frc::SwerveModuleState {
    return frc::SwerveModuleState::Optimize(state, target_state.angle);
}

auto
individual_module::set_target_state(frc::SwerveModuleState const &state) noexcept -> void {
    target_state = state;
    azimuth.set_target_angle(state.angle.Degrees());
    propulsion.set_target_velocity(state.speed);
}

auto
individual_module::get_current_angle() noexcept -> units::degree_t {
    return azimuth.get_current_angle();
}

auto
individual_module::get_target_angle() const noexcept -> units::degree_t {
    return target_state.angle.Degrees();
}

auto
individual_module::get_current_velocity() const noexcept -> units::meters_per_second_t {
    return propulsion.get_current_velocity();
}

auto
individual_module::get_target_velocity() const noexcept -> units::meters_per_second_t {
    return target_state.speed;
}

auto
individual_module::get_travelled_distance() const noexcept -> units::meter_t {
    return propulsion.get_travelled_distance();
}

auto
individual_module::get_module_position() noexcept -> frc::SwerveModulePosition {
    return frc::SwerveModulePosition { get_travelled_distance(), get_current_angle() };
}

auto
individual_module::get_module_state() noexcept -> frc::SwerveModuleState {
    return frc::SwerveModuleState { get_current_velocity(), get_current_angle() };
}

auto
individual_module::get_offset_from_center() const noexcept -> frc::Translation2d {
    return offset;
}

} // namespace td::sub::swerve
