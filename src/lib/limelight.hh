#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>

#include <networktables/NetworkTable.h>

#include <units/angle.h>
#include <units/length.h>

namespace td::ll {

extern std::shared_ptr<nt::NetworkTable> ll_table;

constexpr units::meter_t  mount_height = 53.9_cm;
constexpr units::degree_t mount_angle  = 35.0_deg;

constexpr units::meter_t speaker_height = 53.88_in;
constexpr units::meter_t source_height  = 48.13_in;
constexpr units::meter_t amp_height     = 50.13_in;
constexpr units::meter_t stage_height   = 48.81_in;

enum class target_type {
    SPEAKER,
    SOURCE,
    AMP,
    STAGE,
    NONE
};

auto
init() noexcept -> void;

[[nodiscard]] auto
has_target() noexcept -> bool;

[[nodiscard]] auto
has_no_targets() noexcept -> bool;

[[nodiscard]] auto
get_target_id() noexcept -> int;

[[nodiscard]] auto
get_horizontal_angle_to_target() noexcept -> units::degree_t;

[[nodiscard]] auto
get_vertical_angle_to_target() noexcept -> units::degree_t;

[[nodiscard]] auto
get_distance_to_target() noexcept -> units::meter_t;

[[nodiscard]] auto
get_target_type() noexcept -> target_type;

[[nodiscard]] auto
get_height_of_target_type() -> units::meter_t;

} // namespace td::ll
