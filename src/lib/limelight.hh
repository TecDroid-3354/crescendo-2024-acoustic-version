#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>

#include <networktables/NetworkTable.h>

#include <units/angle.h>
#include <units/length.h>

namespace td::ll {

/// @brief The network table from which to fetch limelight data
extern std::shared_ptr<nt::NetworkTable> ll_table;

/// @brief Limelight's mount height
constexpr units::meter_t mount_height = 53.9_cm;

/// @brief Limelight's mount angle
constexpr units::degree_t mount_angle = 35.0_deg;

/// @brief Height at which the speaker apriltag is located
constexpr units::meter_t speaker_height = 53.88_in;

/// @brief Height at which the source apriltag is located
constexpr units::meter_t source_height = 48.13_in;

/// @brief Height at which the amp apriltag is located
constexpr units::meter_t amp_height = 50.13_in;

/// @brief Height at which the stage apriltag is located
constexpr units::meter_t stage_height = 48.81_in;

/// @brief Target type that the limelight must search for
enum class target_type {
    SPEAKER,
    SOURCE,
    AMP,
    STAGE,
    NONE
};

/// @brief Initialize the limelight
/// @return Nothng
auto
init() noexcept -> void;

/// @brief Checks if the limelight has a target
/// @return True if the limelight has target(s)
[[nodiscard]] auto
has_target() noexcept -> bool;

/// @brief Checks if the limelight has no targets
/// @return True if the limelight has no targets
[[nodiscard]] auto
has_no_targets() noexcept -> bool;

/// @brief Gets the ID of a visible target
/// @return The id of the focused target
[[nodiscard]] auto
get_target_id() noexcept -> int;

/// @brief Get the horizontal angle offset to the target
/// @return The horizontal offset, in degrees
[[nodiscard]] auto
get_horizontal_angle_to_target() noexcept -> units::degree_t;

/// @brief Get the vertical angle offset to the target
/// @return The vertical offset, in degrees
[[nodiscard]] auto
get_vertical_angle_to_target() noexcept -> units::degree_t;

/// @brief Calculates the distance to the target
/// @return The estimated distance from the target
[[nodiscard]] auto
get_distance_to_target() noexcept -> units::meter_t;

/// @brief Returns what kind of target the limelight is pointing at
/// @return The target type
[[nodiscard]] auto
get_target_type() noexcept -> target_type;

/// @brief Returns the height at which the target is expected to be found
/// @return The expected height of the target
[[nodiscard]] auto
get_height_of_target_type() -> units::meter_t;

} // namespace td::ll
