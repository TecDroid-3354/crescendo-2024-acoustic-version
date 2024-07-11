#pragma once

#include <string_view>

namespace td::k::str {

/// @brief Tab to display common data
constexpr std::string_view general_data_tab = "General";

/// @brief Tab to display data pertinent to the driveteam
constexpr std::string_view drive_team_tab = "Drive Team";

/// @brief Tab to display swerve PID data
constexpr std::string_view swerve_pid_manipulation_tab = "Swerve Drive PIDs";

/// @brief Tab to display swerve data
constexpr std::string_view swerve_subsystem_tab = "Swerve Drive";

/// @brief Tab to display shooter data
constexpr std::string_view shooter_subsystem_tab = "Shooter";

/// @brief Tab to display climber data
constexpr std::string_view climber_subsystem_tab = "Climber";

/// @brief Tab to display intake & indexer data
constexpr std::string_view intake_indexer_subsystem_tab = "Intake & Indexer";

/// @brief Tab to display limelight data
constexpr std::string_view limelight_entity_tab = "Limelight";

} // namespace td::k::str
