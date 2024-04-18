#pragma once

#include <string_view>

namespace td::k::str {

constexpr std::string_view general_data_tab = "General";

constexpr std::string_view drive_team_tab = "Drive Team";

constexpr std::string_view swerve_pid_manipulation_tab = "Swerve Drive PIDs";

constexpr std::string_view swerve_subsystem_tab         = "Swerve Drive";
constexpr std::string_view shooter_subsystem_tab        = "Shooter";
constexpr std::string_view climber_subsystem_tab        = "Climber";
constexpr std::string_view intake_indexer_subsystem_tab = "Intake & Indexer";
constexpr std::string_view limelight_entity_tab         = "Limelight";

} // namespace td::k::str
