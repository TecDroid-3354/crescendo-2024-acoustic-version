#pragma once

#include <map>
#include <string>

#include <frc/smartdashboard/SendableChooser.h>

#include "swerve/swerve_drive.hh"

namespace td::auton {

class autonomous_config {

public:

    /// @brief Constructs an autonomous config object
    /// @param drivetrain_ptr A pointer to the drivetrain to control
    explicit autonomous_config(sub::swerve::swerve_drive *drivetrain_ptr);

    /// @brief Searches for autonomous commands in the roboRIO's deploy directory
    /// and adds them to a SendableChooser
    /// @return Nothing
    auto
    load_existing_commands() -> void;

    /// @brief Returns the auto selected from the SendableChooser
    /// @return The selected command
    [[nodiscard]] auto
    get_selected_auto() -> frc2::Command *;

private:

    sub::swerve::swerve_drive *drivetrain;

    std::map<std::string, std::unique_ptr<frc2::Command>> commands {};
    frc::SendableChooser<std::string>                     chooser;
};

} // namespace td::auton
