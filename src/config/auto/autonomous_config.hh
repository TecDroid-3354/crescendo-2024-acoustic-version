#pragma once

#include <map>
#include <string>

#include <frc/smartdashboard/SendableChooser.h>

#include "swerve/swerve_drive.hh"

namespace td::auton {

class autonomous_config {

public:

    explicit autonomous_config(sub::swerve::swerve_drive *drivetrain_ptr);

    auto
    load_existing_commands() -> void;

    [[nodiscard]] auto
    get_selected_auto() -> frc2::Command *;

private:

    sub::swerve::swerve_drive *drivetrain;

    std::map<std::string, std::unique_ptr<frc2::Command>> commands {};
    frc::SendableChooser<std::string>                     chooser;
};

} // namespace td::auton
