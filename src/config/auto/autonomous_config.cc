#include "autonomous_config.hh"

#include <filesystem>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "constants/autonomous.hh"
#include "constants/drivetrain/measurements.hh"
#include "constants/drivetrain/targets.hh"
#include "constants/strings.hh"

namespace td::auton {

autonomous_config::autonomous_config(sub::swerve::swerve_drive *drivetrain_ptr)
    : drivetrain { drivetrain_ptr } {
    frc::ShuffleboardTab &operator_tab = frc::Shuffleboard::GetTab(k::str::drive_team_tab);
    operator_tab.Add("Autonomous Routine", chooser);

    // Configures pathplanner to use the drivetrain's methods to drive
    pathplanner::AutoBuilder::configureHolonomic(
            // Lambda to get robot's pose
            [this]() {
                return this->drivetrain->get_pose();
            },
            // Lambda to set robot's pose
            [this](frc::Pose2d pose) {
                this->drivetrain->set_pose(pose);
            },
            // Lambda to get robot's speeds
            [this]() {
                return this->drivetrain->get_current_speeds();
            },
            // Lambda to drive the robot
            [this](frc::ChassisSpeeds speeds) {
                return this->drivetrain->drive(speeds);
            },

            // PID coefficients
            pathplanner::HolonomicPathFollowerConfig { pathplanner::PIDConstants(
                                                               k::auton::move_coefficients.p,
                                                               k::auton::move_coefficients.i,
                                                               k::auton::move_coefficients.d),
                                                       pathplanner::PIDConstants(
                                                               k::auton::turn_coefficients.p,
                                                               k::auton::turn_coefficients.i,
                                                               k::auton::turn_coefficients.d),
                                                       k::dt::tgt::linear_velocity,
                                                       k::dt::ms::module_forwards_offset,
                                                       pathplanner::ReplanningConfig {} },
            []() {
                // Invert path if alliance is red
                auto alliance = frc::DriverStation::GetAlliance();
                if (alliance.has_value()) { return alliance.value() == frc::DriverStation::Alliance::kRed; }
                return false;
            },
            this->drivetrain);
}

auto
autonomous_config::load_existing_commands() -> void {
    int num = 1;

    // Default command: No auto
    commands["None"] = std::move(frc2::cmd::None().Unwrap());
    chooser.SetDefaultOption("0 - None", "None");

    // Fetch deploy directory
    auto deploy_directory = frc::filesystem::GetDeployDirectory() + "/pathplanner/autos";

    // For each file, if it is an auto file, load and add
    for (auto const &file : std::filesystem::directory_iterator(deploy_directory)) {
        if (!file.is_regular_file()) continue;
        std::string filename = file.path().stem();

        commands[filename] = std::move(pathplanner::AutoBuilder::buildAuto(filename).Unwrap());
        chooser.AddOption(fmt::format("{} - {}", num, filename), filename);

        ++num;
    }
}

auto
autonomous_config::get_selected_auto() -> frc2::Command * {
    // Selected Command's Name
    std::string selected = chooser.GetSelected();

    // Fetch from command map
    return commands[selected].get();
}

} // namespace td::auton
