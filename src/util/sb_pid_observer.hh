#pragma once

#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>

namespace td::util {

class sb_pid_observer {
public:

    explicit sb_pid_observer(std::string_view sb_name, std::vector<frc::PIDController *> controller_vec);

    auto
    update() -> void;

private:

    std::string_view                  name;
    std::vector<frc::PIDController *> controllers;
    std::vector<double>               values;
    frc::ShuffleboardTab             &tab;
    nt::GenericEntry                 *widget;
};

} // namespace td::util
