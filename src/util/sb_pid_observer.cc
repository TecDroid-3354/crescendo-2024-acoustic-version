#include "sb_pid_observer.hh"

#include <frc/shuffleboard/Shuffleboard.h>

#include "constants/strings.hh"

namespace td::util {

sb_pid_observer::sb_pid_observer(std::string_view sb_name, std::vector<frc::PIDController *> controller_vec)
    : name { fmt::format("pid_observer_{}", sb_name) }
    , controllers { controller_vec }
    , values { controllers[0]->GetP(), controllers[0]->GetI(), controllers[0]->GetD() }
    , tab { frc::Shuffleboard::GetTab(k::str::PID_LOGS_TAB) }
    , widget { tab.Add(name, std::span<double> { values.begin(), values.end() }).GetEntry() } { }

auto
sb_pid_observer::update() -> void {
    std::span<const double> values = widget->Get().GetDoubleArray();
    if (values[0] != controllers[0]->GetP()) {
        for (auto controller : controllers) controller->SetP(values[0]);
    }
    if (values[1] != controllers[0]->GetI()) {
        for (auto controller : controllers) controller->SetI(values[1]);
    }
    if (values[2] != controllers[0]->GetD()) {
        for (auto controller : controllers) controller->SetD(values[2]);
    }
}

} // namespace td::util
