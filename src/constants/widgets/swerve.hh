#pragma once

namespace td::k::widgets {

// For automatic widget placement

unsigned int constexpr propulsion_module_current_offset = 0;
unsigned int constexpr propulsion_module_target_offset  = 1;

unsigned int constexpr azimuthal_module_current_offset = 3;
unsigned int constexpr azimuthal_module_target_offset  = 4;

unsigned int constexpr distance_module_current_offset = 6;

unsigned int constexpr azimuthal_pid_target_offset  = 5;
unsigned int constexpr propulsion_pid_target_offset = 0;

} // namespace td::k::widgets
