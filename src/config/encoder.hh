#pragma once

namespace td::cfg {

/// @brief Encoder config structure
struct encoder_output_parameters {
    double position_conversion_factor;
    double velocity_conversion_factor;
    bool   is_inverted;
};

} // namespace td::cfg
