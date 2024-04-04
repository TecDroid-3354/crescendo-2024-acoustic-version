#pragma once

#include "providers/angle_provider.hh"

namespace td::subsystems {

class angle {
public:

    explicit angle();
private:

    provider::angle_provider angle_provider;
};

} // namespace td::subsystems
