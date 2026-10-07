#ifndef BALLISTICS_PM_ENVIRONMENT_H
#define BALLISTICS_PM_ENVIRONMENT_H

#include "atmosphere.h"
#include "wind.h"

namespace Ballistics::PM {

struct Environment {
    Atmosphere atmosphere;
    WindProfile wind;
    double latitude = 0.0;        // degrees, for Coriolis
    double launchAzimuth = 0.0;   // degrees

    static Environment standard();
};

} // namespace Ballistics::PM

#endif
