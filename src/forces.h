#ifndef BALLISTICS_PM_FORCES_H
#define BALLISTICS_PM_FORCES_H

#include "ballistics/pm/environment.h"
#include "ballistics/pm/projectile.h"

namespace Ballistics::PM {

// Everything the acceleration depends on besides the state itself.
struct StepContext {
    const Projectile* projectile = nullptr;
    const Environment* env = nullptr;
    double baseAltitude = 0.0;    // MSL altitude of local y == 0
    double groundRel = 0.0;       // ground plane in local y, for wind height AGL
    double launchSpin = 0.0;      // rpm
};

glm::dvec3 acceleration(const glm::dvec3& pos, const glm::dvec3& vel,
                        const StepContext& ctx, double t);

} // namespace Ballistics::PM

#endif
