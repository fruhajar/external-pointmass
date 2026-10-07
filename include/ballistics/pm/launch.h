#ifndef BALLISTICS_PM_LAUNCH_H
#define BALLISTICS_PM_LAUNCH_H

#include "types.h"

namespace Ballistics::PM {

struct LaunchState {
    glm::dvec3 position{0.0};
    glm::dvec3 velocity{0.0};     // includes platform motion
    double altitudeMsl = 0.0;
    double spinRate = 0.0;        // rpm, zero for a dropped or thrown store
};

// Fired from a fixed point. elevation and azimuthOffset are radians, the latter measured
// from the launch azimuth that the Environment already carries.
LaunchState fromGround(double speed, double elevation, double azimuthOffset,
                       double altitudeMsl, double spinRate = 0.0);

// Released from a moving platform. releaseVelocity is any ejection or throw imparted on
// top of the platform's own motion, in the local frame.
LaunchState fromPlatform(const glm::dvec3& platformVelocity,
                         const glm::dvec3& releaseVelocity,
                         const glm::dvec3& position,
                         double altitudeMsl,
                         double spinRate = 0.0);

// Height of the impact surface relative to the launch point. Negative when firing or
// dropping onto lower ground, which is the normal airborne case.
struct GroundReference {
    double altitudeRel = 0.0;
};

} // namespace Ballistics::PM

#endif
