#include "ballistics/pm/launch.h"

#include <cmath>

namespace Ballistics::PM {

LaunchState fromGround(double speed, double elevation, double azimuthOffset,
                       double altitudeMsl, double spinRate) {
    const double horizontal = speed * std::cos(elevation);
    return {glm::dvec3(0.0),
            glm::dvec3(horizontal * std::cos(azimuthOffset),
                       speed * std::sin(elevation),
                       horizontal * std::sin(azimuthOffset)),
            altitudeMsl,
            spinRate};
}

LaunchState fromPlatform(const glm::dvec3& platformVelocity,
                         const glm::dvec3& releaseVelocity,
                         const glm::dvec3& position,
                         double altitudeMsl,
                         double spinRate) {
    return {position, platformVelocity + releaseVelocity, altitudeMsl, spinRate};
}

} // namespace Ballistics::PM
