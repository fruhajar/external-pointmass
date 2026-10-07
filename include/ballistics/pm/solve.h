#ifndef BALLISTICS_PM_SOLVE_H
#define BALLISTICS_PM_SOLVE_H

#include "predict.h"

namespace Ballistics::PM {

struct LaunchAim {
    double elevation = 0.0;       // radians
    double azimuthOffset = 0.0;   // radians from the environment's launch azimuth
    double tof = 0.0;
    double residual = 0.0;        // metres of miss at convergence
    int iterations = 0;
};

// Elevation and azimuth to hit a target at the given local-frame offset from a fixed
// launch point. Searches within [minElevation, maxElevation], both radians.
Result<LaunchAim> solveLaunchDirection(const LaunchState& from,
                                       const glm::dvec3& target,
                                       const Projectile& projectile,
                                       const Environment& env,
                                       double minElevation,
                                       double maxElevation,
                                       const SolverConfig& config = SolverConfig::standard());

// Both arcs to the same target: the flat trajectory first, the lofted one second.
struct AimPair {
    Result<LaunchAim> direct;
    Result<LaunchAim> indirect;
};

AimPair solveLaunchDirections(const LaunchState& from,
                              const glm::dvec3& target,
                              const Projectile& projectile,
                              const Environment& env,
                              const SolverConfig& config = SolverConfig::standard());

struct ReleaseSolution {
    glm::dvec3 position{0.0};   // where to release, local frame
    double tof = 0.0;
    double residual = 0.0;
    int iterations = 0;
};

// Where a platform flying at the given velocity and altitude should release to hit the
// target. The ballistic displacement from a fixed release state is deterministic, so this
// converges in a couple of forward runs rather than a search.
Result<ReleaseSolution> solveReleasePoint(const glm::dvec3& platformVelocity,
                                          double releaseAltitudeMsl,
                                          const glm::dvec3& target,
                                          const Projectile& projectile,
                                          const Environment& env,
                                          const GroundReference& ground,
                                          const SolverConfig& config = SolverConfig::standard());

struct RangeEnvelope {
    double elevation = 0.0;   // radians giving maximum range
    double range = 0.0;
};

Result<RangeEnvelope> maxRange(const LaunchState& from,
                               const Projectile& projectile,
                               const Environment& env,
                               const GroundReference& ground,
                               const SolverConfig& config = SolverConfig::standard());

} // namespace Ballistics::PM

#endif
