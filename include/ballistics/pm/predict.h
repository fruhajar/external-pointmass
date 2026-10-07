#ifndef BALLISTICS_PM_PREDICT_H
#define BALLISTICS_PM_PREDICT_H

#include "config.h"
#include "environment.h"
#include "launch.h"
#include "projectile.h"
#include "types.h"

namespace Ballistics::PM {

// Integrates to the ground reference. predictImpact skips sampling entirely and is the
// form to call from anything iterating.
Result<Impact> predictImpact(const LaunchState& launch,
                             const Projectile& projectile,
                             const Environment& env,
                             const GroundReference& ground,
                             const SolverConfig& config = SolverConfig::standard());

Result<Trajectory> predict(const LaunchState& launch,
                           const Projectile& projectile,
                           const Environment& env,
                           const GroundReference& ground,
                           const SolverConfig& config = SolverConfig::standard());

} // namespace Ballistics::PM

#endif
