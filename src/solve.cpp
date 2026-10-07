#include "ballistics/pm/solve.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Ballistics::PM {

namespace {

double horizontalMiss(const Impact& impact, const glm::dvec3& target) {
    const double dx = impact.range - target.x;
    const double dz = impact.crossrange - target.z;
    return std::sqrt(dx * dx + dz * dz);
}

double signedRangeError(const Impact& impact, const glm::dvec3& target) {
    const double reached = std::sqrt(impact.range * impact.range +
                                    impact.crossrange * impact.crossrange);
    const double wanted = std::sqrt(target.x * target.x + target.z * target.z);
    return reached - wanted;
}


// NoImpact is an ordinary outcome while sweeping elevations: some of them genuinely do not come
// down. Anything else means the request itself was wrong, and collapsing it into a range of zero
// would report a bad timestep or absent data as an unreachable target.
bool isConfigurationFailure(Status s) {
    return s != Status::Ok && s != Status::NoImpact;
}

} // namespace

Result<LaunchAim> solveLaunchDirection(const LaunchState& from, const glm::dvec3& target,
                                      const Projectile& projectile, const Environment& env,
                                      double minElevation, double maxElevation,
                                      const SolverConfig& config) {
    const GroundReference ground{target.y};
    const double speed = glm::length(from.velocity) > 0.0 ? glm::length(from.velocity)
                                                          : projectile.muzzle.speed;
    if (speed <= 0.0 || minElevation >= maxElevation) {
        return Result<LaunchAim>::fail(Status::InvalidInput);
    }

    LaunchState trial = from;
    double azimuthOffset = 0.0;
    LaunchAim best;
    int totalIterations = 0;

    for (int azIter = 0; azIter < config.maxAzimuthIterations; ++azIter) {
        double low = minElevation;
        double high = maxElevation;

        const auto rangeErrorAt = [&](double elevation) -> Result<Impact> {
            trial = fromGround(speed, elevation, azimuthOffset, from.altitudeMsl,
                               from.spinRate);
            trial.position = from.position;
            return predictImpact(trial, projectile, env, ground, config);
        };

        const Result<Impact> atLow = rangeErrorAt(low);
        const Result<Impact> atHigh = rangeErrorAt(high);
        if (!atLow || !atHigh) {
            const Status why = isConfigurationFailure(atLow.status) ? atLow.status
                               : isConfigurationFailure(atHigh.status) ? atHigh.status
                                                                      : Status::Unreachable;
            return Result<LaunchAim>::fail(why);
        }

        double errLow = signedRangeError(*atLow, target);
        const double errHigh = signedRangeError(*atHigh, target);
        if (errLow * errHigh > 0.0) {
            return Result<LaunchAim>::fail(Status::Unreachable);
        }

        Impact converged{};
        double elevation = 0.0;
        double bestError = std::numeric_limits<double>::max();

        for (int i = 0; i < config.maxElevationIterations; ++i) {
            ++totalIterations;
            const double mid = 0.5 * (low + high);
            const Result<Impact> res = rangeErrorAt(mid);
            if (!res) {
                return Result<LaunchAim>::fail(Status::Unreachable);
            }

            const double err = signedRangeError(*res, target);
            if (std::abs(err) < std::abs(bestError)) {
                bestError = err;
                elevation = mid;
                converged = *res;
            }
            if (std::abs(err) < config.rangeTolerance) {
                break;
            }

            if (err * errLow < 0.0) {
                high = mid;
            } else {
                low = mid;
                errLow = err;
            }
            if (high - low < 1e-6) {
                break;
            }
        }

        if (std::abs(bestError) > config.rangeTolerance) {
            return Result<LaunchAim>::fail(Status::NotConverged);
        }

        best = {elevation, azimuthOffset, converged.tof,
                horizontalMiss(converged, target), totalIterations};

        // Crossrange is driven by drift and wind, so the aim point has to be walked back
        // against the miss rather than solved in one pass.
        const double lateral = converged.crossrange - target.z;
        if (std::abs(lateral) <= config.crossrangeTolerance) {
            return Result<LaunchAim>::ok(best);
        }

        const double reached = std::sqrt(converged.range * converged.range +
                                        converged.crossrange * converged.crossrange);
        if (reached < 1e-6) {
            return Result<LaunchAim>::fail(Status::NotConverged);
        }
        azimuthOffset -= config.azimuthDamping * std::asin(
            std::clamp(lateral / reached, -1.0, 1.0));
    }

    return {Status::NotConverged, best};
}

AimPair solveLaunchDirections(const LaunchState& from, const glm::dvec3& target,
                              const Projectile& projectile, const Environment& env,
                              const SolverConfig& config) {
    const GroundReference ground{target.y};
    const Result<RangeEnvelope> envelope = maxRange(from, projectile, env, ground, config);
    if (!envelope) {
        return {Result<LaunchAim>::fail(envelope.status),
                Result<LaunchAim>::fail(envelope.status)};
    }

    const double apexElevation = envelope->elevation;
    constexpr double NEAR_VERTICAL = 1.5533430342749532;   // 89 degrees

    return {solveLaunchDirection(from, target, projectile, env, 0.0, apexElevation, config),
            solveLaunchDirection(from, target, projectile, env, apexElevation, NEAR_VERTICAL,
                                 config)};
}

Result<ReleaseSolution> solveReleasePoint(const glm::dvec3& platformVelocity,
                                          double releaseAltitudeMsl,
                                          const glm::dvec3& target,
                                          const Projectile& projectile,
                                          const Environment& env,
                                          const GroundReference& ground,
                                          const SolverConfig& config) {
    // From a fixed release velocity and altitude the fall is deterministic, so the release
    // point is the target minus the displacement. Only the air density along the path
    // depends on where that is, hence a couple of refinements rather than a search.
    constexpr int MAX_REFINEMENTS = 6;

    glm::dvec3 release(target.x, 0.0, target.z);
    ReleaseSolution solution;

    for (int i = 0; i < MAX_REFINEMENTS; ++i) {
        const LaunchState launch = fromPlatform(platformVelocity, glm::dvec3(0.0),
                                                glm::dvec3(release.x, 0.0, release.z),
                                                releaseAltitudeMsl, projectile.muzzle.spinRate);

        const Result<Impact> impact = predictImpact(launch, projectile, env, ground, config);
        if (!impact) {
            return Result<ReleaseSolution>::fail(impact.status);
        }

        const double missX = impact->range - target.x;
        const double missZ = impact->crossrange - target.z;

        release.x -= missX;
        release.z -= missZ;

        solution.position = glm::dvec3(release.x, 0.0, release.z);
        solution.tof = impact->tof;
        solution.residual = std::sqrt(missX * missX + missZ * missZ);
        solution.iterations = i + 1;

        if (solution.residual < config.rangeTolerance) {
            return Result<ReleaseSolution>::ok(solution);
        }
    }

    return {Status::NotConverged, solution};
}

Result<RangeEnvelope> maxRange(const LaunchState& from, const Projectile& projectile,
                               const Environment& env, const GroundReference& ground,
                               const SolverConfig& config) {
    const double speed = glm::length(from.velocity) > 0.0 ? glm::length(from.velocity)
                                                          : projectile.muzzle.speed;
    if (speed <= 0.0) {
        return Result<RangeEnvelope>::fail(Status::InvalidInput);
    }

    Status failure = Status::Ok;
    const auto rangeAt = [&](double elevation) -> double {
        LaunchState trial = fromGround(speed, elevation, 0.0, from.altitudeMsl, from.spinRate);
        trial.position = from.position;
        const Result<Impact> r = predictImpact(trial, projectile, env, ground, config);
        if (isConfigurationFailure(r.status)) {
            failure = r.status;
        }
        return r ? r->range : 0.0;
    };

    // Golden section over elevation. Firing downhill moves the optimum below 45 degrees.
    static const double RESPHI = 2.0 - (1.0 + std::sqrt(5.0)) * 0.5;
    const double terrain = std::atan2(ground.altitudeRel,
                                      std::max(1.0, std::abs(from.position.x) + 1000.0));
    const double guess = std::clamp(0.7853981633974483 - terrain * 0.5, 0.0, 1.5533430342749532);

    double a = std::max(0.0, guess - 0.4363323129985824);
    double b = std::min(1.5533430342749532, guess + 0.4363323129985824);

    double x1 = a + RESPHI * (b - a);
    double x2 = b - RESPHI * (b - a);
    double f1 = rangeAt(x1);
    double f2 = rangeAt(x2);

    constexpr double TOLERANCE = 1e-4;   // radians
    while (std::abs(b - a) > TOLERANCE) {
        if (f1 > f2) {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = a + RESPHI * (b - a);
            f1 = rangeAt(x1);
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = b - RESPHI * (b - a);
            f2 = rangeAt(x2);
        }
    }

    const double elevation = 0.5 * (a + b);
    const double reach = rangeAt(elevation);
    if (failure != Status::Ok) {
        return Result<RangeEnvelope>::fail(failure);
    }
    if (reach <= 0.0) {
        return Result<RangeEnvelope>::fail(Status::NoImpact);
    }
    return Result<RangeEnvelope>::ok({elevation, reach});
}

} // namespace Ballistics::PM
