// Airborne release: where should a platform let go to hit a target on the ground?

#include "ballistics/pm/ballistics.h"

#include <cstdio>

using namespace Ballistics::PM;

int main() {
    Projectile store;
    store.id = "store";
    store.name = "inert 250 kg store";
    store.geometry = {250.0, 0.35, 2.0};
    store.drag = DragTable::g7ForCd(0.25);
    store.forces = Force::Gravity | Force::Drag | Force::Coriolis;

    Environment env = Environment::standard();
    env.latitude = 50.0;
    env.launchAzimuth = 90.0;
    env.wind = WindProfile::sounding({{0.0, 4.0, 200.0},
                                      {1500.0, 12.0, 230.0},
                                      {4000.0, 22.0, 255.0}});

    const double releaseAltitude = 4000.0;
    const GroundReference ground{-releaseAltitude};
    const glm::dvec3 platformVelocity(220.0, 0.0, 0.0);
    const glm::dvec3 target(25000.0, ground.altitudeRel, 0.0);

    const Result<ReleaseSolution> release =
        solveReleasePoint(platformVelocity, releaseAltitude, target, store, env, ground);

    if (!release) {
        std::printf("no release solution: %s\n", describe(release.status));
        return 1;
    }

    std::printf("release %.0f m short of the target and %.0f m across it\n",
                target.x - release->position.x, release->position.z);
    std::printf("fall time %.1f s, converged in %d runs to %.2f m\n",
                release->tof, release->iterations, release->residual);

    const Result<Trajectory> path = predict(
        fromPlatform(platformVelocity, glm::dvec3(0.0), release->position, releaseAltitude),
        store, env, ground);

    if (!path) {
        std::printf("release point does not reach the ground: %s\n", describe(path.status));
        return 1;
    }

    std::printf("impact at %.0f m downrange, %.0f m across, %.0f m/s at %.1f degrees\n",
                path->impact.range, path->impact.crossrange, path->impact.terminalSpeed,
                glm::degrees(path->impact.impactAngle));
    std::printf("%zu samples stored\n", path->size());
    return 0;
}
