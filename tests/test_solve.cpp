#include "ballistics/pm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::PM;

namespace {

Environment quiet() {
    Environment e = Environment::standard();
    e.latitude = 50.0;
    return e;
}

const Projectile& round155() {
    static const Catalogue c = Catalogue::withBuiltins();
    return *c.find("Howitzer155mm_HE");
}

} // namespace

int main() {
    const Environment env = quiet();
    const GroundReference flat{0.0};
    const Projectile& shell = round155();

    // Max range sits near 45 degrees on flat ground and is a genuine maximum.
    {
        const LaunchState from = fromGround(shell.muzzle.speed, 0.0, 0.0, 0.0,
                                            shell.muzzle.spinRate);
        const Result<RangeEnvelope> envelope = maxRange(from, shell, env, flat);
        CHECK_MSG(static_cast<bool>(envelope), describe(envelope.status));

        CHECK(envelope->elevation > 0.6 && envelope->elevation < 1.0);
        CHECK(envelope->range > 15000.0 && envelope->range < 30000.0);

        for (double delta : {-0.15, -0.05, 0.05, 0.15}) {
            const Result<Impact> off = predictImpact(
                fromGround(shell.muzzle.speed, envelope->elevation + delta, 0.0, 0.0,
                           shell.muzzle.spinRate),
                shell, env, flat);
            CHECK(static_cast<bool>(off));
            CHECK_MSG(off->range <= envelope->range + 1.0,
                      "elevation offset " + std::to_string(delta));
        }
    }

    // Solving for an aim point and integrating it again must land on the target.
    {
        const LaunchState from = fromGround(shell.muzzle.speed, 0.0, 0.0, 0.0,
                                            shell.muzzle.spinRate);
        const glm::dvec3 target(12000.0, 0.0, 0.0);

        const Result<LaunchAim> aim = solveLaunchDirection(from, target, shell, env, 0.0, 0.7);
        CHECK_MSG(static_cast<bool>(aim), describe(aim.status));
        CHECK(aim->iterations > 0);

        const Result<Impact> check = predictImpact(
            fromGround(shell.muzzle.speed, aim->elevation, aim->azimuthOffset, 0.0,
                       shell.muzzle.spinRate),
            shell, env, flat);
        CHECK(static_cast<bool>(check));
        CHECK_NEAR(check->range, target.x, 20.0);
        CHECK_NEAR(check->crossrange, target.z, 2.0);
    }

    // Both arcs reach the same target, the lofted one taking longer.
    {
        const LaunchState from = fromGround(shell.muzzle.speed, 0.0, 0.0, 0.0,
                                            shell.muzzle.spinRate);
        const AimPair pair = solveLaunchDirections(from, glm::dvec3(10000.0, 0.0, 0.0), shell, env);

        CHECK_MSG(static_cast<bool>(pair.direct), describe(pair.direct.status));
        CHECK_MSG(static_cast<bool>(pair.indirect), describe(pair.indirect.status));
        CHECK(pair.direct->elevation < pair.indirect->elevation);
        CHECK(pair.direct->tof < pair.indirect->tof);
    }

    // A target past maximum range is reported unreachable, not approximated.
    {
        const LaunchState from = fromGround(shell.muzzle.speed, 0.0, 0.0, 0.0,
                                            shell.muzzle.spinRate);
        const Result<LaunchAim> aim = solveLaunchDirection(
            from, glm::dvec3(500000.0, 0.0, 0.0), shell, env, 0.0, 1.5);
        CHECK(!aim);
        CHECK(aim.status == Status::Unreachable);
    }

    // Release point for an airborne drop, then verified by integrating from it.
    {
        Projectile store;
        store.id = "store";
        store.geometry = {250.0, 0.35, 2.0};
        store.drag = DragTable::g7ForCd(0.25);
        store.forces = Force::Gravity | Force::Drag;

        const double altitude = 3000.0;
        const GroundReference ground{-altitude};
        const glm::dvec3 platform(180.0, 0.0, 0.0);
        const glm::dvec3 target(20000.0, -altitude, 1500.0);

        const Result<ReleaseSolution> release = solveReleasePoint(
            platform, altitude, target, store, env, ground);
        CHECK_MSG(static_cast<bool>(release), describe(release.status));

        // The planner primitive has to be cheap: a couple of runs, not a search.
        CHECK_MSG(release->iterations <= 3, "iterations " + std::to_string(release->iterations));
        CHECK(release->residual < 10.0);

        // Releasing there actually hits.
        const Result<Impact> hit = predictImpact(
            fromPlatform(platform, glm::dvec3(0.0), release->position, altitude),
            store, env, ground);
        CHECK(static_cast<bool>(hit));
        CHECK_NEAR(hit->range, target.x, 10.0);
        CHECK_NEAR(hit->crossrange, target.z, 10.0);

        // Release must be short of the target by the ballistic throw.
        CHECK(release->position.x < target.x);
        CHECK(release->tof > 0.0);
    }

    // A faster platform has to release earlier, and a crosswind shifts the release across.
    {
        Projectile store;
        store.id = "store";
        store.geometry = {250.0, 0.35, 2.0};
        store.drag = DragTable::g7ForCd(0.25);
        store.forces = Force::Gravity | Force::Drag;

        const double altitude = 2000.0;
        const GroundReference ground{-altitude};
        const glm::dvec3 target(10000.0, -altitude, 0.0);

        const Result<ReleaseSolution> slow = solveReleasePoint(
            glm::dvec3(120.0, 0.0, 0.0), altitude, target, store, env, ground);
        const Result<ReleaseSolution> fast = solveReleasePoint(
            glm::dvec3(260.0, 0.0, 0.0), altitude, target, store, env, ground);
        CHECK(static_cast<bool>(slow) && static_cast<bool>(fast));
        CHECK(fast->position.x < slow->position.x);

        Environment windy = quiet();
        windy.wind = WindProfile::uniform(20.0, 270.0);
        const Result<ReleaseSolution> drifted = solveReleasePoint(
            glm::dvec3(120.0, 0.0, 0.0), altitude, target, store, windy, ground);
        CHECK(static_cast<bool>(drifted));
        CHECK(drifted->position.z < slow->position.z);
    }

    return test::summary("solve");
}
