// Ground launch: both arcs onto a target, and the range envelope.

#include "ballistics/pm/ballistics.h"

#include <cstdio>

using namespace Ballistics::PM;

int main() {
    const Catalogue catalogue = Catalogue::withBuiltins();
    const Projectile* shell = catalogue.find("Howitzer155mm_HE");
    if (shell == nullptr) {
        return 1;
    }

    Environment env = Environment::standard();
    env.latitude = 50.0;
    env.wind = WindProfile::uniform(8.0, 270.0);

    const GroundReference ground{-50.0};
    const LaunchState gun = fromGround(shell->muzzle.speed, 0.0, 0.0, 300.0,
                                       shell->muzzle.spinRate);

    const Result<RangeEnvelope> envelope = maxRange(gun, *shell, env, ground);
    if (envelope) {
        std::printf("%s reaches %.0f m at %.1f degrees\n\n", shell->name.c_str(),
                    envelope->range, glm::degrees(envelope->elevation));
    }

    const glm::dvec3 target(14000.0, ground.altitudeRel, 0.0);
    const AimPair arcs = solveLaunchDirections(gun, target, *shell, env);

    const auto show = [](const char* label, const Result<LaunchAim>& aim) {
        if (!aim) {
            std::printf("%-9s %s\n", label, describe(aim.status));
            return;
        }
        std::printf("%-9s elevation %6.2f deg, aim off %+6.3f deg, %5.1f s, miss %.2f m, "
                    "%d iterations\n",
                    label, glm::degrees(aim->elevation), glm::degrees(aim->azimuthOffset),
                    aim->tof, aim->residual, aim->iterations);
    };

    std::printf("onto a target at %.0f m:\n", target.x);
    show("direct", arcs.direct);
    show("indirect", arcs.indirect);
    return 0;
}
