// Emits this library's results for the multimodel repository to check itself against.
//
// The two libraries are one design: multimodel's point-mass model must reproduce this one to
// integration noise, or putting the model behind a seam changed the physics. They live in separate
// repositories, so the check travels as a recorded file rather than a link-time dependency, the
// same way a recorded reference does.
//
// Values are printed at full double precision, so the comparison measures the two libraries
// rather than this file's rounding.
//
// Regenerate and copy into external-multimodel/tests/data/ only if this library's physics
// changes:
//     cmake --build <build> --target dump_reference
//     <build>/dump_reference > ../external-multimodel/tests/data/pointmass-reference.csv

#include "ballistics/pm/ballistics.h"

#include <cstdio>

using namespace Ballistics::PM;

int main() {
    const Catalogue catalogue = Catalogue::withBuiltins();

    Environment env = Environment::standard();
    env.latitude = 50.0;

    std::printf("# ballistics-pointmass %s, SolverConfig::precise(), latitude 50, calm, ISA\n",
                VERSION);
    std::printf("# forces: gravity, drag, Coriolis. Spin drift excluded: multimodel puts it on\n");
    std::printf("# MPMM, so what is compared here is the shared engine.\n");
    std::printf("id,elevation_deg,range_m,tof_s,crossrange_m,terminal_speed_ms,impact_angle_deg\n");

    for (const std::string& id : catalogue.ids()) {
        const Projectile* round = catalogue.find(id);

        Projectile bare = *round;
        bare.forces = Force::Gravity | Force::Drag | Force::Coriolis;

        for (double deg : {15.0, 30.0, 45.0, 60.0}) {
            const Result<Impact> r = predictImpact(
                fromGround(round->muzzle.speed, glm::radians(deg), 0.0, 0.0), bare, env,
                GroundReference{0.0}, SolverConfig::precise());
            if (!r) {
                std::printf("%s,%.1f,,,,,\n", id.c_str(), deg);
                continue;
            }
            std::printf("%s,%.1f,%.17g,%.17g,%.17g,%.17g,%.17g\n", id.c_str(), deg, r->range, r->tof,
                        r->crossrange, r->terminalSpeed, glm::degrees(r->impactAngle));
        }
    }
    return 0;
}
