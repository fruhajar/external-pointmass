#include "ballistics/pm/atmosphere.h"

#include "test_support.h"

using namespace Ballistics::PM;

int main() {
    const Atmosphere isa;

    // ISA reference values.
    CHECK_REL(isa.density(0.0), 1.2250, 1e-3);
    CHECK_REL(isa.density(5000.0), 0.73643, 2e-3);
    CHECK_REL(isa.density(11000.0), 0.36480, 3e-3);
    CHECK_REL(isa.density(20000.0), 0.08891, 1e-2);

    CHECK_REL(isa.speedOfSound(0.0), 340.29, 1e-3);
    CHECK_REL(isa.speedOfSound(11000.0), 295.07, 2e-3);

    CHECK_NEAR(isa.temperature(0.0), 288.15, 1e-9);
    CHECK_NEAR(isa.temperature(11000.0), 216.65, 1e-9);
    CHECK_NEAR(isa.temperature(20000.0), 216.65, 1e-9);

    // Density must fall monotonically.
    double previous = isa.density(0.0);
    for (double h = 250.0; h <= 25000.0; h += 250.0) {
        const double d = isa.density(h);
        CHECK(d < previous);
        previous = d;
    }

    // A station reading reproduces itself when read back at its own altitude.
    const Atmosphere hot = Atmosphere::fromStation(303.15, 95000.0, 500.0);
    CHECK_NEAR(hot.temperature(500.0), 303.15, 1e-9);
    CHECK_REL(hot.density(500.0), 95000.0 / (287.05 * 303.15), 1e-9);

    // Hotter and thinner than standard at the same height.
    CHECK(hot.density(500.0) < isa.density(500.0));

    return test::summary("atmosphere");
}
