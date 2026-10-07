#include "ballistics/pm/wind.h"

#include "test_support.h"

using namespace Ballistics::PM;

int main() {
    CHECK(WindProfile::calm().at(0.0) == glm::dvec3(0.0));
    CHECK(WindProfile().empty());
    CHECK(WindProfile().at(500.0) == glm::dvec3(0.0));

    // A tailwind for a shot fired downwind: wind from 180, launch azimuth 0, so it pushes
    // along +x (downrange).
    WindProfile tail = WindProfile::uniform(10.0, 180.0);
    tail.orientTo(0.0);
    CHECK_NEAR(tail.at(0.0).x, 10.0, 1e-9);
    CHECK_NEAR(tail.at(0.0).z, 0.0, 1e-9);

    // Same wind, shot fired into it.
    WindProfile head = WindProfile::uniform(10.0, 0.0);
    head.orientTo(0.0);
    CHECK_NEAR(head.at(0.0).x, -10.0, 1e-9);

    // Wind from the west pushes toward the east, which is +z when firing north.
    WindProfile cross = WindProfile::uniform(10.0, 270.0);
    cross.orientTo(0.0);
    CHECK_NEAR(cross.at(0.0).x, 0.0, 1e-9);
    CHECK_NEAR(cross.at(0.0).z, 10.0, 1e-9);

    // Rotating the launch azimuth onto the wind turns a crosswind into a tailwind.
    cross.orientTo(90.0);
    CHECK_NEAR(cross.at(0.0).x, 10.0, 1e-9);
    CHECK_NEAR(cross.at(0.0).z, 0.0, 1e-9);

    // A sounding holds the end values and interpolates between.
    WindProfile shear = WindProfile::sounding({{250.0, 5.0, 180.0},
                                               {500.0, 15.0, 180.0},
                                               {750.0, 25.0, 180.0}});
    shear.orientTo(0.0);
    CHECK_NEAR(shear.at(0.0).x, 5.0, 1e-9);
    CHECK_NEAR(shear.at(250.0).x, 5.0, 1e-9);
    CHECK_NEAR(shear.at(375.0).x, 10.0, 1e-9);
    CHECK_NEAR(shear.at(500.0).x, 15.0, 1e-9);
    CHECK_NEAR(shear.at(750.0).x, 25.0, 1e-9);
    CHECK_NEAR(shear.at(9000.0).x, 25.0, 1e-9);

    // Layers given out of order are sorted.
    WindProfile jumbled = WindProfile::sounding({{750.0, 25.0, 180.0},
                                                 {250.0, 5.0, 180.0},
                                                 {500.0, 15.0, 180.0}});
    jumbled.orientTo(0.0);
    CHECK_NEAR(jumbled.at(375.0).x, 10.0, 1e-9);

    // A veer through north takes the short way round, not the long way through south.
    WindProfile veer = WindProfile::sounding({{0.0, 10.0, 350.0}, {1000.0, 10.0, 10.0}});
    veer.orientTo(0.0);
    const glm::dvec3 mid = veer.at(500.0);
    CHECK_NEAR(mid.x, -10.0, 1e-9);
    CHECK_NEAR(mid.z, 0.0, 1e-9);

    // Speed is preserved through interpolation of the bearing.
    for (double h = 0.0; h <= 1000.0; h += 50.0) {
        CHECK_NEAR(glm::length(veer.at(h)), 10.0, 1e-9);
    }

    return test::summary("wind");
}
