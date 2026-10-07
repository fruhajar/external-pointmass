#include "ballistics/pm/drag.h"

#include "test_support.h"

using namespace Ballistics::PM;

int main() {
    const DragTable g1 = DragTable::g1(1.0);
    const DragTable g7 = DragTable::g7(1.0);

    CHECK(!g1.empty());
    CHECK(!g7.empty());

    // Exact table entries, including the ends where interpolation degrades to linear.
    CHECK_NEAR(g1.at(0.0), 0.2629, 1e-12);
    CHECK_NEAR(g1.at(5.0), 0.4890, 1e-12);
    CHECK_NEAR(g7.at(0.0), 0.1198, 1e-12);
    CHECK_NEAR(g7.at(5.0), 0.3656, 1e-12);

    // Clamped outside the table rather than extrapolated.
    CHECK_NEAR(g1.at(-1.0), 0.2629, 1e-12);
    CHECK_NEAR(g1.at(50.0), 0.4890, 1e-12);

    // Both families peak in the transonic climb and stay bounded by their table extremes.
    for (double m = 0.0; m <= 6.0; m += 0.01) {
        CHECK(g1.at(m) > 0.15 && g1.at(m) < 0.60);
        CHECK(g7.at(m) > 0.10 && g7.at(m) < 0.40);
    }
    CHECK(g1.at(1.5) > g1.at(0.5));
    CHECK(g7.at(3.0) > g7.at(0.5));

    // Form factor scales linearly and the Cd helpers invert the reference value.
    CHECK_NEAR(DragTable::g1(2.0).at(1.0), 2.0 * g1.at(1.0), 1e-12);
    CHECK_NEAR(DragTable::g1ForCd(0.20).at(0.0), 0.20, 1e-12);
    CHECK_NEAR(DragTable::g7ForCd(0.175).at(0.0), 0.175 * 0.1198 / 0.1197, 1e-12);

    // A custom table behaves like the built-ins.
    const DragTable flat = DragTable::custom({0.0, 1.0, 2.0}, {0.3, 0.3, 0.3});
    CHECK_NEAR(flat.at(0.5), 0.3, 1e-12);
    CHECK_NEAR(flat.at(1.7), 0.3, 1e-12);

    // Mismatched lengths are rejected rather than half-accepted.
    CHECK(DragTable::custom({0.0, 1.0}, {0.3}).empty());

    return test::summary("drag");
}
