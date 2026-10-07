// The yaw-of-repose drift force is
//     F = -(Ixx * p * C_L_alpha) / (d * C_M_alpha * v^2) * cross(v, g)
// so drift scales linearly in Ixx, spin and C_L_alpha, and inversely in C_M_alpha and
// diameter. These checks pin the implementation down independently of the coefficient
// values, which are class-typical rather than measured per round.

#include "ballistics/pm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::PM;

namespace {

Projectile spun() {
    Projectile p;
    p.id = "spun";
    p.geometry = {46.5, 0.155, 0.857};
    p.muzzle = {827.0, 17000.0, 0.0};   // no spin decay, to keep the scaling exact
    p.drag = DragTable::g7ForCd(0.175);
    p.spinAero = SpinAero{0.121, 2.5, 1.2};
    p.forces = Force::Gravity | Force::Drag | Force::SpinDrift;
    return p;
}

Environment quiet() {
    Environment e = Environment::standard();
    e.latitude = 0.0;   // Coriolis off via the force set, latitude kept neutral anyway
    return e;
}

double driftOf(const Projectile& p) {
    const Environment env = quiet();
    const LaunchState launch = fromGround(p.muzzle.speed, 0.7853981633974483, 0.0, 0.0,
                                          p.muzzle.spinRate);
    const Result<Impact> r = predictImpact(launch, p, env, GroundReference{0.0},
                                           SolverConfig::precise());
    return r ? r->crossrange : 0.0;
}

} // namespace

int main() {
    const Projectile base = spun();
    const double reference = driftOf(base);

    // Right-hand spin drifts right, which is +z.
    CHECK(reference > 0.0);

    // Reversing the spin mirrors the drift.
    {
        Projectile left = base;
        left.muzzle.spinRate = -base.muzzle.spinRate;
        CHECK_REL(driftOf(left), -reference, 1e-6);
    }

    // No spin, no drift. Same for a round carrying no spin aerodynamics.
    {
        Projectile still = base;
        still.muzzle.spinRate = 0.0;
        CHECK_NEAR(driftOf(still), 0.0, 1e-9);

        Projectile bare = base;
        bare.forces = Force::Gravity | Force::Drag;
        CHECK_NEAR(driftOf(bare), 0.0, 1e-9);
    }

    // Linear in spin, axial inertia and lift slope, to first order. Factors are kept modest
    // because once the drift is kilometres wide it bends the path enough to feed back.
    for (double factor : {0.25, 0.5, 2.0}) {
        Projectile spin = base;
        spin.muzzle.spinRate *= factor;
        CHECK_REL(driftOf(spin), reference * factor, 2e-3);

        Projectile inertia = base;
        inertia.spinAero->Ixx *= factor;
        CHECK_REL(driftOf(inertia), reference * factor, 2e-3);

        Projectile lift = base;
        lift.spinAero->C_L_alpha *= factor;
        CHECK_REL(driftOf(lift), reference * factor, 2e-3);
    }

    // Inverse in the overturning moment coefficient, the value the drift is most sensitive
    // to, and the one a class-typical value is least certain in.
    for (double factor : {1.6, 2.0, 4.0}) {
        Projectile stiff = base;
        stiff.spinAero->C_M_alpha *= factor;
        CHECK_REL(driftOf(stiff), reference / factor, 2e-3);
    }

    // Drift is independent of air density, because it cancels between the repose angle and
    // the lift it produces. Only the trajectory shape carries any density dependence, so
    // hold the shape fixed by dropping drag.
    {
        Projectile noDrag = base;
        noDrag.forces = Force::Gravity | Force::SpinDrift;

        Environment thin = quiet();
        thin.atmosphere = Atmosphere::fromStation(233.15, 40000.0, 0.0);

        const Environment thick = quiet();
        const LaunchState launch = fromGround(base.muzzle.speed, 0.7853981633974483, 0.0, 0.0,
                                              base.muzzle.spinRate);
        const Result<Impact> a = predictImpact(launch, noDrag, thick, GroundReference{0.0},
                                               SolverConfig::precise());
        const Result<Impact> b = predictImpact(launch, noDrag, thin, GroundReference{0.0},
                                               SolverConfig::precise());
        CHECK(static_cast<bool>(a) && static_cast<bool>(b));
        CHECK_REL(b->crossrange, a->crossrange, 1e-6);
    }

    // Magnitude sits in the band real spin drift occupies, against the several percent an unphysical Magnus term produces.
    // Deliberately loose: the coefficients are uncalibrated, so this is a bound, not a fit.
    {
        const Environment env = quiet();
        const LaunchState launch = fromGround(base.muzzle.speed, 0.7853981633974483, 0.0, 0.0,
                                              base.muzzle.spinRate);
        const Result<Impact> r = predictImpact(launch, base, env, GroundReference{0.0},
                                               SolverConfig::precise());
        CHECK(static_cast<bool>(r));
        const double pct = 100.0 * r->crossrange / r->range;
        CHECK_MSG(pct > 0.1 && pct < 6.0, "drift " + std::to_string(pct) + "% of range");
    }

    return test::summary("drift");
}
