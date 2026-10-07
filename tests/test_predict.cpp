#include "ballistics/pm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::PM;

namespace {

Projectile inert(double mass, double diameter) {
    Projectile p;
    p.id = "inert";
    p.geometry = {mass, diameter, 0.5};
    p.drag = DragTable::g7ForCd(0.2);
    p.forces = static_cast<ForceSet>(Force::Gravity);
    return p;
}

Environment quiet() {
    Environment e = Environment::standard();
    e.latitude = 50.0;
    return e;
}

} // namespace

int main() {
    const Environment env = quiet();
    const GroundReference flat{0.0};

    // Gravity alone must reproduce the closed-form parabola.
    {
        const Projectile p = inert(10.0, 0.1);
        const double speed = 100.0;
        const double elevation = 0.6108652381980153;   // 35 degrees

        const Result<Impact> r = predictImpact(fromGround(speed, elevation, 0.0, 0.0), p, env,
                                               flat, SolverConfig::precise());
        CHECK(static_cast<bool>(r));

        const double g = 9.80665;
        CHECK_REL(r->range, speed * speed * std::sin(2.0 * elevation) / g, 1e-3);
        CHECK_REL(r->tof, 2.0 * speed * std::sin(elevation) / g, 1e-3);
        CHECK_NEAR(r->crossrange, 0.0, 1e-9);

        // Symmetric arc: impact angle mirrors launch, speed is recovered.
        CHECK_REL(r->impactAngle, elevation, 1e-3);
        CHECK_REL(r->terminalSpeed, speed, 1e-3);
    }

    // Vertical drop from a hover. Gating impact detection on downrange travel would make this
    // case never register at all.
    {
        Projectile p = inert(20.0, 0.15);
        p.forces = Force::Gravity | Force::Drag;

        const double height = 1000.0;
        const LaunchState hover = fromPlatform(glm::dvec3(0.0), glm::dvec3(0.0),
                                               glm::dvec3(0.0), height);
        const Result<Impact> r = predictImpact(hover, p, env, GroundReference{-height});

        CHECK_MSG(static_cast<bool>(r), describe(r.status));
        CHECK_NEAR(r->range, 0.0, 1e-9);
        CHECK_NEAR(r->crossrange, 0.0, 1e-9);
        CHECK_NEAR(r->altitudeRel, -height, 1e-6);

        // Slower than the vacuum fall and steeper than any lobbed arc.
        const double vacuumTof = std::sqrt(2.0 * height / 9.80665);
        CHECK(r->tof > vacuumTof);
        CHECK_REL(r->impactAngle, 1.5707963267948966, 1e-6);
    }

    // Forward release from a moving platform lands downrange, and faster means further.
    {
        Projectile p = inert(20.0, 0.15);
        p.forces = Force::Gravity | Force::Drag;

        const double height = 1000.0;
        const GroundReference ground{-height};
        double previous = 0.0;

        for (double speed : {50.0, 100.0, 200.0, 300.0}) {
            const LaunchState release = fromPlatform(glm::dvec3(speed, 0.0, 0.0),
                                                     glm::dvec3(0.0), glm::dvec3(0.0), height);
            const Result<Impact> r = predictImpact(release, p, env, ground);
            CHECK_MSG(static_cast<bool>(r), describe(r.status));
            CHECK(r->range > previous);
            previous = r->range;
        }
        CHECK(previous > 1000.0);
    }

    // Lateral platform velocity produces crossrange in the same sense.
    {
        Projectile p = inert(20.0, 0.15);
        p.forces = static_cast<ForceSet>(Force::Gravity);

        const double height = 500.0;
        const GroundReference ground{-height};

        const Result<Impact> right = predictImpact(
            fromPlatform(glm::dvec3(100.0, 0.0, 40.0), glm::dvec3(0.0), glm::dvec3(0.0), height),
            p, env, ground, SolverConfig::precise());
        const Result<Impact> left = predictImpact(
            fromPlatform(glm::dvec3(100.0, 0.0, -40.0), glm::dvec3(0.0), glm::dvec3(0.0), height),
            p, env, ground, SolverConfig::precise());

        CHECK(static_cast<bool>(right) && static_cast<bool>(left));
        CHECK(right->crossrange > 0.0);
        CHECK_NEAR(right->crossrange, -left->crossrange, 1e-6);

        // With gravity only the fall time is fixed, so crossrange is just drift time times speed.
        CHECK_REL(right->crossrange, 40.0 * right->tof, 1e-6);
    }

    // Release position offsets the impact point one for one.
    {
        Projectile p = inert(20.0, 0.15);
        p.forces = Force::Gravity | Force::Drag;

        const glm::dvec3 platform(120.0, 0.0, 0.0);
        const Result<Impact> a = predictImpact(
            fromPlatform(platform, glm::dvec3(0.0), glm::dvec3(0.0, 0.0, 0.0), 800.0),
            p, env, GroundReference{-800.0});
        const Result<Impact> b = predictImpact(
            fromPlatform(platform, glm::dvec3(0.0), glm::dvec3(500.0, 0.0, 250.0), 800.0),
            p, env, GroundReference{-800.0});

        CHECK(static_cast<bool>(a) && static_cast<bool>(b));
        CHECK_REL(b->range - a->range, 500.0, 1e-6);
        CHECK_REL(b->crossrange - a->crossrange, 250.0, 1e-6);
    }

    // A target above the launch point that the shot cannot reach reports no impact
    // rather than a sentinel time of flight.
    {
        const Projectile p = inert(10.0, 0.1);
        const Result<Impact> r = predictImpact(fromGround(50.0, 0.2, 0.0, 0.0), p, env,
                                               GroundReference{5000.0});
        CHECK(!r);
        CHECK(r.status == Status::NoImpact);
    }

    // Convergence: refining dt must move the answer less each time.
    {
        Projectile p = inert(46.5, 0.155);
        p.forces = Force::Gravity | Force::Drag | Force::Coriolis;
        p.drag = DragTable::g7ForCd(0.175);

        const LaunchState launch = fromGround(827.0, 0.7853981633974483, 0.0, 0.0, 17000.0);

        SolverConfig reference = SolverConfig::precise();
        reference.dt = 0.0001;
        const double truth = predictImpact(launch, p, env, flat, reference)->range;

        double previousError = 1e30;
        for (double dt : {0.04, 0.02, 0.01, 0.005}) {
            SolverConfig c = SolverConfig::standard();
            c.dt = dt;
            const double error = std::abs(predictImpact(launch, p, env, flat, c)->range - truth);
            CHECK_MSG(error < previousError,
                      "dt " + std::to_string(dt) + " error " + std::to_string(error));
            previousError = error;
        }

        // The presets must be ordered in accuracy, which is what makes them meaningful.
        const double fast = std::abs(
            predictImpact(launch, p, env, flat, SolverConfig::fast())->range - truth);
        const double standard = std::abs(
            predictImpact(launch, p, env, flat, SolverConfig::standard())->range - truth);
        const double precise = std::abs(
            predictImpact(launch, p, env, flat, SolverConfig::precise())->range - truth);
        CHECK_MSG(fast > standard, "fast " + std::to_string(fast) +
                                       " standard " + std::to_string(standard));
        CHECK_MSG(standard > precise, "standard " + std::to_string(standard) +
                                          " precise " + std::to_string(precise));
    }

    // Sample density follows sampleInterval and does not move the impact point.
    {
        Projectile p = inert(46.5, 0.155);
        p.forces = Force::Gravity | Force::Drag;
        const LaunchState launch = fromGround(400.0, 0.7853981633974483, 0.0, 0.0);

        SolverConfig coarse = SolverConfig::standard();
        coarse.sampleInterval = 0.5;
        SolverConfig fine = SolverConfig::standard();
        fine.sampleInterval = 0.05;

        const Result<Trajectory> a = predict(launch, p, env, flat, coarse);
        const Result<Trajectory> b = predict(launch, p, env, flat, fine);
        CHECK(static_cast<bool>(a) && static_cast<bool>(b));

        CHECK(b->size() > a->size() * 8);
        CHECK_NEAR(a->impact.range, b->impact.range, 1e-9);
        CHECK_NEAR(a->impact.tof, b->impact.tof, 1e-9);

        // Stored samples must be ordered and end on the impact point.
        for (std::size_t i = 1; i < a->size(); ++i) {
            CHECK(a->t[i] > a->t[i - 1]);
        }
        CHECK_NEAR(a->x.back(), a->impact.range, 1e-9);
        CHECK_NEAR(a->y.back(), a->impact.altitudeRel, 1e-9);
    }

    // Energy never rises when only gravity and drag act.
    {
        Projectile p = inert(14.5, 0.12);
        p.forces = Force::Gravity | Force::Drag;
        p.drag = DragTable::g1ForCd(0.18);

        const Result<Trajectory> t = predict(fromGround(332.0, 1.1, 0.0, 0.0), p, env, flat);
        CHECK(static_cast<bool>(t));

        double previous = 1e30;
        for (std::size_t i = 0; i < t->size(); ++i) {
            const double speed2 = t->vx[i] * t->vx[i] + t->vy[i] * t->vy[i] + t->vz[i] * t->vz[i];
            const double energy = 0.5 * p.geometry.mass * speed2 +
                                 p.geometry.mass * 9.80665 * t->y[i];
            CHECK(energy <= previous + 1e-6);
            previous = energy;
        }
    }

    // Missing data is refused rather than silently treated as zero.
    {
        Projectile p = inert(10.0, 0.1);
        p.forces = Force::Gravity | Force::SpinDrift;
        const Result<Impact> r = predictImpact(fromGround(200.0, 0.5, 0.0, 0.0), p, env, flat);
        CHECK(!r);
        CHECK(r.status == Status::MissingData);
    }

    return test::summary("predict");
}
