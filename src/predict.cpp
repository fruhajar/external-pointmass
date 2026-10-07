#include "ballistics/pm/predict.h"

#include "forces.h"

#include <cmath>

namespace Ballistics::PM {

namespace {

struct State {
    glm::dvec3 pos{0.0};
    glm::dvec3 vel{0.0};
};

State derivative(const State& s, const StepContext& ctx, double t) {
    return {s.vel, acceleration(s.pos, s.vel, ctx, t)};
}

State advance(const State& s, const State& d, double h) {
    return {s.pos + d.pos * h, s.vel + d.vel * h};
}

void step(State& s, double dt, Integrator integrator, const StepContext& ctx, double t) {
    const State k1 = derivative(s, ctx, t);

    if (integrator == Integrator::RK2) {
        const State k2 = derivative(advance(s, k1, dt * 0.5), ctx, t + dt * 0.5);
        s = advance(s, k2, dt);
        return;
    }

    const State k2 = derivative(advance(s, k1, dt * 0.5), ctx, t + dt * 0.5);
    const State k3 = derivative(advance(s, k2, dt * 0.5), ctx, t + dt * 0.5);
    const State k4 = derivative(advance(s, k3, dt), ctx, t + dt);

    const double sixth = dt / 6.0;
    s.pos += (k1.pos + 2.0 * k2.pos + 2.0 * k3.pos + k4.pos) * sixth;
    s.vel += (k1.vel + 2.0 * k2.vel + 2.0 * k3.vel + k4.vel) * sixth;
}

Impact makeImpact(double t, const glm::dvec3& pos, const glm::dvec3& vel) {
    Impact i;
    i.tof = t;
    i.range = pos.x;
    i.crossrange = pos.z;
    i.altitudeRel = pos.y;
    i.velocity = vel;
    i.terminalSpeed = glm::length(vel);
    if (i.terminalSpeed > 1e-9) {
        i.impactAngle = -std::asin(vel.y / i.terminalSpeed);
    }
    return i;
}

Result<Trajectory> run(const LaunchState& launch, const Projectile& projectile,
                       const Environment& env, const GroundReference& ground,
                       const SolverConfig& config, bool store) {
    if (const Status s = validate(projectile); s != Status::Ok) {
        return Result<Trajectory>::fail(s);
    }
    if (config.dt <= 0.0 || config.maxFlightTime <= 0.0) {
        return Result<Trajectory>::fail(Status::InvalidInput);
    }

    Environment local = env;
    local.wind.orientTo(env.launchAzimuth);

    StepContext ctx;
    ctx.projectile = &projectile;
    ctx.env = &local;
    ctx.baseAltitude = launch.altitudeMsl - launch.position.y;
    ctx.groundRel = ground.altitudeRel;
    ctx.launchSpin = launch.spinRate;

    Trajectory out;
    if (store) {
        const double span = config.sampleInterval > 0.0 ? config.sampleInterval : config.dt;
        out.reserve(static_cast<std::size_t>(config.maxFlightTime / span) + 2);
    }

    State s{launch.position, launch.velocity};
    double t = 0.0;
    double nextSample = 0.0;

    while (t < config.maxFlightTime) {
        if (store && t >= nextSample) {
            out.append(t, s.pos, s.vel);
            nextSample += config.sampleInterval;
        }

        const State prev = s;
        step(s, config.dt, config.integrator, ctx, t);
        t += config.dt;

        // Descending through the ground plane is the only impact condition; a vertical drop
        // has to register just as a lobbed shot does.
        const bool descending = s.vel.y < 0.0;
        const bool crossed = prev.pos.y >= ground.altitudeRel && s.pos.y < ground.altitudeRel;

        if (descending && crossed) {
            const double dy = s.pos.y - prev.pos.y;
            double frac = 0.0;
            if (std::abs(dy) > 1e-12) {
                frac = (ground.altitudeRel - prev.pos.y) / dy;
            }

            const glm::dvec3 pos = prev.pos + frac * (s.pos - prev.pos);
            const glm::dvec3 vel = prev.vel + frac * (s.vel - prev.vel);
            const double tImpact = t - config.dt + frac * config.dt;

            out.impact = makeImpact(tImpact, pos, vel);
            if (store) {
                out.append(tImpact, pos, vel);
            }
            return Result<Trajectory>::ok(std::move(out));
        }

        if (glm::length(s.vel) < 0.01) {
            break;
        }
    }

    return Result<Trajectory>::fail(Status::NoImpact);
}

} // namespace

Result<Impact> predictImpact(const LaunchState& launch, const Projectile& projectile,
                             const Environment& env, const GroundReference& ground,
                             const SolverConfig& config) {
    const Result<Trajectory> r = run(launch, projectile, env, ground, config, false);
    return {r.status, r.value.impact};
}

Result<Trajectory> predict(const LaunchState& launch, const Projectile& projectile,
                           const Environment& env, const GroundReference& ground,
                           const SolverConfig& config) {
    return run(launch, projectile, env, ground, config, true);
}

} // namespace Ballistics::PM
