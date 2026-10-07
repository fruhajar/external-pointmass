#include "forces.h"

#include "constants.h"

#include <cmath>

namespace Ballistics::PM {

using namespace Constants;

namespace {

constexpr double RPM_TO_RAD_S = 2.0 * 3.14159265358979323846 / 60.0;
constexpr double STILL_AIR = 0.1;   // m/s below which aerodynamic terms are dropped

double spinRateRadPerSec(const MuzzleData& muzzle, double launchSpinRpm, double t) {
    double rpm = launchSpinRpm;
    if (muzzle.spinDecayHalflife > 0.0) {
        rpm *= std::exp(-0.6931471805599453 * t / muzzle.spinDecayHalflife);
    }
    return rpm * RPM_TO_RAD_S;
}

glm::dvec3 gravity(double altitudeMsl) {
    const double ratio = R_EARTH / (R_EARTH + altitudeMsl);
    return glm::dvec3(0.0, -G * ratio * ratio, 0.0);
}

// Earth rotation, resolved through the launch azimuth into the local frame.
glm::dvec3 coriolis(const glm::dvec3& vel, double latitudeRad, double azimuthRad) {
    const double sinLat = std::sin(latitudeRad);
    const double cosLat = std::cos(latitudeRad);
    const double sinAz = std::sin(azimuthRad);
    const double cosAz = std::cos(azimuthRad);

    const double east = vel.x * sinAz + vel.z * cosAz;
    const double north = vel.x * cosAz - vel.z * sinAz;

    const double aEast = 2.0 * OMEGA_EARTH * (north * sinLat - vel.y * cosLat);
    const double aNorth = -2.0 * OMEGA_EARTH * east * sinLat;
    const double aUp = 2.0 * OMEGA_EARTH * east * cosLat;

    return glm::dvec3(aEast * sinAz + aNorth * cosAz, aUp, aEast * cosAz - aNorth * sinAz);
}

// Yaw of repose drift, MPMM form (STANAG 4355). The gyroscopic response to the overturning
// moment holds the nose slightly off the trajectory, and the lift from that angle is what
// pushes a spun projectile sideways. Air density cancels between the two terms.
glm::dvec3 spinDrift(const glm::dvec3& velRel, const SpinAero& aero, double diameter,
                     double spin, double altitudeMsl) {
    const double v = glm::length(velRel);
    if (v < 1.0 || aero.C_M_alpha <= 0.0 || spin == 0.0) {
        return glm::dvec3(0.0);
    }

    const double scale = -(aero.Ixx * spin * aero.C_L_alpha) /
                         (diameter * aero.C_M_alpha * v * v);
    return scale * glm::cross(velRel, gravity(altitudeMsl));
}

} // namespace

glm::dvec3 acceleration(const glm::dvec3& pos, const glm::dvec3& vel,
                        const StepContext& ctx, double t) {
    const Projectile& p = *ctx.projectile;
    const Environment& env = *ctx.env;
    const ForceSet forces = p.forces;

    const double altitudeMsl = ctx.baseAltitude + pos.y;
    glm::dvec3 force(0.0);

    if (has(forces, Force::Gravity)) {
        force += p.geometry.mass * gravity(altitudeMsl);
    }

    if (has(forces, Force::Coriolis)) {
        force += p.geometry.mass * coriolis(vel, glm::radians(env.latitude),
                                            glm::radians(env.launchAzimuth));
    }

    const glm::dvec3 velRel = vel - env.wind.at(pos.y - ctx.groundRel);
    const double speed = glm::length(velRel);

    if (speed >= STILL_AIR) {
        const double rho = env.atmosphere.density(altitudeMsl);

        if (has(forces, Force::Drag)) {
            const double mach = speed / env.atmosphere.speedOfSound(altitudeMsl);
            double cd = p.drag.at(mach);

            if (p.motor && t < p.motor->burnTime) {
                cd *= p.motor->dragFactorDuringBurn;
            }
            force += -0.5 * rho * cd * p.geometry.refArea() * speed * velRel;
        }

        if (has(forces, Force::Thrust) && p.motor && t <= p.motor->burnTime) {
            force += p.motor->thrust * (velRel / speed);
        }

        if (has(forces, Force::SpinDrift) && p.spinAero) {
            force += spinDrift(velRel, *p.spinAero, p.geometry.diameter,
                               spinRateRadPerSec(p.muzzle, ctx.launchSpin, t), altitudeMsl);
        }
    }

    return force / p.geometry.mass;
}

} // namespace Ballistics::PM
