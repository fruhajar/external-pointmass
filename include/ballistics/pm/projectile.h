#ifndef BALLISTICS_PM_PROJECTILE_H
#define BALLISTICS_PM_PROJECTILE_H

#include "drag.h"
#include "types.h"

#include <cstdint>
#include <optional>
#include <string>

namespace Ballistics::PM {

struct MassGeometry {
    double mass = 0.0;
    double diameter = 0.0;
    double length = 0.0;

    double refArea() const;
};

struct MuzzleData {
    double speed = 0.0;
    double spinRate = 0.0;            // rpm at the muzzle
    double spinDecayHalflife = 0.0;   // seconds; 0 disables decay
};

struct MotorProfile {
    double burnTime = 0.0;
    double thrust = 0.0;              // N, constant over the burn
    double dragFactorDuringBurn = 1.0;
};

// Needed only by the spin-drift term: axial inertia and the two coefficients that set
// the yaw of repose and the lift it produces.
struct SpinAero {
    double Ixx = 0.0;
    double C_M_alpha = 0.0;
    double C_L_alpha = 0.0;
};

enum class Force : std::uint32_t {
    None      = 0,
    Gravity   = 1u << 0,
    Drag      = 1u << 1,
    Coriolis  = 1u << 2,
    Thrust    = 1u << 3,
    SpinDrift = 1u << 4,
};

using ForceSet = std::uint32_t;

constexpr ForceSet operator|(Force a, Force b) {
    return static_cast<ForceSet>(a) | static_cast<ForceSet>(b);
}
constexpr ForceSet operator|(ForceSet a, Force b) { return a | static_cast<ForceSet>(b); }
constexpr bool has(ForceSet set, Force f) { return (set & static_cast<ForceSet>(f)) != 0; }

inline constexpr ForceSet BALLISTIC_FORCES = Force::Gravity | Force::Drag | Force::Coriolis;

// How much to trust a catalogue entry. Calibrated means the drag form factor was fitted to a
// published maximum range; Estimated means class-typical coefficients with no round-specific
// source; Placeholder means the entry is unverified and its numbers carry no authority.
enum class DataQuality { Calibrated, Estimated, Placeholder };

const char* describe(DataQuality q);

struct Projectile {
    std::string id;
    std::string name;
    std::string role;

    MassGeometry geometry;
    MuzzleData muzzle;
    DragTable drag;

    std::optional<MotorProfile> motor = std::nullopt;
    std::optional<SpinAero> spinAero = std::nullopt;

    ForceSet forces = BALLISTIC_FORCES;
    DataQuality quality = DataQuality::Placeholder;
};

// Reports the first block the requested forces need and the projectile does not carry.
Status validate(const Projectile& p);

} // namespace Ballistics::PM

#endif
