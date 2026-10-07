# ballistics-pointmass

A compact point-mass trajectory library for short and mid-range impact prediction, launched from
the ground or from a moving airborne platform. C++20, depends on glm and the standard library and
nothing else. Around 1,700 lines.

If you need interchangeable models, spin drift from integrated attitude, or a choice between
accuracy and cost, use [ballistics-multimodel](https://github.com/fruhajar/external-multimodel) instead. This library
is the one to reach for when a point mass is the right model: short flights, unspun stores, or
predicting where a release lands rather than computing a firing table.

## Building

Needs a C++20 compiler and [glm](https://github.com/g-truc/glm), which must be discoverable by
`find_package(glm CONFIG)`. On Debian or Ubuntu that is `apt install libglm-dev`; otherwise build
glm from source and point `CMAKE_PREFIX_PATH` at where you installed it. Without it, configuring
fails with `Could not find a package configuration file provided by "glm"`.

Tested with GCC 12 and Clang 14, shared and static, clean under AddressSanitizer and
UndefinedBehaviorSanitizer.

```sh
cmake -S . -B build -DBALLISTICS_PM_BUILD_TESTS=ON -DBALLISTICS_PM_BUILD_EXAMPLES=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Shared by default; `-DBUILD_SHARED_LIBS=OFF` gives a static library. Consuming it:

```cmake
find_package(BallisticsPointMass REQUIRED)
target_link_libraries(app PRIVATE Ballistics::PointMass)
```

## What it does

Forward prediction, and the inverse problems a mission planner actually asks:

```cpp
predictImpact(launch, projectile, env, ground, config);   // where does this land
predict(...);                                            // same, keeping the trajectory
solveReleasePoint(platformVelocity, altitude, target, ...);   // where to let go
solveLaunchDirection(from, target, ..., minElev, maxElev);     // where to aim
solveLaunchDirections(from, target, ...);                     // both arcs
maxRange(from, ...);                                          // the envelope
```

`solveReleasePoint` is built for being called in a loop. From a fixed release velocity and
altitude the fall is deterministic, so the release point is the target minus the ballistic
displacement; it converges in two or three integrations rather than searching.

A launch is a full position and velocity vector, so a release from a moving platform at altitude
is expressible:

```cpp
fromGround(speed, elevation, azimuthOffset, altitudeMsl, spinRate);
fromPlatform(platformVelocity, releaseVelocity, position, altitudeMsl);
```

Everything returns `Result<T>` carrying a `Status`, so an unreachable target, a projectile missing
data its force set needs, and a shot that never comes down are distinguishable. No sentinel values.

## Conventions

**Frame.** Local, launch-centred, left-handed: `+x` downrange along the launch azimuth, `+y` up,
`+z` crossrange to the right of downrange. The launch bearing lives in
`Environment::launchAzimuth`, so a solution is an offset from where the launcher already points.
`toGeo` projects a finished trajectory onto the globe.

**Units.** Metres, seconds, kilograms, radians, kelvin, pascals. Radians internally; degrees only
for compass bearings, which are always named `...azimuth`, `...bearing` or suffixed `Deg`.

**Altitude.** Three distinct things, kept apart by name. `altitudeMsl` feeds the atmosphere.
`altitudeRel` is relative to the launch point, which is what a trajectory's `y` carries.
`GroundReference::altitudeRel` is where the impact surface sits relative to the launch point, so
it is negative when dropping or firing onto lower ground.

**Wind.** `WindLayer::fromBearing` is where the wind blows *from*, the met convention. Layers
interpolate by speed and bearing rather than as vectors, so a veering wind keeps its speed.

**Spin.** Rpm at the boundary, positive for right-handed. Right-handed spin drifts right.

## Precision

`SolverConfig` holds every knob, with three presets. Nothing is a compile-time constant.

| | `fast()` | `standard()` | `precise()` |
|---|---|---|---|
| Integrator | RK2 | RK4 | RK4 |
| Timestep | 20 ms | 5 ms | 0.5 ms |
| Sample interval | 100 ms | 25 ms | 10 ms |

`sampleInterval` is in seconds, not steps, so changing the timestep does not change how much
trajectory you get back. Use `fast()` inside anything that iterates; the inverse solvers take a
config and pass it down.

## Accuracy, honestly

The catalogue holds nine rounds and every one is calibrated: published mass, diameter, length and
muzzle velocity, with the drag form factor fitted to a published maximum range, which it reproduces
to better than 0.5%. A round that could not be anchored to a published figure was left out rather
than shipped with an estimate standing in for data.

The fit checks itself. A form factor can absorb an error in the muzzle velocity instead of
describing the drag, so the implied Cd at rest is checked against the band a mortar bomb or a shell
physically occupies. That bound removed a round whose quoted range and velocity did not belong to
the same charge.

Aerodynamic coefficients are class-typical rather than measured per round, but they are not free:
`C_M_alpha` is boxed into 3.5 to 4.0 by the gyroscopic stability the round must have, and the tests
assert `1 < Sg < 3` for every spin-stabilised entry. Spin rate comes from the twist rate, axial
inertia from a measured mass ratio, and spin decay from roll damping.

Drift uses the STANAG 4355 yaw-of-repose form and runs from about 1% of range at 15 degrees to 6% at
60 degrees. For corroboration: a 7.62x51 M80 bullet at 32 degrees drifts about 2.4% of range, and
this model gives 2.15% at 30 degrees. `tests/test_drift.cpp` pins the implementation to its scaling
laws independently of any coefficient value.

**This is not firing-table data and the libraries are not validated for operational fire control.**
Range and time of flight are anchored to published figures; drift has the right form, sign and
order but rests on class-typical coefficients. If you hold measured aeroballistic data,
`Projectile::spinAero` and the drag model are per-round inputs for exactly that reason.

`docs/catalogue-review.md` records every number, its source, and what the catalogue is not.
`docs/extensibility.md` records what a declared input contract measured against this library.


## Status

Complete and verified: 10 tests, 3,814 assertions, published and frozen. `main` only advances
through a pull request. No further development is planned unless something turns out to be wrong.
