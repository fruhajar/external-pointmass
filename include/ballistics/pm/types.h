#ifndef BALLISTICS_PM_TYPES_H
#define BALLISTICS_PM_TYPES_H

// Local frame: +x downrange along the launch azimuth, +y up, +z crossrange (right of downrange).
// Left-handed, matching GLM_FORCE_LEFT_HANDED. Metres, seconds, kilograms, radians.
// Degrees appear only where a caller supplies a compass bearing.

#include <glm/ext/vector_double3.hpp>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

#include <cmath>
#include <vector>

namespace Ballistics::PM {

enum class Status {
    Ok,
    MissingData,     // projectile lacks a block the requested force set needs
    InvalidInput,
    NoImpact,        // ran past maxFlightTime without crossing the ground reference
    Unreachable,     // target outside the achievable envelope
    NotConverged,
};

template <class T>
struct Result {
    Status status = Status::Ok;
    T value{};

    explicit operator bool() const { return status == Status::Ok; }
    const T* operator->() const { return &value; }
    const T& operator*() const { return value; }

    static Result ok(T v) { return {Status::Ok, std::move(v)}; }
    static Result fail(Status s) { return {s, T{}}; }
};

const char* describe(Status s);

struct Impact {
    double tof = 0.0;
    double range = 0.0;          // downrange, +x
    double crossrange = 0.0;     // +z
    double altitudeRel = 0.0;    // relative to the launch point
    glm::dvec3 velocity{0.0};
    double impactAngle = 0.0;    // radians below horizontal, positive downward
    double terminalSpeed = 0.0;
};

// Columns rather than an array of points: the sampled fields are always the same
// set here, and callers typically want one series at a time.
struct Trajectory {
    std::vector<double> t, x, y, z, vx, vy, vz;
    Impact impact;

    std::size_t size() const { return t.size(); }
    bool empty() const { return t.empty(); }
    void reserve(std::size_t n);
    void append(double time, const glm::dvec3& pos, const glm::dvec3& vel);
};

struct GeoCoordinate {
    double latitude = 0.0;   // degrees
    double longitude = 0.0;  // degrees
    double altitude = 0.0;   // metres MSL

    double azimuthTo(const GeoCoordinate& other) const;
    double distanceTo(const GeoCoordinate& other) const;
};

// Projects a local-frame trajectory onto the globe. azimuth is the launch bearing in degrees.
std::vector<GeoCoordinate> toGeo(const Trajectory& local,
                                 const GeoCoordinate& origin,
                                 double azimuthDeg);

} // namespace Ballistics::PM

#endif
