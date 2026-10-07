#ifndef BALLISTICS_PM_WIND_H
#define BALLISTICS_PM_WIND_H

#include "types.h"

namespace Ballistics::PM {

// One measurement in a sounding: altitude AGL, speed, and the bearing the wind blows FROM
// (met convention, degrees).
struct WindLayer {
    double altitude = 0.0;
    double speed = 0.0;
    double fromBearing = 0.0;
};

// Layers are interpolated linearly by altitude and held constant above the top one.
// An airborne release sits above anything a fixed three-band scheme can express, so the
// layer count is open.
class WindProfile {
public:
    WindProfile() = default;

    static WindProfile calm();
    static WindProfile uniform(double speed, double fromBearing);
    static WindProfile sounding(std::vector<WindLayer> layers);

    // Resolves into the local frame; launchAzimuth is the firing bearing in degrees.
    void orientTo(double launchAzimuthDeg);

    glm::dvec3 at(double altitudeAgl) const;
    bool empty() const { return m_layers.empty(); }

private:
    std::vector<WindLayer> m_layers;
    double m_azimuthDeg = 0.0;
};

} // namespace Ballistics::PM

#endif
