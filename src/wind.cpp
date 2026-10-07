#include "ballistics/pm/wind.h"

#include <algorithm>

namespace Ballistics::PM {

WindProfile WindProfile::calm() {
    return uniform(0.0, 0.0);
}

WindProfile WindProfile::uniform(double speed, double fromBearing) {
    WindProfile p;
    p.m_layers.push_back({0.0, speed, fromBearing});
    return p;
}

WindProfile WindProfile::sounding(std::vector<WindLayer> layers) {
    WindProfile p;
    p.m_layers = std::move(layers);
    std::sort(p.m_layers.begin(), p.m_layers.end(),
              [](const WindLayer& a, const WindLayer& b) { return a.altitude < b.altitude; });
    return p;
}

void WindProfile::orientTo(double launchAzimuthDeg) {
    m_azimuthDeg = launchAzimuthDeg;
}

glm::dvec3 WindProfile::at(double altitudeAgl) const {
    if (m_layers.empty()) {
        return glm::dvec3(0.0);
    }

    double speed = 0.0;
    double fromBearing = 0.0;

    if (m_layers.size() == 1 || altitudeAgl <= m_layers.front().altitude) {
        speed = m_layers.front().speed;
        fromBearing = m_layers.front().fromBearing;
    } else if (altitudeAgl >= m_layers.back().altitude) {
        speed = m_layers.back().speed;
        fromBearing = m_layers.back().fromBearing;
    } else {
        const auto upper = std::lower_bound(
            m_layers.begin(), m_layers.end(), altitudeAgl,
            [](const WindLayer& l, double h) { return l.altitude < h; });
        const auto lower = upper - 1;

        const double span = upper->altitude - lower->altitude;
        const double blend = span > 0.0 ? (altitudeAgl - lower->altitude) / span : 0.0;

        // Interpolate the bearing the short way round so a veer through north behaves.
        double delta = upper->fromBearing - lower->fromBearing;
        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;

        speed = lower->speed + blend * (upper->speed - lower->speed);
        fromBearing = lower->fromBearing + blend * delta;
    }

    // Met bearing is where the wind comes from; add 180 to get where it pushes.
    const double toward = glm::radians(fromBearing + 180.0 - m_azimuthDeg);
    return glm::dvec3(speed * std::cos(toward), 0.0, speed * std::sin(toward));
}

} // namespace Ballistics::PM
