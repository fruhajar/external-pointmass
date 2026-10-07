#ifndef BALLISTICS_PM_ATMOSPHERE_H
#define BALLISTICS_PM_ATMOSPHERE_H

namespace Ballistics::PM {

// ISA below 25 km, with the sea-level reference optionally replaced by a station reading.
// Setting station values shifts the whole profile rather than only the launch altitude,
// which is the usual first-order met correction.
struct Atmosphere {
    double seaLevelTemperature = 288.15;  // K
    double seaLevelPressure = 101325.0;   // Pa

    double density(double altitudeMsl) const;
    double speedOfSound(double altitudeMsl) const;
    double temperature(double altitudeMsl) const;

    // Derives the sea-level reference from a reading taken at a known altitude.
    static Atmosphere fromStation(double temperatureK, double pressurePa, double stationAltitude);
};

} // namespace Ballistics::PM

#endif
