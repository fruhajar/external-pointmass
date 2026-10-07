#include "ballistics/pm/atmosphere.h"

#include "constants.h"

#include <cmath>

namespace Ballistics::PM {

using namespace Constants;

namespace {

// Exponent of the ISA troposphere pressure relation.
const double PRESSURE_EXPONENT = G / (R_GAS * LAPSE);

} // namespace

double Atmosphere::temperature(double altitudeMsl) const {
    // Above the tropopause the lapse stops; hold the temperature the profile reached there
    // so a non-standard sea-level reading stays self-consistent all the way up.
    const double h = altitudeMsl < TROPOPAUSE ? altitudeMsl : TROPOPAUSE;
    return seaLevelTemperature - LAPSE * h;
}

double Atmosphere::density(double altitudeMsl) const {
    const double t = temperature(altitudeMsl);
    double pressure = seaLevelPressure * std::pow(t / seaLevelTemperature, PRESSURE_EXPONENT);

    if (altitudeMsl > TROPOPAUSE) {
        pressure *= std::exp(-G * (altitudeMsl - TROPOPAUSE) / (R_GAS * t));
    }
    return pressure / (R_GAS * t);
}

double Atmosphere::speedOfSound(double altitudeMsl) const {
    return std::sqrt(GAMMA * R_GAS * temperature(altitudeMsl));
}

Atmosphere Atmosphere::fromStation(double temperatureK, double pressurePa, double stationAltitude) {
    Atmosphere a;
    a.seaLevelTemperature = temperatureK + LAPSE * stationAltitude;
    a.seaLevelPressure =
        pressurePa * std::pow(a.seaLevelTemperature / temperatureK, PRESSURE_EXPONENT);
    return a;
}

} // namespace Ballistics::PM
