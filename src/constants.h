#ifndef BALLISTICS_PM_CONSTANTS_H
#define BALLISTICS_PM_CONSTANTS_H

namespace Ballistics::PM::Constants {

inline constexpr double G = 9.80665;
inline constexpr double R_GAS = 287.05;          // J/(kg K), dry air
inline constexpr double GAMMA = 1.4;
inline constexpr double LAPSE = 0.0065;          // K/m, ISA troposphere
inline constexpr double TROPOPAUSE = 11000.0;    // m
inline constexpr double OMEGA_EARTH = 7.2921159e-5;
inline constexpr double R_EARTH = 6371000.0;

} // namespace Ballistics::PM::Constants

#endif
