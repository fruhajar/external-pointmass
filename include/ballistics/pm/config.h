#ifndef BALLISTICS_PM_CONFIG_H
#define BALLISTICS_PM_CONFIG_H

namespace Ballistics::PM {

enum class Integrator { RK2, RK4 };

struct SolverConfig {
    Integrator integrator = Integrator::RK4;
    double dt = 0.005;
    double maxFlightTime = 600.0;

    // Seconds between stored samples, so output density does not move when dt does.
    double sampleInterval = 0.025;
    bool storeTrajectory = false;

    double rangeTolerance = 10.0;
    double crossrangeTolerance = 1.0;
    int maxElevationIterations = 120;
    int maxAzimuthIterations = 30;
    double azimuthDamping = 0.7;

    static SolverConfig fast();
    static SolverConfig standard();
    static SolverConfig precise();
};

} // namespace Ballistics::PM

#endif
