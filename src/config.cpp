#include "ballistics/pm/config.h"

namespace Ballistics::PM {

SolverConfig SolverConfig::fast() {
    SolverConfig c;
    c.integrator = Integrator::RK2;
    c.dt = 0.02;
    c.sampleInterval = 0.1;
    return c;
}

SolverConfig SolverConfig::standard() {
    return SolverConfig{};
}

SolverConfig SolverConfig::precise() {
    SolverConfig c;
    c.dt = 0.0005;
    c.sampleInterval = 0.01;
    c.rangeTolerance = 1.0;
    c.crossrangeTolerance = 0.25;
    return c;
}

} // namespace Ballistics::PM
