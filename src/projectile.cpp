#include "ballistics/pm/projectile.h"

namespace Ballistics::PM {

double MassGeometry::refArea() const {
    return 3.14159265358979323846 * diameter * diameter * 0.25;
}

const char* describe(DataQuality q) {
    switch (q) {
    case DataQuality::Calibrated:  return "calibrated against a published maximum range";
    case DataQuality::Estimated:   return "class-typical coefficients, no round-specific source";
    case DataQuality::Placeholder: return "unverified, carries no authority";
    }
    return "unknown";
}

Status validate(const Projectile& p) {
    if (p.geometry.mass <= 0.0 || p.geometry.diameter <= 0.0) {
        return Status::InvalidInput;
    }
    if (has(p.forces, Force::Drag) && p.drag.empty()) {
        return Status::MissingData;
    }
    if (has(p.forces, Force::Thrust) && !p.motor) {
        return Status::MissingData;
    }
    if (has(p.forces, Force::SpinDrift) && !p.spinAero) {
        return Status::MissingData;
    }
    return Status::Ok;
}

} // namespace Ballistics::PM
