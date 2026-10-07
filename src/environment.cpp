#include "ballistics/pm/environment.h"

namespace Ballistics::PM {

Environment Environment::standard() {
    Environment e;
    e.wind = WindProfile::calm();
    return e;
}

} // namespace Ballistics::PM
