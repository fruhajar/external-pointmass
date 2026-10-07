#include "ballistics/pm/drag.h"

#include <algorithm>
#include <array>
#include <utility>

namespace Ballistics::PM {

namespace {

constexpr std::array<double, 55> G1_MACH = {
    0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50,
    0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.875, 0.90, 0.925, 0.95,
    0.975, 1.00, 1.025, 1.05, 1.075, 1.10, 1.125, 1.15, 1.20, 1.25, 1.30,
    1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.70, 1.80, 1.90, 2.00, 2.10,
    2.20, 2.30, 2.40, 2.50, 2.60, 2.70, 2.80, 3.00, 3.50, 4.00, 5.00
};

constexpr std::array<double, 55> G1_CD = {
    0.2629, 0.2558, 0.2487, 0.2413, 0.2344, 0.2278, 0.2214, 0.2155, 0.2104,
    0.2061, 0.2032, 0.2020, 0.2034, 0.2165, 0.2230, 0.2313, 0.2417, 0.2546,
    0.2633, 0.2742, 0.2872, 0.3018, 0.3182, 0.3357, 0.3525, 0.3684, 0.3835,
    0.3977, 0.4110, 0.4236, 0.4473, 0.4677, 0.4841, 0.4974, 0.5074, 0.5155,
    0.5221, 0.5278, 0.5328, 0.5410, 0.5464, 0.5495, 0.5507, 0.5506, 0.5497,
    0.5481, 0.5462, 0.5442, 0.5420, 0.5397, 0.5373, 0.5325, 0.5238, 0.5120, 0.4890
};

constexpr std::array<double, 43> G7_MACH = {
    0.00, 0.50, 0.60, 0.70, 0.80, 0.85, 0.90, 0.925, 0.95, 0.975,
    1.00, 1.025, 1.05, 1.075, 1.10, 1.125, 1.15, 1.20, 1.25, 1.30,
    1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.70, 1.80, 1.90, 2.00,
    2.10, 2.20, 2.30, 2.40, 2.50, 2.60, 2.70, 2.80, 2.90, 3.00,
    3.50, 4.00, 5.00
};

constexpr std::array<double, 43> G7_CD = {
    0.1198, 0.1197, 0.1196, 0.1194, 0.1193, 0.1194, 0.1194, 0.1194, 0.1193,
    0.1193, 0.1194, 0.1193, 0.1194, 0.1194, 0.1194, 0.1193, 0.1194, 0.1193,
    0.1193, 0.1194, 0.1197, 0.1202, 0.1207, 0.1215, 0.1226, 0.1242, 0.1278,
    0.1338, 0.1422, 0.1533, 0.1672, 0.1815, 0.1950, 0.2067, 0.2173, 0.2273,
    0.2368, 0.2462, 0.2554, 0.2642, 0.3006, 0.3264, 0.3656
};

// Representative zero-yaw drag for a boat-tailed artillery shell, M107 family shape.
constexpr std::array<double, 27> SHELL_MACH = {
    0.00, 0.40, 0.60, 0.70, 0.80, 0.85, 0.90, 0.95, 0.975,
    1.00, 1.025, 1.05, 1.10, 1.15, 1.20, 1.30, 1.40, 1.50,
    1.60, 1.80, 2.00, 2.25, 2.50, 2.75, 3.00, 3.50, 4.00
};

constexpr std::array<double, 27> SHELL_CD = {
    0.130, 0.130, 0.132, 0.136, 0.142, 0.148, 0.158, 0.190, 0.225,
    0.268, 0.300, 0.318, 0.334, 0.338, 0.337, 0.330, 0.322, 0.313,
    0.305, 0.290, 0.276, 0.262, 0.250, 0.240, 0.232, 0.218, 0.208
};

double cubic(double y0, double y1, double y2, double y3, double t) {
    const double a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    const double a1 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    const double a2 = -0.5 * y0 + 0.5 * y2;
    return ((a0 * t + a1) * t + a2) * t + y1;
}

template <std::size_t N>
DragTable build(const std::array<double, N>& mach, const std::array<double, N>& cd,
                double formFactor) {
    return DragTable::custom(std::vector<double>(mach.begin(), mach.end()),
                             std::vector<double>(cd.begin(), cd.end()),
                             formFactor);
}

} // namespace

DragTable DragTable::g1(double formFactor) { return build(G1_MACH, G1_CD, formFactor); }
DragTable DragTable::g7(double formFactor) { return build(G7_MACH, G7_CD, formFactor); }
DragTable DragTable::shell(double formFactor) { return build(SHELL_MACH, SHELL_CD, formFactor); }

DragTable DragTable::custom(std::vector<double> mach, std::vector<double> cd,
                            double formFactor) {
    DragTable t;
    if (mach.size() == cd.size()) {
        t.m_mach = std::move(mach);
        t.m_cd = std::move(cd);
    }
    t.m_formFactor = formFactor;
    return t;
}

double DragTable::at(double mach) const {
    if (empty()) {
        return 0.0;
    }

    const std::size_t last = m_mach.size() - 1;
    if (mach <= m_mach.front()) {
        return m_formFactor * m_cd.front();
    }
    if (mach >= m_mach[last]) {
        return m_formFactor * m_cd[last];
    }

    const auto upper = std::lower_bound(m_mach.begin(), m_mach.end(), mach);
    const std::size_t i = static_cast<std::size_t>(upper - m_mach.begin()) - 1;
    const double t = (mach - m_mach[i]) / (m_mach[i + 1] - m_mach[i]);

    const double cd = (i > 0 && i + 2 <= last)
                          ? cubic(m_cd[i - 1], m_cd[i], m_cd[i + 1], m_cd[i + 2], t)
                          : m_cd[i] + t * (m_cd[i + 1] - m_cd[i]);

    return m_formFactor * cd;
}

} // namespace Ballistics::PM
