#ifndef BALLISTICS_PM_DRAG_H
#define BALLISTICS_PM_DRAG_H

#include <vector>

namespace Ballistics::PM {

// Cd of the reference projectile at rest. A round's form factor is its own Cd over this.
inline constexpr double G1_REFERENCE_CD = 0.2629;
inline constexpr double G7_REFERENCE_CD = 0.1197;
inline constexpr double SHELL_REFERENCE_CD = 0.130;

// Cd as a function of Mach, scaled by a form factor that carries the projectile's
// departure from the reference shape. Cubic through the interior, linear at the ends.
class DragTable {
public:
    DragTable() = default;

    static DragTable g1(double formFactor);
    static DragTable g7(double formFactor);

    // Artillery shell: a sharp transonic rise peaking just above Mach 1 at roughly 2.6x the
    // subsonic value, then a slow decline. G1 peaks far too late and G7 is nearly flat
    // through the transonic region, so neither shape suits a shell.
    static DragTable shell(double formFactor);
    static DragTable custom(std::vector<double> mach, std::vector<double> cd,
                            double formFactor = 1.0);

    static DragTable g1ForCd(double cd) { return g1(cd / G1_REFERENCE_CD); }
    static DragTable g7ForCd(double cd) { return g7(cd / G7_REFERENCE_CD); }
    static DragTable shellForCd(double cd) { return shell(cd / SHELL_REFERENCE_CD); }

    double at(double mach) const;
    bool empty() const { return m_mach.size() < 2; }
    double formFactor() const { return m_formFactor; }

private:
    std::vector<double> m_mach, m_cd;
    double m_formFactor = 1.0;
};

} // namespace Ballistics::PM

#endif
