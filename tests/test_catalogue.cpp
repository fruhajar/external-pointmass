#include "ballistics/pm/ballistics.h"

#include "test_support.h"

using namespace Ballistics::PM;

int main() {
    const Catalogue builtins = Catalogue::withBuiltins();
    CHECK(builtins.size() == 9);
    CHECK(builtins.ids().size() == 9);

    // Unknown ids are refused rather than silently substituted.
    CHECK(builtins.find("no such round") == nullptr);

    for (const std::string& id : builtins.ids()) {
        const Projectile* p = builtins.find(id);
        CHECK_MSG(p != nullptr, id);
        CHECK_MSG(validate(*p) == Status::Ok, id + ": " + describe(validate(*p)));

        CHECK_MSG(p->geometry.mass > 0.0, id);
        CHECK_MSG(p->geometry.diameter > 0.0, id);
        CHECK_MSG(p->muzzle.speed > 0.0, id);
        CHECK_MSG(!p->drag.empty(), id);
        CHECK_MSG(!p->name.empty(), id);

        // The drift term needs its own data block, and only spin-stabilised rounds carry it.
        if (has(p->forces, Force::SpinDrift)) {
            CHECK_MSG(p->spinAero.has_value(), id);
            CHECK_MSG(p->spinAero->Ixx > 0.0, id);
            CHECK_MSG(p->spinAero->C_M_alpha > 0.0, id);
            CHECK_MSG(p->spinAero->C_L_alpha > 0.0, id);
        }
    }

    // Only the rifled 155mm rounds are spin-stabilised. Mortar bombs and the 120mm
    // smoothbore tank rounds are fin-stabilised, so no gyroscopic yaw of repose and no drift.
    for (const std::string& id : builtins.ids()) {
        const Projectile* p = builtins.find(id);
        const bool rifled = id.rfind("Howitzer155mm", 0) == 0;
        CHECK_MSG(has(p->forces, Force::SpinDrift) == rifled, id);
        CHECK_MSG((p->muzzle.spinRate > 0.0) == rifled, id);
    }

    // Spin comes from the 1:20 calibre twist, not a stated rpm: rpm = 60 V / (20 d).
    {
        const Projectile* p = builtins.find("Howitzer155mm_HE");
        CHECK_REL(p->muzzle.spinRate,
                  60.0 * p->muzzle.speed / (20.0 * p->geometry.diameter), 1e-4);

        // Axial inertia from the measured M107 ratio, not a uniform cylinder. The uniform
        // cylinder value 0.5 m r^2 understates it by about 20%.
        CHECK_REL(p->spinAero->Ixx,
                  0.1391 * p->geometry.mass * p->geometry.diameter * p->geometry.diameter, 1e-3);
        CHECK(p->spinAero->Ixx > 0.5 * p->geometry.mass *
                                     (p->geometry.diameter / 2.0) * (p->geometry.diameter / 2.0));
    }

    // Gyroscopic stability must exceed 1 at the muzzle or the round would tumble. This is the
    // constraint that pins C_M_alpha, and it is independent of any drift measurement.
    for (const std::string& id : builtins.ids()) {
        const Projectile* p = builtins.find(id);
        if (!p->spinAero) {
            continue;
        }
        const double d = p->geometry.diameter;
        const double area = p->geometry.refArea();
        const double spin = p->muzzle.spinRate * 2.0 * 3.14159265358979323846 / 60.0;
        const double iy = (p->geometry.mass / 12.0) *
                          (3.0 * (d / 2.0) * (d / 2.0) + p->geometry.length * p->geometry.length);
        const double sg = (p->spinAero->Ixx * p->spinAero->Ixx * spin * spin) /
                          (2.0 * 1.225 * iy * area * d * p->muzzle.speed * p->muzzle.speed *
                           p->spinAero->C_M_alpha);
        CHECK_MSG(sg > 1.0 && sg < 3.0, id + " Sg " + std::to_string(sg));
    }

    // Calibrated rounds must reproduce the published maximum range they were fitted to.
    // This locks the calibration rather than an intermediate drag coefficient.
    {
        struct Anchor { const char* id; double range; };
        const Anchor anchors[] = {
            {"Mortar60mm_HE", 3490.0},
            {"Mortar60mm_SMK", 3200.0},
            {"Mortar81mm_HE", 5650.0},
            {"Mortar81mm_SMK", 4900.0},
            {"Mortar81mm_ILL", 5050.0},
            {"Mortar120mm_HE", 7200.0},
            {"Mortar120mm_SMK", 7200.0},
            {"Howitzer155mm_HE", 22400.0},
            {"Howitzer155mm_SMK", 18000.0},
        };

        Environment env = Environment::standard();
        env.latitude = 45.0;

        for (const Anchor& a : anchors) {
            const Projectile* p = builtins.find(a.id);
            CHECK_MSG(p != nullptr, a.id);
            CHECK_MSG(p->quality == DataQuality::Calibrated, a.id);

            Projectile bare = *p;
            bare.forces = Force::Gravity | Force::Drag;
            const Result<RangeEnvelope> e = maxRange(
                fromGround(p->muzzle.speed, 0.0, 0.0, 0.0), bare, env, GroundReference{0.0});
            CHECK_MSG(static_cast<bool>(e), a.id);
            if (e) {
                CHECK_MSG(std::abs(e->range - a.range) < 0.005 * a.range,
                          std::string(a.id) + " " + test::values(e->range, a.range,
                                                                 0.005 * a.range));
            }
        }
    }

    // Every round in the catalogue is calibrated. A round that could not be anchored to a
    // published figure was left out rather than shipped with an estimate.
    for (const std::string& id : builtins.ids()) {
        CHECK_MSG(builtins.find(id)->quality == DataQuality::Calibrated, id);
    }

    // Callers can extend and shrink the set.
    Catalogue mine = builtins;
    Projectile custom;
    custom.id = "custom";
    custom.name = "test round";
    custom.geometry = {5.0, 0.08, 0.4};
    custom.muzzle = {300.0, 0.0, 0.0};
    custom.drag = DragTable::custom({0.0, 1.0, 3.0}, {0.25, 0.4, 0.3});
    mine.add(std::move(custom));

    CHECK(mine.size() == 10);
    CHECK(mine.find("custom") != nullptr);
    CHECK(builtins.size() == 9);

    CHECK(mine.remove("custom"));
    CHECK(!mine.remove("custom"));
    CHECK(mine.find("custom") == nullptr);

    return test::summary("catalogue");
}
