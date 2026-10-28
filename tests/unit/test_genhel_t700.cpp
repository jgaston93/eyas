// M9 Stage B: GenHel's T700 engines, governor and clutch (D-053). The
// component against its steady maps (closed form), against GenHel's
// equations as printed (tests/support/genhel_t700_reference.hpp) where the two
// must agree, on a free shaft against isochronous governing, the clutch's
// closed forms and conservation of angular momentum, and the config and
// real-time rules.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/genhel_t700_reference.hpp"
#include "support/m9_engine.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using test::engine::Harness;
using test::engine::kFtLbf;
using test::engine::kGovernor;
using test::engine::kLbmHr;
using test::engine::make_config;
namespace t700 = test::t700;
using Engine = GenHelT700;

// RK4 on a state vector with a rate function f(t, x) → ẋ.
template <class F>
void rk4(std::vector<double>& x, double t, double h, F&& f) {
    const std::size_t n = x.size();
    std::vector<double> xs(n);
    const std::vector<double> k1 = f(t, x);
    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x[i] + 0.5 * h * k1[i];
    }
    const std::vector<double> k2 = f(t + 0.5 * h, xs);
    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x[i] + 0.5 * h * k2[i];
    }
    const std::vector<double> k3 = f(t + 0.5 * h, xs);
    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x[i] + h * k3[i];
    }
    const std::vector<double> k4 = f(t + h, xs);
    for (std::size_t i = 0; i < n; ++i) {
        x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

// The collective [0, 1] whose spindle speed is n_lds [fraction], on the
// tables' linear stretches 20-30 % (0.85 deg/%) and 0-20 deg (0.32 %/deg).
double collective_for(double n_lds) {
    const double lds_deg = (n_lds * 100.0 - 78.6) / ((85.0 - 78.6) / 20.0);
    return lds_deg / 0.85 / 100.0;
}

// On the steady maps (D-053 item 1) every rate is zero: N_GG is Table
// 6.5.1's for the delivered torque, the fuel Table 6.5.4's, P3 Table 6.5.5's,
// the speed error zero and the ECU holding N_GG,LDS − N_GG; the engines
// deliver the torque asked for.
TEST(GenHelT700, SteadyStatesLieOnTheMaps) {
    const t700::Data d;
    const Harness h(make_config(d));
    struct Case {
        double q_per_engine_ftlbf;
        double collective;
    };
    for (const Case c : {Case{3000.0, 0.3}, Case{15000.0, 0.5}, Case{27000.0, 0.8}}) {
        const double q_e = 2.0 * c.q_per_engine_ftlbf * kFtLbf;
        const double demand = q_e / 1.1;
        std::vector<double> x(Engine::kNumStates);
        h.engine().equilibrium(q_e, demand, c.collective, x);
        std::vector<double> xd;
        h.evaluate(x, kGovernor, c.collective, demand, xd);
        const double n_expected = t700::interp(d.q_trim, d.ngg_trim, c.q_per_engine_ftlbf) / 100.0;
        EXPECT_NEAR(x[Engine::kNgg], n_expected, 1e-12);
        EXPECT_NEAR(x[Engine::kFuel], t700::interp(d.fuel_ngg, d.fuel, 100.0 * n_expected) * kLbmHr,
                    1e-15);
        const GenHelT700Signals s = h.engine().signals(x, kGovernor, c.collective, demand);
        EXPECT_NEAR(s.q_engine, q_e, 1e-9 * q_e);
        EXPECT_NEAR(s.n_gg_ref, s.n_gg, 1e-12);
        EXPECT_TRUE(s.engaged);
        // High gain only above Q_INT's switch point, K_4 Q_PT = 20 V (D-053).
        EXPECT_EQ(s.high_gain, d.K4 * c.q_per_engine_ftlbf / d.GE > 20.0);
        for (std::size_t i = 0; i < xd.size(); ++i) {
            EXPECT_NEAR(xd[i], 0.0, 1e-9) << "state " << i << ", " << c.q_per_engine_ftlbf;
        }
    }
}

// The ECU (Fig. 6.1.3) is linear in GenHel's increments, so absolute and
// incremental forms are the same networks: driven by the same rotor speed,
// the component's ECU states minus their start equal the reference's, through
// low gain, high gain (a 3 % droop, outside the −4/+1 % band) and V_SS on its
// limits. Started with the spindle at the trim speed (ECU = 0, V_SS on A_U = 0),
// where GenHel's incremental V_SS starts too. GenHel sets V_SS back to its
// limit each frame ("IF V_SS > A_U SET V_SS = A_U", p. 5.6-13), the reference
// after each step; the component returns it within 0.05 s (D-053 amendment
// 2). Both are a clamped integrator to within one step's overshoot past the
// limit, K_9 |V_S| h, so they agree to that, and the difference shrinks with
// the step (measured 2.1e-6 at h = 1e-4 s and 1.2e-6 at h/2, against a
// bound of 2.9e-4).
TEST(GenHelT700, EcuMatchesGenHelAsPrinted) {
    const t700::Data d;
    const Harness h(make_config(d));
    const double q_e = 2.0 * 6000.0 * kFtLbf;  // N_GG 84.6 %
    const double demand = q_e / 1.1;
    const double collective = collective_for(0.846);
    const auto omega = [](double t) {
        if (t < 0.5) {
            return kGovernor * (1.0 + 0.004 * std::sin(9.0 * t));
        }
        if (t < 1.5) {
            return 0.97 * kGovernor;  // high gain
        }
        return 1.02 * kGovernor;  // low gain, V_SS driven down
    };
    struct Run {
        double worst = 0.0;
        double scale = 0.0;
        double overshoot = 0.0;  // K_9 max|V_S| h [V]
        bool saw_high = false;
        bool saw_low = false;
        double vss = 0.0;
    };
    const auto run = [&](double step) {
        std::vector<double> x(Engine::kNumStates);
        h.engine().equilibrium(q_e, demand, collective, x);
        EXPECT_NEAR(x[Engine::kVss], 0.0, 1e-9);
        t700::Reference ref;
        ref.d = d;
        const std::array<double, t700::kNumStates> r0 =
            ref.anchor(demand / kFtLbf, 100.0 * collective, kGovernor);
        std::vector<double> xr(r0.begin(), r0.end());
        const std::vector<double> x0 = x;
        Run out;
        const auto steps = static_cast<int>(std::lround(3.0 / step));
        for (int i = 0; i < steps; ++i) {
            const double t = i * step;
            rk4(x, t, step, [&](double ts, const std::vector<double>& xs) {
                std::vector<double> xd;
                h.evaluate(xs, omega(ts), collective, demand, xd);
                return xd;
            });
            rk4(xr, t, step, [&](double ts, const std::vector<double>& xs) {
                const auto r = ref.rates(xs, omega(ts), 100.0 * collective);
                return std::vector<double>(r.begin(), r.end());
            });
            xr[t700::kVss] = std::clamp(xr[t700::kVss], d.AL, d.AU);  // GenHel's SET
            const t700::Signals s = ref.signals(xr, omega(t + step), 100.0 * collective);
            (s.open ? out.saw_high : out.saw_low) = true;
            const double vs = xr[t700::kVsZ] + (d.t1 / d.t2) * (s.vg - xr[t700::kVsZ]);
            out.overshoot = std::max(out.overshoot, d.K9 * std::abs(vs) * step);
            const std::array<std::pair<std::size_t, std::size_t>, 7> pairs = {
                {{Engine::kIdb, t700::kIdb},
                 {Engine::kIdbRate, t700::kIdbRate},
                 {Engine::kVs, t700::kVsZ},
                 {Engine::kVss, t700::kVss},
                 {Engine::kVdmd, t700::kVdmdZ},
                 {Engine::kMotor, t700::kMZ},
                 {Engine::kIns, t700::kIns}}};
            for (const auto& [ic, ir] : pairs) {
                // INS in inches against metres; the reference applies G_4
                // ahead of the torque motor's lead-lag (mA), the component
                // after it (V); V_SS as the model reads it, clamped.
                const double unit = ic == Engine::kIns ? units::kInch : 1.0;
                const double vc =
                    (ic == Engine::kVss ? std::clamp(x[ic], d.AL, d.AU) : x[ic]) - x0[ic];
                const double vr = xr[ir] * unit / (ic == Engine::kMotor ? d.G4 : 1.0);
                out.worst = std::max(out.worst, std::abs(vc - vr) / unit);
                out.scale = std::max(out.scale, std::abs(vr) / unit);
            }
        }
        out.vss = x[Engine::kVss];
        return out;
    };
    const Run a = run(1e-4);
    const Run b = run(5e-5);
    EXPECT_TRUE(a.saw_high);
    EXPECT_TRUE(a.saw_low);
    EXPECT_GT(a.scale, 0.1);
    EXPECT_LT(a.worst, 1e-9 * a.scale + a.overshoot);
    EXPECT_LT(b.worst, 0.75 * a.worst);  // shrinks with h (O(h): 0.5)
    EXPECT_LT(a.vss, -0.1);              // driven down by the overspeed
}

// GenHel's data with every partial equal to its steady map's slope, on
// affine maps (exact on GenHel's broken-line tables): P3_ss = 69 + 5.2 (N − 70)
// psi, W_ss = 140 + 15 (N − 70) lbm/hr, 1100 ft·lbf per % of N_GG at the rotor
// per engine. The fuel feedback gain ΔW_f/ΔP3 is the steady ratio W_ss/P3_ss at
// each row, which varies with N_GG, so where the ratio is scheduled matters.
t700::Data consistent_data() {
    t700::Data d;
    const auto p3 = [](double n) { return 69.0 + 5.2 * (n - 70.0); };
    const auto wss = [](double n) { return 140.0 + 15.0 * (n - 70.0); };
    for (std::size_t i = 0; i < d.q_trim.size(); ++i) {
        d.ngg_trim[i] = 70.0 + d.q_trim[i] / 1100.0;
    }
    for (std::size_t i = 0; i < d.fuel_ngg.size(); ++i) {
        d.fuel[i] = wss(d.fuel_ngg[i]);
    }
    for (std::size_t i = 0; i < d.ngg.size(); ++i) {
        const double n = d.ngg[i];
        d.p3_ss[i] = p3(n);
        d.dp3_dwf[i] = 0.08;
        d.dp3_dngg[i] = 5.2 - 0.08 * 15.0;
        d.dwf_dp3[i] = wss(n) / p3(n);
        d.dqgg_dwf[i] = 0.27;
        d.dqgg_dngg[i] = -0.27 * 15.0;
        d.dqpt_dwf[i] = 0.36;
        d.dqpt_dnpt[i] = 0.01;
        d.dqpt_dngg[i] = 1100.0 / d.GE - 0.36 * 15.0;
    }
    return d;
}

// Re-anchored on consistent maps, the gas generator and fuel control are
// GenHel's to first order (D-053 item 1 and its amendment): started off the
// steady line by δ with the demanded speed held (rotor at its governed speed,
// so the ECU holds still), the two relax alike, and their difference is
// second order: halving δ quarters it.
TEST(GenHelT700, GasGeneratorMatchesGenHelToSecondOrder) {
    const t700::Data d = consistent_data();
    const Harness h(make_config(d));
    const double n0 = 0.90;  // a row of Table 6.5.6, where ΔW_f/ΔP3 is the steady ratio
    const double q_e = 2.0 * 1100.0 * 20.0 * kFtLbf;
    const double demand = q_e / 1.1;
    const double collective = 0.5;  // spindle 92.1 %: the ECU holds −2.1 %
    const auto run = [&](double delta) {
        std::vector<double> x(Engine::kNumStates);
        h.engine().equilibrium(q_e, demand, collective, x);
        EXPECT_NEAR(x[Engine::kNgg], n0, 1e-12);
        t700::Reference ref;
        ref.d = d;
        const auto a = ref.anchor(demand / kFtLbf, 100.0 * collective, kGovernor);
        std::vector<double> xr(a.begin(), a.end());
        // Off the steady line: N_GG up δ %, fuel up 10 δ lbm/hr, measured
        // speed and P3 feedback at rest there.
        x[Engine::kNgg] += 0.01 * delta;
        x[Engine::kNggMeas] = x[Engine::kNgg];
        x[Engine::kFuel] += 10.0 * delta * kLbmHr;
        xr[t700::kDngg] = delta;
        xr[t700::kDnggMeas] = delta;
        xr[t700::kDwf] = 10.0 * delta;
        const GenHelT700Signals s = h.engine().signals(x, kGovernor, collective, demand);
        x[Engine::kP3Lag] = s.p3 / isa::kSeaLevelPressure;
        const double dp3 = 0.08 * xr[t700::kDwf] + d.dp3_dngg[0] * delta;
        xr[t700::kP3Fb] = t700::interp(d.ngg, d.dwf_dp3, 100.0 * n0 + delta) * dp3;
        double worst = 0.0;
        double moved = 0.0;
        const double step = 1e-4;
        const double n_eq = n0;
        for (int i = 0; i < 20000; ++i) {
            const double t = i * step;
            rk4(x, t, step, [&](double, const std::vector<double>& xs) {
                std::vector<double> xd;
                h.evaluate(xs, kGovernor, collective, demand, xd);
                return xd;
            });
            rk4(xr, t, step, [&](double, const std::vector<double>& xs) {
                const auto r = ref.rates(xs, kGovernor, 100.0 * collective);
                return std::vector<double>(r.begin(), r.end());
            });
            const double dn_c = 100.0 * (x[Engine::kNgg] - n_eq);
            worst = std::max(worst, std::abs(dn_c - xr[t700::kDngg]));
            moved = std::max(moved, std::abs(xr[t700::kDngg] - delta));
        }
        EXPECT_GT(moved, 0.1 * delta);  // it does relax
        return worst;
    };
    const double e1 = run(0.5);
    const double e2 = run(0.25);
    EXPECT_LT(e1, 0.02 * 0.5);
    EXPECT_GT(e1 / e2, 3.5);
    EXPECT_LT(e1 / e2, 4.5);
}

// An airframe heavy enough to stand still (so its own free motion, a
// gyrostat's slow precession, doesn't mix into the rotor speed), a free shaft
// with the drive's inertia J_d (a part at gear 1 about −z), the stand-in rotor
// demanding `before` then `after` [ft·lbf] from t_step, and the engines at
// their equilibrium for `before`.
struct Rig {
    std::unique_ptr<Executive> ex;
    std::size_t engine = 0;  // component index
};
const MassProperties kHeavy = make_mass_properties(7e9, 6e12, 4e13, 3.7e13, 0.0);

Rig make_rig(const t700::Data& d, double before, double after, double t_step, double collective,
             double j_drive_slugft2 = 7911.0) {
    ExecutiveConfig ec;
    ec.mass = kHeavy;
    ec.max_substeps = 1'000'000;
    ShaftConfig sh;
    sh.name = "main";
    sh.speed = kGovernor;
    sh.drive = ShaftDrive::free;
    sh.parts = {ShaftPart{j_drive_slugft2 * test::engine::kSlugFt2, 1.0, Vec3(0.0, 0.0, -1.0)}};
    ec.shafts = {sh};
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<test::engine::Load>(before * kFtLbf, after * kFtLbf, t_step));
    comps.push_back(std::make_unique<Engine>(make_config(d, j_drive_slugft2)));
    Rig r;
    r.ex = std::make_unique<Executive>(ec, std::move(comps));
    r.engine = 1;
    std::vector<double> x(r.ex->state().begin(), r.ex->state().end());
    const auto& eng = static_cast<const Engine&>(r.ex->component(r.engine));
    eng.equilibrium(
        (1.0 + d.KQ) * before * kFtLbf, before * kFtLbf, collective,
        std::span<double>(x).subspan(r.ex->component_offset(r.engine), Engine::kNumStates));
    EXPECT_TRUE(r.ex->restore(x, 0, 0));
    return r;
}

double shaft_speed(const Executive& ex) {
    return ex.state()[ex.shaft_offset(0) + 1];
}

std::span<const double> engine_states(const Rig& r) {
    return r.ex->state().subspan(r.ex->component_offset(r.engine), Engine::kNumStates);
}

// The governor is isochronous (D-053 item 2): after a 50 % step in the rotor's
// demand and, separately, a step in collective, the rotor speed droops (or
// overspeeds) and returns to the governed speed, the ECU's integrator taking
// up the change, and the gas generator settles on Table 6.5.1's speed for
// the new torque.
TEST(GenHelT700, GovernorIsIsochronous) {
    const t700::Data d;
    struct Case {
        double before, after, collective_before, collective_after;
    };
    for (const Case c : {Case{20000.0, 30000.0, 0.6, 0.6}, Case{20000.0, 20000.0, 0.5, 0.65}}) {
        Rig r = make_rig(d, c.before, c.after, 1.0, c.collective_before);
        Controls u;
        u.collective = c.collective_before;
        (void)r.ex->advance(u, {}, 1'000'000'000);
        u.collective = c.collective_after;
        double lo = kGovernor;
        double hi = kGovernor;
        for (int i = 0; i < 800; ++i) {  // 80 s
            ASSERT_FALSE(r.ex->advance(u, {}, 100'000'000).non_finite);
            lo = std::min(lo, shaft_speed(*r.ex));
            hi = std::max(hi, shaft_speed(*r.ex));
        }
        EXPECT_GT(hi - lo, 1e-3 * kGovernor) << c.after;  // it was disturbed
        EXPECT_NEAR(shaft_speed(*r.ex), kGovernor, 1e-7 * kGovernor) << c.after;
        const double n = engine_states(r)[Engine::kNgg];
        const double q = (1.0 + d.KQ) * c.after / d.NE;
        EXPECT_NEAR(n, t700::interp(d.q_trim, d.ngg_trim, q) / 100.0, 1e-5) << c.after;
    }
}

// The cockpit's engine-speed trim (D-055 item 1) moves the governor's
// reference: from a steady 20000 N·m, a reference of 1.02 (and then 0.97) of
// the governed speed is held, isochronously, once the ECU has taken it up.
TEST(GenHelT700, GovernsAtTheSpeedReference) {
    const t700::Data d;
    Rig r = make_rig(d, 20000.0, 20000.0, 1.0, 0.6);
    Controls u;
    u.collective = 0.6;
    (void)r.ex->advance(u, {}, 1'000'000'000);
    for (const double ref : {1.02, 0.97}) {
        u.rotor_speed_reference = ref;
        for (int i = 0; i < 800; ++i) {  // 80 s
            ASSERT_FALSE(r.ex->advance(u, {}, 100'000'000).non_finite);
        }
        EXPECT_NEAR(shaft_speed(*r.ex), ref * kGovernor, 1e-6 * kGovernor) << ref;
    }
}

// GenHel's engine has no fuel shutoff (D-056 item 1): the fuel off changes
// nothing, bit for bit, as the power lever doesn't (D-055 item 1).
TEST(GenHelT700, IgnoresTheFuelShutoff) {
    const t700::Data d;
    Rig on = make_rig(d, 20000.0, 20000.0, 1.0, 0.6);
    Rig off = make_rig(d, 20000.0, 20000.0, 1.0, 0.6);
    Controls u;
    u.collective = 0.6;
    Controls v = u;
    v.engines[0].fuel_on = false;
    (void)on.ex->advance(u, {}, 2'000'000'000);
    (void)off.ex->advance(v, {}, 2'000'000'000);
    const auto a = on.ex->state();
    const auto b = off.ex->state();
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i], b[i]) << i;
    }
}

// The clutch (D-053 item 3). When the rotor drives the shaft (its demand
// negative, as in autorotation) the lagged demand FLQREQ turns negative and
// the clutch lets go: then the drive alone carries the load,
// Ω̇ = −(1 + K_Q) Q_MR / J_d, and the engine side runs on its own,
// ṡ = Ω̇ − Q_E/J_TOTE (+ n·ω̇/G_E). When the demand returns, the drive slows
// until the engine side catches it, and the clutch takes up again.
TEST(GenHelT700, ClutchLetsGoAndTakesUp) {
    const t700::Data d;
    const double j_d = 7911.0;
    Rig r = make_rig(d, 15000.0, -4000.0, 0.5, 0.5, j_d);
    Controls u;
    u.collective = 0.5;
    (void)r.ex->advance(u, {}, 2'000'000'000);
    const auto& eng = static_cast<const Engine&>(r.ex->component(r.engine));
    ASSERT_GT(engine_states(r)[Engine::kSlip], 0.0);
    std::vector<double> xd(r.ex->num_states(), 0.0);
    ASSERT_TRUE(r.ex->derivatives(r.ex->sim_time(), r.ex->state(), u, {}, xd));
    const double omega_dot = xd[r.ex->shaft_offset(0) + 1];
    // The airframe stands still (kHeavy), so the drive alone: Ω̇ = Q/J_d to
    // J_d/I ≈ 1e-9.
    const double expect = -(1.0 + d.KQ) * (-4000.0) * kFtLbf / (j_d * test::engine::kSlugFt2);
    EXPECT_NEAR(omega_dot, expect, 1e-7 * std::abs(expect));
    const GenHelT700Signals s =
        eng.signals(engine_states(r), shaft_speed(*r.ex), u.collective, -4000.0 * kFtLbf);
    EXPECT_FALSE(s.engaged);
    const Vec3 w_dot(xd[body_layout::kOmega], xd[body_layout::kOmega + 1],
                     xd[body_layout::kOmega + 2]);
    const double slip_rate = xd[r.ex->component_offset(r.engine) + Engine::kSlip];
    EXPECT_NEAR(slip_rate,
                omega_dot - s.q_engine / (d.JTOTE * test::engine::kSlugFt2) +
                    Vec3(1.0, 0.0, 0.0).dot(w_dot) / d.GE,
                1e-9 * std::abs(omega_dot));
    // Back to a positive demand at 2 s: the clutch takes up again.
    Rig r2 = make_rig(d, 15000.0, -4000.0, 0.5, 0.5, j_d);
    (void)r2.ex->advance(u, {}, 1'500'000'000);
    ASSERT_GT(engine_states(r2)[Engine::kSlip], 0.0);
    std::vector<double> x(r2.ex->state().begin(), r2.ex->state().end());
    const double t_now = r2.ex->sim_time();
    // Rebuild with the demand back on from now (the Load's step time is fixed):
    Rig r3 = make_rig(d, -4000.0, 15000.0, t_now, 0.5, j_d);
    ASSERT_TRUE(r3.ex->restore(x, r2.ex->n_steps(), 0));
    bool engaged = false;
    for (int i = 0; i < 200 && !engaged; ++i) {  // up to 20 s
        (void)r3.ex->advance(u, {}, 100'000'000);
        engaged =
            eng.signals(engine_states(r3), shaft_speed(*r3.ex), u.collective, 15000.0 * kFtLbf)
                .engaged;
    }
    EXPECT_TRUE(engaged);
    EXPECT_LE(engine_states(r3)[Engine::kSlip], 1e-9);
}

// Angular momentum (D-052, D-053 item 5): the engines' torques are internal,
// so with the airframe tumbling in vacuum, the drive's part, the engine side
// and the gas-generator spools exchanging momentum with it, the total about
// the CG — airframe I ω plus each spinning part's relative momentum, (J/g) Ω n
// for the drive, (J_TOTE/G_E) Ω_e n_pt, J_GG N_GG100 N_GG n_gg — is conserved
// to RK4's truncation (h⁴), clutch engaged and free.
TEST(GenHelT700, ConservesAngularMomentum) {
    const t700::Data d;
    for (const bool engaged : {true, false}) {
        const auto run = [&](std::int64_t step_ns) {
            ExecutiveConfig ec;
            ec.mass = make_mass_properties(7000.0, 6000.0, 40000.0, 37000.0, 1900.0);
            ec.step_ns = step_ns;
            ec.max_substeps = 1'000'000;
            ShaftConfig sh;
            sh.name = "main";
            sh.speed = kGovernor;
            sh.drive = ShaftDrive::free;
            const Vec3 n_d = Vec3(0.1, 0.05, -1.0).normalized();
            sh.parts = {ShaftPart{900.0, 1.0, n_d}, ShaftPart{30.0, 22.5, Vec3(1.0, 0.0, 0.0)}};
            ec.shafts = {sh};
            GenHelT700Config c = make_config(d, 800.0);
            c.pt_axis_b = Vec3(1.0, 0.1, 0.0).normalized();
            c.gg_axis_b = Vec3(1.0, -0.2, 0.05).normalized();
            c.gg_inertia = 2.0;  // exaggerated, so its terms count
            // No governing: the ECU's switch and integrator limits would make
            // the rates non-smooth, which RK4's h⁴ check can't see past; the
            // mechanical terms under test don't depend on them.
            c.k1 = 0.0;
            c.k3 = 0.0;
            std::vector<std::unique_ptr<DynamicComponent>> comps;
            comps.push_back(std::make_unique<test::engine::Load>(0.0, 0.0, 0.0));
            comps.push_back(std::make_unique<Engine>(c));
            Executive ex(ec, std::move(comps));
            BodyState b;
            b.omega_b = Vec3(0.3, -0.2, 0.4);
            b.q_nb = Quat(Eigen::AngleAxisd(0.3, Vec3(1, 2, 3).normalized()));
            ex.set_body_state(b);
            std::vector<double> x(ex.state().begin(), ex.state().end());
            const std::size_t o = ex.component_offset(1);
            const auto& eng = static_cast<const Engine&>(ex.component(1));
            // Off equilibrium: torque from the engines and a spooling gas generator.
            eng.equilibrium(2.0 * 9000.0 * kFtLbf, 0.0, 0.4,
                            std::span<double>(x).subspan(o, Engine::kNumStates));
            // Engaged, fuel up so the engines drive the shaft; free, fuel
            // down so the engine side slows and the clutch stays free.
            x[o + Engine::kFuel] *= engaged ? 1.3 : 0.7;
            x[o + Engine::kDemand] = 10.0;  // [rad/s²] engaged unless slipping
            x[o + Engine::kSlip] = engaged ? 0.0 : 20.0;
            EXPECT_TRUE(ex.restore(x, 0, 0));
            const auto momentum = [&](const Executive& e) {
                const std::span<const double> xs = e.state();
                const BodyState body = e.body_state();
                Vec3 h = ec.mass.inertia_b * body.omega_b;
                const double omega = xs[e.shaft_offset(0) + 1];
                for (const ShaftPart& p : sh.parts) {
                    h += p.inertia / p.gear * omega * p.axis_b;
                }
                const double slip = std::max(xs[o + Engine::kSlip], 0.0);
                h += c.engine_inertia / c.gear * (omega - slip) * c.pt_axis_b;
                h += c.gg_inertia * c.gas_generator_100 * xs[o + Engine::kNgg] * c.gg_axis_b;
                return Vec3(body.q_nb * h);
            };
            const Vec3 h0 = momentum(ex);
            const double s0 = ex.state()[o + Engine::kSlip];
            const double n0 = ex.state()[o + Engine::kNgg];
            (void)ex.advance({}, {}, 500'000'000);
            const Vec3 h1 = momentum(ex);
            EXPECT_EQ(ex.state()[o + Engine::kSlip] > 0.0, !engaged);  // stayed in its mode
            EXPECT_EQ(eng.signals(ex.state().subspan(o, Engine::kNumStates),
                                  ex.state()[ex.shaft_offset(0) + 1], 0.0, 0.0)
                          .engaged,
                      engaged);
            EXPECT_GT(std::abs(ex.state()[o + Engine::kNgg] - n0), 1e-4);
            (void)s0;
            return (h1 - h0).norm() / h0.norm();
        };
        const double e1 = run(1'000'000);
        const double e2 = run(500'000);
        EXPECT_LT(e1, 1e-9) << engaged;
        if (e2 > 1e-14) {
            EXPECT_GT(e1 / e2, 10.0) << engaged;
        }
    }
}

// The engine's shaft terms are conservative (D-052): engaged, the shaft
// torque's sensitivity to the body acceleration (K_a) is the airframe load's
// to the shaft acceleration (H_s), and K_s is −J_TOTE; free, the ω̇ block of H
// is symmetric.
TEST(GenHelT700, ShaftTermsAreSymmetric) {
    const t700::Data d;
    GenHelT700Config c = make_config(d);
    c.pt_axis_b = Vec3(1.0, 0.2, -0.1).normalized();
    const Harness h(c);
    std::vector<double> x(Engine::kNumStates);
    h.engine().equilibrium(2.0 * 12000.0 * kFtLbf, 2.0 * 12000.0 * kFtLbf / 1.1, 0.5, x);
    std::vector<double> xd;
    ComponentDerivs dd{{}, {}, Wrench{}, Mat66::Zero()};
    h.evaluate(x, kGovernor, 0.5, 2.0 * 12000.0 * kFtLbf / 1.1, xd, &dd);
    for (int j = 0; j < 6; ++j) {
        EXPECT_DOUBLE_EQ(dd.Ka(0, j), dd.Hs(j, 0)) << j;
    }
    EXPECT_DOUBLE_EQ(dd.Ks(0, 0), -c.engine_inertia);
    x[Engine::kSlip] = 3.0;
    h.evaluate(x, kGovernor, 0.5, 0.0, xd, &dd);
    EXPECT_EQ(dd.Ks(0, 0), 0.0);
    const Mat3 Hw = dd.H.block<3, 3>(3, 3);
    EXPECT_LT((Hw - Hw.transpose()).norm(), 1e-12 * Hw.norm());
}

// The readouts (D-060): GenHel runs its N_E engines as one, so each of its
// entries, from first_engine on, reads the same; the others stay unset.
TEST(GenHelT700, ReportsEachOfItsEnginesAlike) {
    const t700::Data d;
    GenHelT700Config c = make_config(d);
    c.first_engine = 1;
    const Harness h(c);
    std::vector<double> x(Engine::kNumStates);
    h.engine().equilibrium(2.0 * 12000.0 * kFtLbf, 2.0 * 12000.0 * kFtLbf / 1.1, 0.5, x);
    std::vector<double> xd;
    ComponentDerivs dd{{}, {}, Wrench{}, Mat66::Zero()};
    Readouts r;
    dd.readouts = &r;
    h.evaluate(x, kGovernor, 0.5, 2.0 * 12000.0 * kFtLbf / 1.1, xd, &dd);
    ASSERT_EQ(c.engines, 2);
    EXPECT_TRUE(std::isnan(r.engines[0].torque));
    EXPECT_TRUE(std::isnan(r.engines[3].torque));
    for (const std::size_t e : {std::size_t{1}, std::size_t{2}}) {
        const EngineState& s = r.engines[e];
        EXPECT_EQ(s.gas_generator_speed_percent, 100.0 * x[Engine::kNgg]) << e;
        EXPECT_NEAR(s.torque, 12000.0 * kFtLbf / (d.GE), 1e-6 * s.torque) << e;  // per engine
        EXPECT_NEAR(s.power, s.torque * s.power_turbine_speed_percent / 100.0 * c.power_turbine_100,
                    1e-9 * s.power)
            << e;
        EXPECT_TRUE(std::isnan(s.turbine_gas_temperature)) << e;
        EXPECT_FALSE(s.out) << e;
    }
    c.first_engine = 3;  // its second engine past Controls::engines
    EXPECT_THROW((void)GenHelT700(c), InitError);
}

// The high/low-gain switch's torque integrator (D-053 item 2): Q_INT rises
// above K_4 Q_PT = 20 V, 339 ft·lbf of power-turbine torque per engine, and
// falls below, held at its limits 0 and 40.
TEST(GenHelT700, SwitchIntegratorTurnsAt339FootPounds) {
    const t700::Data d;
    const Harness h(make_config(d));
    const double q_switch = 20.0 / d.K4;  // [ft·lbf], per engine at the power turbine
    EXPECT_NEAR(q_switch, 339.0, 0.5);
    for (const double f : {0.98, 1.02}) {
        const double q_e = d.NE * d.GE * q_switch * f * kFtLbf;
        std::vector<double> x(Engine::kNumStates);
        h.engine().equilibrium(q_e, q_e / 1.1, 0.9, x);
        x[Engine::kQint] = 20.0;
        std::vector<double> xd;
        h.evaluate(x, kGovernor, 0.9, q_e / 1.1, xd);
        EXPECT_EQ(xd[Engine::kQint] > 0.0, f > 1.0) << f;
        x[Engine::kQint] = f > 1.0 ? 40.0 : 0.0;  // at the limit it drives to
        h.evaluate(x, kGovernor, 0.9, q_e / 1.1, xd);
        EXPECT_EQ(xd[Engine::kQint], 0.0) << f;
    }
}

// D-053 amendment 2: the rotor's demand is affine in the body acceleration
// a (gravity is in its value at a = 0), so the losses on the shaft and the
// clutch's lagged demand carry its a terms: K_Q and (1 + K_Q)/(J_d τ) times
// them. Without them uniform gravity would torque the shaft.
TEST(GenHelT700, LossesAndDemandFollowTheRotorsAccelerationTerms) {
    const t700::Data d;
    Harness h(make_config(d));
    const std::array<double, 6> da = {11.0, -7.0, 3.0, 230.0, -170.0, 90.0};
    h.set_demand_a(da);
    const double q = 60000.0;
    std::vector<double> x(Engine::kNumStates);
    h.engine().equilibrium(1.1 * q, q, 0.6, x);
    std::vector<double> xd;
    ComponentDerivs out{{}, {}, Wrench{}, Mat66::Zero()};
    h.evaluate(x, kGovernor, 0.6, q, xd, &out);
    const GenHelT700Config& c = h.engine().config();
    for (std::size_t j = 0; j < 6; ++j) {
        // Engaged: the engine's own Ka is the power turbines' momentum, in ω̇ only.
        const double own =
            j < 3 ? 0.0
                  : -(c.engine_inertia / c.gear) * c.pt_axis_b(static_cast<Eigen::Index>(j) - 3);
        EXPECT_NEAR(out.Ka(0, static_cast<Eigen::Index>(j)), own - d.KQ * da[j],
                    1e-12 * (1.0 + std::abs(da[j]) + std::abs(own)));
        EXPECT_NEAR(h.G()[Engine::kDemand * 6 + j],
                    (1.0 + d.KQ) * da[j] / (c.drive_inertia * c.demand_lag), 1e-15);
    }
}

// Code review of #69: an integration step that carries the slip below zero
// as the clutch re-engages leaves it there unless something returns it;
// engaged, it returns to zero within 0.05 s, as the limiters do, so a later
// disengagement isn't delayed by a negative slip to integrate away.
TEST(GenHelT700, ClutchSlipOvershootReturnsToZero) {
    const t700::Data d;
    const Harness h(make_config(d));
    const double q = 40000.0 * kFtLbf;
    std::vector<double> x(Engine::kNumStates);
    h.engine().equilibrium(q, q / 1.1, 0.6, x);
    x[Engine::kSlip] = -0.01;
    std::vector<double> xd;
    h.evaluate(x, kGovernor, 0.6, q / 1.1, xd);
    EXPECT_TRUE(h.engine().signals(x, kGovernor, 0.6, q / 1.1).engaged);
    EXPECT_DOUBLE_EQ(xd[Engine::kSlip], 0.01 / 0.05);
    x[Engine::kSlip] = 0.0;
    h.evaluate(x, kGovernor, 0.6, q / 1.1, xd);
    EXPECT_EQ(xd[Engine::kSlip], 0.0);
}

// D-053 amendment 2: the states sitting on a switch or a limit, whose rates
// aren't differentiable there, are reported pinned so that linearization
// holds them: the slip of an engaged clutch, and a limited integrator (Q_INT,
// V_SS) at a limit its input pushes against; not one inside its range, nor
// one at a limit its input pulls it off.
TEST(GenHelT700, ReportsTheStatesPinnedAtALimit) {
    const t700::Data d;
    const Harness h(make_config(d));
    const GenHelT700Config& c = h.engine().config();
    const double q_low = d.NE * d.GE * 300.0 * kFtLbf;  // below the switch: Q_INT's input < 0
    const auto pinned = [&](std::vector<double> x, double omega) {
        std::vector<unsigned char> p(Engine::kNumStates, 7);
        h.engine().pinned_states(h.context(omega, 0.6, q_low / 1.1), x, p);
        return p;
    };
    std::vector<double> x(Engine::kNumStates);
    h.engine().equilibrium(q_low, q_low / 1.1, 0.6, x);
    ASSERT_EQ(x[Engine::kQint], 0.0);
    x[Engine::kVss] = 0.5 * (c.lower + c.upper);
    std::vector<unsigned char> p = pinned(x, kGovernor);
    for (std::size_t i = 0; i < Engine::kNumStates; ++i) {
        const bool expect = i == Engine::kSlip || i == Engine::kQint;
        EXPECT_EQ(p[i], expect ? 1 : 0) << i;
    }
    x[Engine::kQint] = 13.0;  // inside its range
    EXPECT_EQ(pinned(x, kGovernor)[Engine::kQint], 0);
    x[Engine::kQint] = c.qint_limit;  // at the top, pulled down
    EXPECT_EQ(pinned(x, kGovernor)[Engine::kQint], 0);
    // V_SS at A_U with the rotor slow (low gain: V_S > 0 pushes up): pinned;
    // with it fast (V_S < 0): not.
    x[Engine::kQint] = 0.0;
    x[Engine::kVss] = c.upper;
    EXPECT_EQ(pinned(x, 0.99 * kGovernor)[Engine::kVss], 1);
    EXPECT_EQ(pinned(x, 1.01 * kGovernor)[Engine::kVss], 0);
    // Disengaged, the slip moves: not pinned.
    x[Engine::kSlip] = 0.5;
    EXPECT_EQ(pinned(x, kGovernor)[Engine::kSlip], 0);
}

// Trim (D-053 amendment): Q_INT, a limited integrator, has a steady value only
// at the limit its input drives it to, so the trim holds it there, set from
// the engine's torque at the state packed, whatever the value it came with.
TEST(GenHelT700, TrimHoldsTheSwitchIntegratorAtItsLimit) {
    const t700::Data d;
    const Harness h(make_config(d));
    std::vector<SteadyKind> kinds(Engine::kNumStates);
    h.engine().steady_kinds(kinds);
    EXPECT_EQ(kinds[Engine::kQint], SteadyKind::held);
    EXPECT_EQ(kinds[Engine::kSlip], SteadyKind::held);
    const double q_switch = 20.0 / d.K4;
    for (const double f : {0.999, 1.001}) {
        const double q_e = d.NE * d.GE * q_switch * f * kFtLbf;
        for (const double start : {0.0, 13.0, 40.0}) {
            std::vector<double> x(Engine::kNumStates);
            h.engine().equilibrium(q_e, q_e / 1.1, 0.9, x);
            x[Engine::kQint] = start;
            h.engine().steady_states(h.context(kGovernor, 0.9, q_e / 1.1), x);
            EXPECT_EQ(x[Engine::kQint], f > 1.0 ? 40.0 : 0.0) << f << " " << start;
        }
    }
}

// Real-time (spec §7): advance() with the engines on a free shaft allocates
// nothing.
TEST(GenHelT700, AllocatesNothing) {
    const t700::Data d;
    Rig r = make_rig(d, 20000.0, 30000.0, 0.05, 0.6);
    Controls u;
    u.collective = 0.6;
    (void)r.ex->advance(u, {}, 16'666'667);
    test::AllocationCounter count;
    for (int i = 0; i < 20; ++i) {
        (void)r.ex->advance(u, {}, 16'666'667);
    }
    EXPECT_EQ(count.count(), 0U);
}

}  // namespace
}  // namespace eyas
