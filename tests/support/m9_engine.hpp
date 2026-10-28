#pragma once

#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "core/component.hpp"
#include "core/coupling.hpp"
#include "engine/genhel_t700.hpp"
#include "support/genhel_t700_reference.hpp"
#include "util/table.hpp"
#include "util/units.hpp"

// M9 Stage B fixtures (D-053): the genhel_t700 component built from the
// reference's GenHel data in SI, a stand-in rotor that loads a shaft and
// publishes its demand, and a single-component evaluation with a coupling
// store.
namespace eyas::test::engine {

inline constexpr double kFtLbf = units::kFoot * units::kPoundForce;  // [N·m]
inline constexpr double kLbmHr = units::kPoundMass / 3600.0;         // [kg/s]
inline constexpr double kSlugFt2 = units::kSlug * units::kFoot * units::kFoot;
inline constexpr double kGovernor = 27.0;  // the shaft's governed speed [rad/s]

template <std::size_t N>
std::shared_ptr<const Table1D> table(const std::array<double, N>& x, double sx,
                                     const std::array<double, N>& y, double sy) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (std::size_t i = 0; i < N; ++i) {
        xs.push_back(x[i] * sx);
        ys.push_back(y[i] * sy);
    }
    return std::make_shared<const Table1D>(Axis(std::move(xs)), std::move(ys));
}

// The component's config from GenHel's data in its units (p. 5.6-24, Tables
// 6.5.1 to 6.5.6), converted to SI; on shaft "main", reading "rotor".
inline GenHelT700Config make_config(const t700::Data& d, double drive_inertia_slugft2 = 7911.0) {
    using units::kDegree;
    using units::kInch;
    using units::kPsi;
    using units::kRpm;
    GenHelT700Config c;
    c.name = "engines";
    c.shaft = "main";
    c.rotor = "rotor";
    c.engines = static_cast<int>(d.NE);
    c.gear = d.GE;
    c.power_turbine_100 = d.NPT100 * kRpm;
    c.gas_generator_100 = 44700.0 * kRpm;
    c.loss_factor = d.KQ;
    c.torque_ngg = table(d.q_trim, kFtLbf, d.ngg_trim, 0.01);
    c.lds_rigging = table(d.xc, 0.01, d.lds, kDegree);
    c.lds_cam = table(d.cam_lds, kDegree, d.cam_ngg, 0.01);
    c.fuel = table(d.fuel_ngg, 0.01, d.fuel, kLbmHr);
    c.p3 = table(d.ngg, 0.01, d.p3_ss, kPsi);
    c.dp3_dwf = table(d.ngg, 0.01, d.dp3_dwf, kPsi / kLbmHr);
    c.dqgg_dwf = table(d.ngg, 0.01, d.dqgg_dwf, kFtLbf / kLbmHr);
    c.dqpt_dwf = table(d.ngg, 0.01, d.dqpt_dwf, kFtLbf / kLbmHr);
    c.dqpt_dnpt = table(d.ngg, 0.01, d.dqpt_dnpt, kFtLbf / kRpm);
    c.k1 = d.K1 * 100.0;
    c.k3 = d.K3 * 100.0;
    c.k4 = d.K4 / kFtLbf;
    c.k6 = d.K6;
    c.k9 = d.K9;
    c.k10 = d.K10;
    c.k11 = d.K11;
    c.dead_band_a = d.a;
    c.dead_band_b = d.b;
    c.t1 = d.t1;
    c.t2 = d.t2;
    c.t3 = d.t3;
    c.t4 = d.t4;
    c.upper = d.AU;
    c.lower = d.AL;
    c.g4 = d.G4 * 0.001;
    c.g5 = d.G5 * kInch / 0.001;
    c.g6 = d.G6 / kInch;
    c.g7 = d.G7 * 0.01 / kInch;
    c.T2 = d.T2;
    c.T3 = d.T3;
    c.speed_lag = d.TNGG;
    c.ratio_gain = d.DWP3_DNGG * kLbmHr / (kPsi * 0.01);
    c.p3_lag = d.TP3;
    c.valve_lag = d.TMV;
    c.acceleration = d.KCF_JGG * 0.01 / kFtLbf;
    c.heat_sink = d.KHS;
    c.drive_inertia = drive_inertia_slugft2 * kSlugFt2;
    c.engine_inertia = d.JTOTE * kSlugFt2;
    c.gg_inertia = 0.116 * kSlugFt2;
    return c;
}

// A stand-in rotor on shaft 0: demands `before` [N·m] until t_step, `after`
// from then on, publishing it as rotor.shaft_torque (D-053) and loading the
// shaft with it. Its reaction is internal, as an engine's (D-052).
class Load : public DynamicComponent {
public:
    Load(double before, double after, double t_step)
        : before_(before), after_(after), t_step_(t_step) {}
    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    std::string_view name() const noexcept override { return "rotor"; }
    std::vector<std::string> produces() const override {
        std::vector<std::string> out = {"rotor.shaft_torque"};
        for (int j = 0; j < 6; ++j) {
            out.push_back("rotor.shaft_torque_a" + std::to_string(j));
        }
        return out;
    }
    void bind(const CouplingRegistry& r) override {
        slot_ = r.find("rotor.shaft_torque", "rotor");
        for (std::size_t j = 0; j < 6; ++j) {
            slot_a_[j] = r.find("rotor.shaft_torque_a" + std::to_string(j), "rotor");
        }
    }
    void derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        const double q = ctx.t < t_step_ ? before_ : after_;
        out.Q0(0) = -q;
        if (out.coupling != nullptr) {
            out.coupling->set(slot_, q);
            for (const CouplingSlot s : slot_a_) {
                out.coupling->set(s, 0.0);  // a torque independent of the body's motion
            }
        }
    }

private:
    double before_;
    double after_;
    double t_step_;
    CouplingSlot slot_;
    std::array<CouplingSlot, 6> slot_a_;
};

// The component evaluated alone at the shaft speed `omega`, collective
// `collective` and rotor demand `demand` [N·m], body at rest.
struct Evaluation {
    std::vector<double> xdot;
    std::vector<double> G;
    std::vector<double> Gs;
    ComponentDerivs d;
};

class Harness {
public:
    explicit Harness(GenHelT700Config c) : engine_(std::move(c)) {
        slot_ = registry_.add("rotor.shaft_torque", "rotor");
        for (std::size_t j = 0; j < 6; ++j) {
            slot_a_[j] = registry_.add("rotor.shaft_torque_a" + std::to_string(j), "rotor");
        }
        store_ = CouplingOutputs(registry_.size());
        engine_.bind(registry_);
        const std::vector<ShaftInfo> info = {{"main", kGovernor}};
        engine_.bind_shafts(info);
    }
    const GenHelT700& engine() const { return engine_; }

    // The demand's sensitivity to the body acceleration [N·m per unit a].
    void set_demand_a(const std::array<double, 6>& a) { demand_a_ = a; }

    SimContext context(double omega, double collective, double demand) const {
        SimContext ctx;
        ctx.num_shafts = 1;
        ctx.shafts[0] = {0.0, omega};
        ctx.controls.collective = collective;
        store_.begin_evaluation();
        store_.set(slot_, demand);
        for (std::size_t j = 0; j < 6; ++j) {
            store_.set(slot_a_[j], demand_a_[j]);
        }
        ctx.coupling = &store_;
        return ctx;
    }

    // ẋ (a = 0, Ω̇ = 0) and the rest of the affine form; `full`'s readouts,
    // if set, are passed on (D-060).
    void evaluate(std::span<const double> x, double omega, double collective, double demand,
                  std::vector<double>& xdot, ComponentDerivs* full = nullptr) const {
        const SimContext ctx = context(omega, collective, demand);
        xdot.assign(GenHelT700::kNumStates, 0.0);
        g_.assign(6 * GenHelT700::kNumStates, 0.0);
        gs_.assign(kMaxShafts * GenHelT700::kNumStates, 0.0);
        ComponentDerivs d{xdot, g_, Wrench{}, Mat66::Zero()};
        d.Gs = gs_;
        d.readouts = full != nullptr ? full->readouts : nullptr;  // D-060
        engine_.derivatives(ctx, x, d);
        if (full != nullptr) {
            *full = d;
        }
    }
    const std::vector<double>& G() const { return g_; }
    const std::vector<double>& Gs() const { return gs_; }

private:
    GenHelT700 engine_;
    CouplingRegistry registry_;
    CouplingSlot slot_;
    std::array<CouplingSlot, 6> slot_a_;
    std::array<double, 6> demand_a_{};
    mutable CouplingOutputs store_;
    mutable std::vector<double> g_;
    mutable std::vector<double> gs_;
};

}  // namespace eyas::test::engine
