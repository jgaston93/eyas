#include "analysis/held_orbit.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "analysis/trim.hpp"
#include "core/rigid_body.hpp"
#include "util/error.hpp"

namespace eyas {

HeldIntegrator::HeldIntegrator(const Executive& ex, Environment env, double period, int steps)
    : ex_(ex),
      env_(std::move(env)),
      period_(period),
      steps_(steps),
      n_(ex.num_states()),
      tmp_(n_, 0.0),
      k1_(n_, 0.0),
      k2_(n_, 0.0),
      k3_(n_, 0.0),
      k4_(n_, 0.0) {}

void HeldIntegrator::step(int s, const Controls& u, std::vector<double>& x, Vec6& integral,
                          std::array<std::vector<double>, 4>* stages) {
    const double h = period_ / steps_;
    Vec6 n1;
    Vec6 n2;
    Vec6 n3;
    Vec6 n4;
    const std::size_t b = body_layout::kSize;
    std::copy(x.begin(), x.end(), tmp_.begin());  // the body part stays
    const auto stage = [&](const std::vector<double>& k, double scale) {
        for (std::size_t i = b; i < n_; ++i) {
            tmp_[i] = x[i] + scale * k[i];
        }
    };
    const auto eval = [&](std::size_t which, double t, const std::vector<double>& xs,
                          std::vector<double>& k, Vec6& net) {
        if (stages != nullptr) {
            (*stages)[which] = xs;
        }
        if (!ex_.held_derivatives(t, xs, u, env_, k, net)) {
            ++unconverged_;
        }
    };
    const double t = period_ * static_cast<double>(s) / steps_;
    eval(0, t, x, k1_, n1);
    stage(k1_, 0.5 * h);
    eval(1, t + 0.5 * h, tmp_, k2_, n2);
    stage(k2_, 0.5 * h);
    eval(2, t + 0.5 * h, tmp_, k3_, n3);
    stage(k3_, h);
    eval(3, t + h, tmp_, k4_, n4);
    for (std::size_t i = b; i < n_; ++i) {
        x[i] += h / 6.0 * (k1_[i] + 2.0 * k2_[i] + 2.0 * k3_[i] + k4_[i]);
    }
    integral += h / 6.0 * (n1 + 2.0 * n2 + 2.0 * n3 + n4);
}

Environment trim_environment(const TrimCondition& c) {
    Environment env;
    env.isa_delta_T = c.isa_delta_T;
    env.wind_n = c.wind_n;
    env.ground_height = std::numeric_limits<double>::quiet_NaN();
    return env;
}

std::vector<double> held_orbit_start(const Executive& ex, const TrimCondition& c,
                                     const TrimResult& r) {
    const std::size_t nc = ex.num_states() - body_layout::kSize;
    if (r.component_states.size() != nc) {
        throw InitError("held orbit: the trim's component states don't fit this model");
    }
    BodyState body;
    if (!trim_body_state(c, r.pitch, r.roll, ex.origin_altitude(), body)) {
        throw InitError("held orbit: the trim attitude admits no body state");
    }
    std::vector<double> x(ex.num_states(), 0.0);
    pack_body(body, x);
    std::copy(r.component_states.begin(), r.component_states.end(),
              x.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize));
    return x;
}

HeldOrbit held_orbit(const Executive& ex, const TrimCondition& c, const TrimResult& r, int samples,
                     int steps) {
    if (samples <= 0 || steps <= 0 || steps % samples != 0) {
        throw InitError("held orbit: the samples must divide the steps per revolution");
    }
    std::vector<double> x = held_orbit_start(ex, c, r);
    HeldIntegrator held(ex, trim_environment(c), trim_period(ex), steps);
    HeldOrbit orbit;
    orbit.states.reserve(static_cast<std::size_t>(samples));
    const int every = steps / samples;
    Vec6 ignored = Vec6::Zero();
    for (int s = 0; s < steps; ++s) {
        if (s % every == 0) {
            orbit.states.push_back(x);
        }
        held.step(s, r.controls, x, ignored);
    }
    orbit.unconverged = held.unconverged();
    return orbit;
}

}  // namespace eyas
