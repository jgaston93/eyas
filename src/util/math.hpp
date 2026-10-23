#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

// Small numeric helpers shared by the force models. No allocation, no
// exceptions (spec §7).
namespace eyas {

// smoothstep(t) = t²(3 − 2t) on t clamped to [0, 1]: 0 and 1 at the ends,
// zero slope at both. Used for the fades that keep a derivative continuous
// (D-031, D-032 item 2).
inline double smoothstep(double t) noexcept {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// An angle wrapped into (−π, π] [rad], e.g. a heading difference taken the
// short way round.
inline double wrap_pi(double a) noexcept {
    a = std::remainder(a, 2.0 * std::numbers::pi);
    return a == -std::numbers::pi ? std::numbers::pi : a;
}

// An angle wrapped into [0, 2π) [rad], e.g. a rotor azimuth; exactly 0, not
// 2π, where the fix-up of a tiny negative angle would round up to 2π.
inline double wrap_2pi(double a) noexcept {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    double w = std::fmod(a, kTwoPi);
    if (w < 0.0) {
        w += kTwoPi;
    }
    return w >= kTwoPi ? 0.0 : w;
}

// The three points Brent's method keeps: a (the previous b), b (the best
// estimate) and c (with b, the bracket).
enum class BrentPoint : unsigned char { a, b, c };

// Brent's method (Brent, "Algorithms for Minimization without Derivatives",
// 1973, Ch. 4) for a root of f on a sign-changing bracket [a, b], with
// fa = f(a), fb = f(b): inverse quadratic interpolation or secant steps,
// falling back to bisection, always inside the bracket [b, c]. Superlinear,
// and never slower than bisection. Returns b once |c − b|/2 ≤ 2ε|b| + tol/2
// or f(b) = 0 (converged = true), or when evals reaches max_evals
// (converged = false). f(x) must count itself in evals. A caller that keeps
// data with each point (the loads of a blade-element pass) passes `moved`,
// called as moved(to, from) whenever one point is copied to another, and
// f(x) stores its data as point b.
template <typename F, typename Moved>
double brent(F&& f, double a, double fa, double b, double fb, double tol, int max_evals,
             const int& evals, bool& converged, Moved&& moved) noexcept {
    using P = BrentPoint;
    double c = a;
    double fc = fa;
    moved(P::c, P::a);
    double d = b - a;
    double e = d;
    converged = false;
    while (evals < max_evals) {
        if ((fb > 0.0) == (fc > 0.0)) {
            c = a;
            fc = fa;
            moved(P::c, P::a);
            d = b - a;
            e = d;
        }
        if (std::abs(fc) < std::abs(fb)) {
            a = b;
            b = c;
            c = a;
            fa = fb;
            fb = fc;
            fc = fa;
            moved(P::a, P::b);
            moved(P::b, P::c);
            moved(P::c, P::a);
        }
        const double t = 2.0 * std::numeric_limits<double>::epsilon() * std::abs(b) + 0.5 * tol;
        const double xm = 0.5 * (c - b);
        if (std::abs(xm) <= t || fb == 0.0) {
            converged = true;
            return b;
        }
        if (std::abs(e) >= t && std::abs(fa) > std::abs(fb)) {
            const double sr = fb / fa;
            double p = 0.0;
            double q = 0.0;
            if (a == c) {
                p = 2.0 * xm * sr;
                q = 1.0 - sr;
            } else {
                const double qa = fa / fc;
                const double r = fb / fc;
                p = sr * (2.0 * xm * qa * (qa - r) - (b - a) * (r - 1.0));
                q = (qa - 1.0) * (r - 1.0) * (sr - 1.0);
            }
            if (p > 0.0) {
                q = -q;
            }
            p = std::abs(p);
            if (2.0 * p < std::min(3.0 * xm * q - std::abs(t * q), std::abs(e * q))) {
                e = d;
                d = p / q;
            } else {
                d = xm;
                e = d;
            }
        } else {
            d = xm;
            e = d;
        }
        a = b;
        fa = fb;
        moved(P::a, P::b);
        b += std::abs(d) > t ? d : std::copysign(t, xm);
        fb = f(b);
    }
    return b;
}

// Brent's method with no data kept per point.
template <typename F>
double brent(F&& f, double a, double fa, double b, double fb, double tol, int max_evals,
             const int& evals, bool& converged) noexcept {
    return brent(std::forward<F>(f), a, fa, b, fb, tol, max_evals, evals, converged,
                 [](BrentPoint, BrentPoint) noexcept {});
}

}  // namespace eyas
