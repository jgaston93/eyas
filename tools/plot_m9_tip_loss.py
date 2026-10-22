"""Tip loss on the inflow (D-051) against Knight & Hefner's 1937 hover tests.

Reads the human-reviewed readings of Leishman 2nd ed. Figs. 3.2 (thrust) and 3.3
(power) for four untwisted rotors and compares the tip-loss readings through the
uniform-inflow closed forms (Leishman 2nd ed. Eqs. 3.33-3.37; Johnson 2013
Eqs. 3.111-3.115). Prints the statistics quoted in DECISIONS.md D-051 and
VALIDATION.md M9, and writes one figure.

    .venv/bin/python tools/plot_m9_tip_loss.py docs/validation/m9
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
REF = ROOT / "tests" / "validation" / "reference"
B = 0.97  # tip loss factor, as Leishman's theory curves (p. 124)

# Tip-loss readings: (lift to B·R, momentum disc radius / R).
VARIANTS = {
    "no tip loss": (1.0, 1.0),
    "lift cut only (GenHel)": (B, 1.0),
    "Johnson: lift cut + effective disc (D-051)": (B, B),
    "Leishman: full lift, inflow / B": (1.0, B),
}


def load(name):
    """The data rows (comment lines and the header row dropped)."""
    with (REF / name).open(encoding="utf-8") as f:
        rows = [line for line in f if not line.startswith("#")][1:]
    return np.loadtxt(rows, delimiter=",")


def ct_closed_form(sigma, theta, a, lift, disc):
    """C_T = (σa/2)(θL³/3 − λL²/2), λ = √(C_T/2)/disc, untwisted, uniform inflow."""
    k = sigma * a / 2.0
    c1 = k * lift ** 3 * theta / 3.0
    c2 = k * lift ** 2 / (2.0 * np.sqrt(2.0) * disc)
    x = (-c2 + np.sqrt(c2 * c2 + 4.0 * c1)) / 2.0  # x = √C_T
    return x * x


def main(out):
    thrust = load("knight_hefner_1937.csv")
    power = load("knight_hefner_1937_power.csv")
    s_t, th_t, ct_t = thrust[:, 0], np.radians(thrust[:, 1]), thrust[:, 2]
    use = np.degrees(th_t) >= 4.0  # below 4° the ±0.0001 reading error dominates
    sigmas = np.unique(s_t)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
    print("Thrust against collective, lift slope fitted per variant (θ0 ≥ 4°):")
    for i, (name, (lift, disc)) in enumerate(VARIANTS.items()):
        best = None
        for a in np.linspace(4.5, 6.5, 401):
            e = 100.0 * (ct_closed_form(s_t, th_t, a, lift, disc) / ct_t - 1.0)
            r = np.sqrt(np.mean(e[use] ** 2))
            if best is None or r < best[0]:
                best = (r, a, e)
        r, a, e = best
        trend = "  ".join(f"σ {s:g}: {e[use & (s_t == s)].mean():+.1f} %" for s in sigmas)
        print(f"  {name:44s} a = {a:.2f}/rad, rms {r:.1f} %   {trend}")
        th = np.radians(np.linspace(0.0, 13.0, 60))
        for j, s in enumerate(sigmas):
            a1.plot(np.degrees(th), ct_closed_form(s, th, a, lift, disc), color=f"C{i}", lw=1.0,
                    ls="-" if i == 2 else ":", label=name if j == 0 else None)
    for j, s in enumerate(sigmas):
        m = s_t == s
        a1.plot(np.degrees(th_t[m]), ct_t[m], "ko", mfc="none" if j % 2 else "k", ms=4,
                label="Knight & Hefner" if j == 0 else None)
    a1.set(xlabel="θ0 [deg]", ylabel="C_T", title="Thrust (Leishman Fig. 3.2), lift slope fitted")
    a1.legend(fontsize=7)
    a1.grid(alpha=0.3)

    # Pair power with thrust at the same σ and θ0.
    def key(s, t):
        assert abs(t - round(t)) < 1e-9, f"θ0 = {t} isn't a whole degree"
        return (round(s, 3), round(t))
    ct_at = {}
    for s, t, c in thrust:
        assert key(s, t) not in ct_at, f"duplicate thrust reading at σ {s}, θ0 {t}"
        ct_at[key(s, t)] = c
    # Readings at θ0 ≥ 4°, and below that only where a thrust reading exists
    # to pair with (Fig. 3.2 reads none at 0° and few under 4°).
    pairs = np.array([(s, ct_at[key(s, t)], cp) for s, t, cp in power
                      if t >= 4 and key(s, t) in ct_at])
    missing = [(s, t) for s, t, _ in power if t >= 4 and key(s, t) not in ct_at]
    print("power readings at θ0 ≥ 4° with no thrust reading to pair:", missing or "none")
    s_p, ct_p, cp_p = pairs.T
    ideal = ct_p ** 1.5 / np.sqrt(2.0)
    print(f"\nPower against thrust, {len(pairs)} paired points, C_P = κ C_T^1.5/√2 + σC_d0/8:")
    fits = {}
    for name, kappa in (("κ = 1 (ideal momentum)", 1.0), ("κ = 1/B (tip loss, D-051)", 1.0 / B)):
        resid = cp_p - kappa * ideal
        w = s_p / 8.0 / cp_p
        cd = np.sum(resid / cp_p * w) / np.sum(w * w)
        e = 100.0 * ((kappa * ideal + s_p * cd / 8.0) / cp_p - 1.0)
        fits[name] = (kappa, cd)
        print(f"  {name:28s} C_d0 = {cd:.4f}, rms {np.sqrt(np.mean(e ** 2)):.1f} %")
    A = np.c_[ideal, s_p / 8.0] / cp_p[:, None]
    kappa, cd = np.linalg.lstsq(A, np.ones_like(cp_p), rcond=None)[0]
    e = 100.0 * (A @ [kappa, cd] - 1.0)
    fits["κ fitted"] = (kappa, cd)
    print(f"  {'κ fitted':28s} κ = {kappa:.3f}, C_d0 = {cd:.4f}, rms {np.sqrt(np.mean(e ** 2)):.1f} %")
    cl = 6.0 * ct_p / s_p  # mean lift coefficient, Johnson 2013 Eq. 3.119
    A3 = np.c_[ideal, s_p / 8.0, s_p / 8.0 * cl ** 2] / cp_p[:, None]
    sol = np.linalg.lstsq(A3, np.ones_like(cp_p), rcond=None)[0]
    rng = np.random.default_rng(1)
    boot = [np.linalg.lstsq(A3[i], np.ones(len(i)), rcond=None)[0][0]
            for i in (rng.integers(0, len(cp_p), len(cp_p)) for _ in range(2000))]
    print(f"  {'κ fitted, C_d = d0 + d2 cl²':28s} κ = {sol[0]:.3f} "
          f"(bootstrap 5-95 %: {np.percentile(boot, 5):.3f}-{np.percentile(boot, 95):.3f})")

    ct = np.linspace(0.0, 0.011, 100)
    for j, s in enumerate(sigmas):
        m = s_p == s
        a2.plot(ct_p[m], cp_p[m], "o", color=f"C{j}", ms=5, label=f"σ = {s:g}")
        for (name, (kap, cd)), ls in zip(fits.items(), (":", "--", "-")):
            a2.plot(ct, kap * ct ** 1.5 / np.sqrt(2.0) + s * cd / 8.0, color=f"C{j}", ls=ls,
                    lw=0.9, label=name if j == 0 else None)
    a2.set(xlabel="C_T", ylabel="C_P",
           title="Power (Leishman Fig. 3.3) against thrust (Fig. 3.2), C_d0 fitted")
    a2.legend(fontsize=7)
    a2.grid(alpha=0.3)
    fig.suptitle("Knight & Hefner (1937), untwisted rotors: tip loss on the inflow (D-051)")
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "m9_tip_loss_knight_hefner.png", dpi=120)


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "validation" / "m9")
