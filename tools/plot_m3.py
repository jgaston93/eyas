"""Plots for the M3 entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m3_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m3_cases build/m3_cases
    .venv/bin/python tools/plot_m3.py build/m3_cases docs/validation/m3
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def refinement(src, out):
    d = load(src / "refinement.csv")
    fig, ax = plt.subplots(figsize=(8, 5))
    labels = {
        "hover_uniform": "hover, uniform spacing",
        "hover_equal_area": "hover, equal-area spacing",
        "tip_loss": "hover, tip loss B = 0.97",
        "forward": "μ ≈ 0.2, rates, cyclic, tilt",
        "reverse_stall": "μ ≈ 0.45, reverse flow, stall (nonlinear table)",
    }
    for i, (case, label) in enumerate(labels.items()):
        m = d["case"] == case
        n = d["N"][m]
        c = f"C{i}"
        ax.loglog(n, d["error"][m], "-o", color=c, ms=3, label=label)
        r = d["richardson_error"][m]
        ok = np.isfinite(r)
        ax.loglog(n[ok], np.maximum(r[ok], 1e-16), "--", color=c, lw=1)
    n = np.array([15, 3840])
    ax.loglog(n, 0.3 * (n / 15.0) ** -2, ":", color="gray", label="slope −2 (midpoint rule)")
    ax.axhline(1e-9, color="red", lw=0.8)
    ax.text(16, 1.5e-9, "1e-9 gate (D-007 layer 1)", color="red", fontsize=8)
    ax.set(xlabel="elements per blade N", ylabel="max relative error vs exact reference",
           title="Blade-element loop vs exact-equation reference\nsolid: raw; dashed: Richardson (4E(2N) − E(N))/3")
    ax.legend(fontsize=8)
    ax.grid(True, which="both", lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m3_refinement.png", dpi=120)


def closed_form(src, out):
    d = load(src / "closed_form.csv")
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.5))
    for i, s in enumerate(np.unique(d["sigma"])):
        m = d["sigma"] == s
        th = d["theta0_deg"][m]
        a1.plot(th, d["ct"][m], color=f"C{i}", label=f"Eyas, σ = {s:g}")
        a1.plot(th, d["ct_cf"][m], "--", color=f"C{i}", lw=1)
        k = th > 0.4
        a2.plot(th[k], 100 * (d["ct"][m][k] / d["ct_cf"][m][k] - 1), color=f"C{i}", label=f"C_T, σ = {s:g}")
        a2.plot(th[k], 100 * (d["cp"][m][k] / d["cp_cf"][m][k] - 1), ":", color=f"C{i}", label=f"C_P, σ = {s:g}")
    a1.set(xlabel="θ0 [deg]", ylabel="C_T", title="Hover C_T: Eyas vs Leishman Eqs. 3.25, 3.32 (dashed)")
    a1.legend(fontsize=8)
    a2.set(xlabel="θ0 [deg]", ylabel="Eyas / closed form − 1 [%]",
           title="Residual (small-angle approximations)")
    a2.legend(fontsize=7, ncol=2)
    for a in (a1, a2):
        a.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m3_closed_form.png", dpi=120)


def knight_hefner(src, out):
    model = load(src / "knight_hefner.csv")
    ref = Path(__file__).resolve().parent.parent / "tests/validation/reference/knight_hefner_1937.csv"
    data = np.genfromtxt(ref, delimiter=",", comments="#", skip_header=4,
                         names=["sigma", "theta0_deg", "ct"])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.5))
    markers = {0.042: "o", 0.064: "o", 0.085: "s", 0.106: "s"}
    filled = {0.042: True, 0.064: False, 0.085: True, 0.106: False}
    rows = []
    for i, s in enumerate(sorted(markers)):
        c = f"C{i}"
        m = np.isclose(model["sigma"], s)
        th = model["theta0_deg"][m]
        a1.plot(th, model["ct_b100"][m], color=c, lw=1.2)
        a1.plot(th, model["ct_b097"][m], "--", color=c, lw=1)
        d = np.isclose(data["sigma"], s)
        a1.plot(data["theta0_deg"][d], data["ct"][d], markers[s], color=c,
                mfc=c if filled[s] else "none", label=f"Knight & Hefner, σ = {s:g}")
        for b, key, ls in ((1.0, "ct_b100", "-"), (0.97, "ct_b097", ":")):
            pred = np.interp(data["theta0_deg"][d], th, model[key][m])
            err = 100 * (pred / data["ct"][d] - 1)
            a2.plot(data["theta0_deg"][d], err, markers[s] + ls, color=c,
                    mfc=c if filled[s] else "none", label=f"σ = {s:g}, B = {b:g}")
            rows.append((s, b, data["theta0_deg"][d], err))
    a1.set(xlabel="θ0 [deg]", ylabel="C_T", title="Eyas (solid B = 1, dashed B = 0.97) vs 1937 data")
    a1.legend(fontsize=8)
    a2.axhline(0, color="gray", lw=0.8)
    a2.set(xlabel="θ0 [deg]", ylabel="Eyas / measured − 1 [%]", title="Difference from the measurements")
    a2.legend(fontsize=7, ncol=2)
    for a in (a1, a2):
        a.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m3_knight_hefner.png", dpi=120)
    # Below 4° the ±0.0001 reading uncertainty dominates the relative error.
    for s, b, th, err in rows:
        k = th >= 4.0
        print(f"σ = {s:g}, B = {b:g}: θ0 ≥ 4°: [{err[k].min():+.1f}, {err[k].max():+.1f}] %")


def azimuth(src, out):
    d = load(src / "azimuth.csv")
    psi = d["psi_deg"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
    a1.plot(psi, d["T_hover"] / 1e3, label="thrust [kN], hover")
    a1.plot(psi, d["T_fwd"] / 1e3, label="thrust [kN], 43 m/s")
    a1.set(xlabel="rotor azimuth ψ [deg]", ylabel="kN", title="Thrust: steady in hover, 4/rev in forward flight")
    a2.plot(psi, d["Q_hover"] / 1e3, label="hover")
    a2.plot(psi, d["Q_fwd"] / 1e3, label="43 m/s")
    a2.set(xlabel="rotor azimuth ψ [deg]", ylabel="torque [kN·m]", title="Torque")
    for a in (a1, a2):
        a.legend(fontsize=8)
        a.grid(True, lw=0.3)
        a.set_xticks(range(0, 361, 90))
    for a, key in ((a1, "T_hover"), (a2, "Q_hover")):
        spread = np.ptp(d[key]) / np.mean(d[key])
        a.text(5, np.mean(d[key]) / 1e3, f"hover: peak-to-peak / mean = {spread:.1e}\n", fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m3_azimuth.png", dpi=120)


def inflow(src, out):
    d = load(src / "inflow.csv")
    vh = d["lambda_i"][d["descent"] == 0][0]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
    x = d["mu_z"] / vh
    a1.plot(x, d["lambda_i"] / vh, ".-", ms=3)
    a1.axvspan(0, 2, color="orange", alpha=0.15, label="approximate vortex-ring region")
    a1.set(xlabel="descent rate / hover induced velocity, μ_z/λ_h", ylabel="λ_i / λ_h",
           title="Uniform momentum inflow vs vertical speed")
    a1.legend(fontsize=8)
    a2.plot(x, d["passes"], ".-", ms=3)
    a2.axhline(30, color="red", lw=0.8)
    a2.text(x[0], 28.5, "pass limit", color="red", fontsize=8)
    a2.set(xlabel="μ_z/λ_h", ylabel="blade-element passes", title="Inflow solve cost (all converged)")
    for a in (a1, a2):
        a.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m3_inflow.png", dpi=120)


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    refinement(src, out)
    closed_form(src, out)
    knight_hefner(src, out)
    azimuth(src, out)
    inflow(src, out)


if __name__ == "__main__":
    main()
