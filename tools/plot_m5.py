"""Plots for the M5 entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m5_cases executable and the NACA TN 3044
readings, and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m5_cases build/m5_cases
    .venv/bin/python tools/plot_m5.py build/m5_cases docs/validation/m5
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REFERENCE = Path(__file__).resolve().parent.parent / "tests/validation/reference/carpenter_fridovich_1953.csv"


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def reference():
    rows = {}
    with open(REFERENCE, encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.startswith("#")]
    for ln in lines[1:]:
        case, qty, t, value = ln.split(",")
        rows.setdefault((case, qty), []).append((float(t) if t else np.nan, float(value)))
    return {k: np.array(v) for k, v in rows.items()}


def summary(t, ct, lam):
    """Peak C_T, its time, final C_T, and the time at which λ0 reaches 0.9 of its final value."""
    i = int(np.argmax(ct))
    ratio = lam / lam[-1]
    t90 = t[int(np.argmax(ratio >= 0.9))]
    return ct[i], t[i], ct[-1], t90


def ramps(src, out):
    d = load(src / "ramps.csv")
    ref = reference()
    titles = {"b": "Fig. 3(b), ≈48°/s", "c": "Fig. 3(c), ≈20°/s", "d": "Fig. 3(d), ≈6°/s"}
    fig, axes = plt.subplots(2, 3, figsize=(14, 7.5), sharey="row")
    rows = []
    for col, (case, title) in enumerate(titles.items()):
        ax_ct, ax_v = axes[0, col], axes[1, col]
        m = (d["case"] == case) & (d["model"] == "pitt_peters")
        cts, vs = [], []
        for twist, colour in ((0.0, "C0"), (-8.0, "C1")):
            for a in (0.105, 0.1075, 0.110):
                k = m & np.isclose(d["twist_deg"], twist) & np.isclose(d["a_per_deg"], a)
                t, ct, lam = d["t"][k], d["ct"][k], d["lambda0"][k]
                cts.append(ct)
                vs.append(lam / lam[-1])
                if np.isclose(a, 0.1075):
                    label = f"Eyas Pitt–Peters, twist {twist:g}°"
                    ax_ct.plot(t, ct, color=colour, lw=1.6, label=label)
                    ax_v.plot(t, lam / lam[-1], color=colour, lw=1.6, label=label)
                    rows.append((case, twist, *summary(t, ct, lam)))
        ax_ct.fill_between(t, np.min(cts, axis=0), np.max(cts, axis=0), color="C0", alpha=0.15, lw=0,
                           label="Eyas spread (a, twist)")
        u = (d["case"] == case) & (d["model"] == "uniform")
        ax_ct.plot(d["t"][u], d["ct"][u], "--", color="0.4", lw=1.2, label="Eyas uniform inflow (no lag)")
        lu = d["lambda0"][u]
        ax_v.plot(d["t"][u], lu / lu[-1], "--", color="0.4", lw=1.2, label="Eyas uniform inflow (no lag)")

        pk = ref[(case, "ct_peak")]
        st = ref[(case, "ct_steady")]
        ax_ct.plot(pk[:, 0], pk[:, 1], "kv", ms=8, label="TN 3044 peak C_T")
        ax_ct.axhline(st[0, 1], color="k", lw=1, ls=":", label="TN 3044 steady C_T")
        if (case, "v_ratio") in ref:
            v = ref[(case, "v_ratio")]
            ax_v.plot(v[:, 0], v[:, 1], "ko", ms=5, label="TN 3044 v/v_s")
        t90 = ref[(case, "t90")][0, 1]
        ax_v.plot([t90], [0.9], "ks", ms=7, mfc="none", label="TN 3044 v/v_s = 0.9")

        th = ref[(case, "theta")]
        ax_ct.set_title(f"{title}: 0→12° by {th[-1, 0]:g} s, {ref[(case, 'rpm')][0, 1]:g} rpm")
        ax_v.set_xlabel("Time [s]")
        for ax in (ax_ct, ax_v):
            ax.grid(alpha=0.3)
            ax.set_xlim(0, d["t"][m].max())
    axes[0, 0].set_ylabel("C_T")
    axes[1, 0].set_ylabel("λ0 / λ0,final  (Eyas)   ·   v / v_s  (TN 3044)")
    axes[0, 2].legend(fontsize=7.5, loc="lower right")
    axes[1, 2].legend(fontsize=7.5, loc="lower right")
    fig.suptitle("Collective ramps on a full-scale rotor in hover: Eyas (Pitt–Peters) vs Carpenter & Fridovich, NACA TN 3044 (1953)")
    fig.tight_layout()
    fig.savefig(out / "m5_naca_ramps.png", dpi=110)
    plt.close(fig)

    print("case twist  peak_ct  t_peak  final_ct  t90")
    for r in rows:
        print(f"{r[0]}  {r[1]:5g}  {r[2]:.5f}  {r[3]:.3f}  {r[4]:.5f}  {r[5]:.3f}")


def step_anchor(src, out):
    d = load(src / "step_anchor.csv")
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    ax.plot(d["t"], d["dct"] * 1e4, color="C0", lw=2, label="Eyas (production rotor)")
    ax.plot(d["t"], d["closed_form"] * 1e4, "--", color="k", lw=1.2,
            label=r"Closed form: (σa/6)δθ₀ [C′ + (1 − C′) $e^{s_0 t}$]")
    ax.set_xlabel("Time after the step [s]")
    ax.set_ylabel("ΔC_T × 10⁴")
    ax.set_title("Step collective, 0.25° at θ₀ = 12°, hub fixed\nJohnson 2013 Eqs. 11.148–11.149")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "m5_step_anchor.png", dpi=110)
    plt.close(fig)


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    ramps(src, out)
    step_anchor(src, out)


if __name__ == "__main__":
    main()
