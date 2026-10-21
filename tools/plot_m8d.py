"""Plots for the M8d entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m8d_cases executable (the vortex-ring
correction, D-049) and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m8d_cases build/m8d_cases
    .venv/bin/python tools/plot_m8d.py build/m8d_cases docs/validation/m8
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

A = dict(color="C0", ls="-", lw=2)
B = dict(color="C1", ls="--", lw=2)

# Johnson's stability boundary, Table 3 step 3.1 with Table 4 (vz/v_h).
VZ_N, VZ_X, VX_M = -0.45, -1.5, 0.95


def load(path, skip=1):
    """A CSV with a header row, after `skip` comment lines."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=skip)


def boundary(vx):
    s = 1.0 - (vx / VX_M) ** 2
    mid, half = 0.5 * (VZ_N + VZ_X), 0.5 * (VZ_N - VZ_X)
    return mid + half * s ** 0.2, mid - half * s ** 1.5


def curves(data, out):
    """Johnson's Figs. 38 and 39 axes: (vz + v)/v_h and v/v_h against vz/v_h."""
    d = load(data / "m8d_curves.csv")
    fig, ax = plt.subplots(2, 2, figsize=(11, 9), sharex=True)
    shown = (0.0, 0.4, 0.6, 0.8, 1.0, 1.4, 2.0, 4.0)
    for j, (col, title) in enumerate((("v_baseline", "baseline (cf. Johnson Fig. 38)"),
                                      ("v_vrs", "VRS model (cf. Johnson Fig. 39)"))):
        for k, vx in enumerate(shown):
            m = np.isclose(d["vx"], vx)
            z = d["vz"][m]
            ax[0, j].plot(z, z + d[col][m], color=f"C{k}", lw=1.8, label=f"vx/v_h = {vx:g}")
            ax[1, j].plot(z, d[col][m], color=f"C{k}", lw=1.8)
        ax[0, j].set_title(title)
        ax[0, j].plot([-3, 0.5], [-3, 0.5], "k--", lw=0.8)
        ax[0, j].set_ylim(-2, 2)
        ax[1, j].plot([-3, 0], [3, 0], "k--", lw=0.8)
        ax[1, j].set_ylim(0, 3)
        for a in ax[:, j]:
            a.axhline(0, color="k", lw=0.5)
            a.axvline(0, color="k", lw=0.5)
            a.set_xlim(-3, 0.5)
            a.grid(True, alpha=0.3)
        ax[1, j].set_xlabel("vz / v_h  (climb +)")
    ax[0, 0].set_ylabel("(vz + v) / v_h")
    ax[1, 0].set_ylabel("v / v_h")
    ax[0, 1].legend(fontsize=8, loc="upper left")
    fig.suptitle("Johnson NASA/TP-2005-213477 Table 3 as Eyas has it (D-049)")
    fig.tight_layout()
    fig.savefig(out / "m8d_curves.png", dpi=110)
    plt.close(fig)

    # The fades: just below each edge, Johnson as published jumps to momentum
    # theory at the edge; Eyas has faded to it.
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for a, (col, edge) in zip(ax, (("baseline", 0.75), ("vrs", 0.95))):
        below = np.isclose(d["vx"], edge - 0.001)
        at = np.isclose(d["vx"], edge)
        z = d["vz"][below]
        a.plot(z, z + d[f"v_{col}_unfaded"][below], label=f"Johnson as published, vx/v_h = {edge - 0.001:g}",
               **B)
        a.plot(z, z + d[f"v_{col}"][below], label=f"Eyas (faded), vx/v_h = {edge - 0.001:g}", **A)
        a.plot(d["vz"][at], d["vz"][at] + d[f"v_{col}"][at], color="0.3", ls=":", lw=1.5,
               label=f"both at vx/v_h = {edge:g} (momentum theory)")
        a.set_title(f"{'baseline' if col == 'baseline' else 'VRS model'}: Johnson jumps at vx/v_h = {edge}")
        a.set_xlim(-2.6, 0.2)
        a.set_xlabel("vz / v_h")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    ax[0].set_ylabel("(vz + v) / v_h")
    fig.suptitle("Eyas's fades below Johnson's edges (D-049)")
    fig.tight_layout()
    fig.savefig(out / "m8d_fades.png", dpi=110)
    plt.close(fig)


def heave(data, out):
    d = load(data / "m8d_heave.csv")
    v = d["vrs"] == 1
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.3))
    a = ax[0]
    a.plot(d["vz_vh"][v], d["zw_qs"][v], label="VRS correction, deficit quasi-static", marker="o",
           ms=3, **A)
    a.plot(d["vz_vh"][v], d["heave_lag"][v], label="VRS correction, with its lag (heave root)",
           color="C2", ls="-.", lw=2, marker="^", ms=3)
    a.plot(d["vz_vh"][~v], d["zw_qs"][~v], label="momentum theory (no correction)", marker="s",
           ms=3, **B)
    n, x = boundary(np.mean(d["vx_vh"][v]))
    for b in (n, x):
        a.axvline(b, color="0.4", ls=":", lw=1)
    a.axhline(0, color="k", lw=0.6)
    a.invert_yaxis()
    a.set_xlabel("main-rotor vz / v_h")
    a.set_ylabel("heave damping Z_w [1/s]  (positive: unstable)")
    a.set_title("UH-60A, vertical descent, collective fixed (cf. Johnson Fig. 52)")
    a.legend(fontsize=8)
    a.grid(True, alpha=0.3)
    a = ax[1]
    a.plot(d["rate_mps"][v], d["collective"][v], marker="o", ms=3, label="VRS correction", **A)
    a.plot(d["rate_mps"][~v], d["collective"][~v], marker="s", ms=3, label="momentum theory", **B)
    a.set_xlabel("vertical descent rate [m/s]")
    a.set_ylabel("trimmed collective [0..1]")
    a.set_title("collective to hold a steady vertical descent")
    a.legend(fontsize=8)
    a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "m8d_heave.png", dpi=110)
    plt.close(fig)


def encounter(data, out):
    d = load(data / "m8d_encounter.csv")
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for on, style, name in ((1, A, "VRS correction"), (0, B, "momentum theory")):
        m = d["vrs"] == on
        ax[0].plot(d["t"][m], d["collective"][m], label=name, **style)
        ax[1].plot(d["t"][m], d["vz_vh"][m], label=name, **style)
        ax[2].plot(d["t"][m], d["vz_mps"][m], label=name, **style)
    ax[0].set_ylabel("collective [0..1]")
    ax[1].set_ylabel("Vz / v_h  (climb +)")
    ax[2].set_ylabel("vertical speed [m/s]  (climb +)")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    ax[1].axhline(VZ_N, color="0.4", ls=":", lw=1)
    ax[1].axhline(VZ_X, color="0.4", ls=":", lw=1)
    fig.suptitle("UH-60A vortex-ring entry: collective lowered in three steps, whole AFCS on\n"
                 "(the shape of Johnson's D6075 vertical case, Figs. 56–57)")
    fig.tight_layout()
    fig.savefig(out / "m8d_encounter.png", dpi=110)
    plt.close(fig)


def main():
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    curves(data, out)
    heave(data, out)
    encounter(data, out)


if __name__ == "__main__":
    main()
