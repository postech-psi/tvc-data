"""
The (A, B) -> thrust surface, and a bivariate cubic fitted to it.

Input is `out/pwm_grid_thrust.csv` -- the 121 steady-state points produced by
`pwm_grid_thrust.py`, each already the settle-trimmed, MAD-rejected mean of one
~4 s commanded hold. No voltage correction is applied here, by request; the
battery caveat from that script still stands and is restated on the figure.

The model
---------
Commands are centred and scaled to keep the Vandermonde matrix conditioned --
raw microseconds cubed overflows the fit:

    x = (A - 1500) / 500        y = (B - 1500) / 500        both in [-1, 1]

and thrust is a full bivariate cubic, every term with i + j <= 3:

    F(x,y) = c00 + c10 x + c01 y
                 + c20 x^2 + c11 x y + c02 y^2
                 + c30 x^3 + c21 x^2 y + c12 x y^2 + c03 y^3

10 coefficients, solved by ordinary least squares (`np.linalg.lstsq`) over the
121 points. Unweighted on purpose: the per-point SEMs describe within-step noise
only, and the dominant error here is a per-run offset (see below), so weighting
by SEM would chase the wrong error term.

Why cubic is the right stopping point
-------------------------------------
Degrees 1-5 are all fitted and reported. RMS residual falls 0.90 -> 0.28 -> 0.25
-> 0.22 -> 0.18 N for degrees 1-5, so past quadratic the gains are small. The
reason to stop is not the RMS but what the residual is *made of*: decomposing
the cubic's residual by source run shows 79 % of it is a constant offset shared
by all 11 points of a run (A=1400 sits +0.50 N high across its whole sweep,
A=1300 sits -0.24 N low). That is run-to-run reproducibility -- the same +/-0.14 N
scatter the voltage/thrust analysis found -- not curvature the surface is missing.
Degree 4 and 5 shrink the total RMS while that between-run share *grows* to 87 %,
i.e. the extra terms are absorbing per-run offsets into surface shape. That is
fitting the session, not the rig.

So: cubic is enough, and the honest accuracy statement is that its 0.25 N RMS is
mostly a floor set by the data, not by the polynomial.
"""

import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "out", "pwm_grid_thrust.csv")

CENTER = 1500.0
SCALE = 500.0
DEGREE = 3


def norm(a_us, b_us):
    """Commands in microseconds -> the [-1, 1] coordinates the fit uses."""
    return (a_us - CENTER) / SCALE, (b_us - CENTER) / SCALE


def terms(deg):
    """Exponent pairs (i, j) of a full bivariate polynomial, i + j <= deg."""
    return [(i, j) for i in range(deg + 1) for j in range(deg + 1)
            if i + j <= deg]


def design(x, y, deg):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    return np.column_stack([np.power(x, i) * np.power(y, j)
                            for i, j in terms(deg)])


# numpy 2.0 on Accelerate raises spurious divide/overflow warnings from matmul
# on small well-conditioned problems. Checked before silencing: the design
# matrix is finite with condition number 7.8, and `X @ coef` agrees with an
# explicit einsum to 3.6e-15, so the arithmetic is sound and only the warning
# is wrong.
np.seterr(divide="ignore", over="ignore", invalid="ignore")


def fit(x, y, t, deg=DEGREE):
    X = design(x, y, deg)
    coef, *_ = np.linalg.lstsq(X, t, rcond=None)
    resid = t - X @ coef
    ss_tot = float(np.sum((t - t.mean()) ** 2))
    r2 = 1 - float(np.sum(resid ** 2)) / ss_tot
    return coef, resid, r2


def formula(coef, deg=DEGREE):
    """The fitted polynomial written out, for printing."""
    out = []
    for c, (i, j) in zip(coef, terms(deg)):
        if i == 0 and j == 0:
            out.append("%+.4f" % c)
            continue
        mono = ""
        if i:
            mono += " x" + ("^%d" % i if i > 1 else "")
        if j:
            mono += " y" + ("^%d" % j if j > 1 else "")
        out.append("%+.4f%s" % (c, mono))
    return "F(x,y) = " + " ".join(out)


def load():
    with open(SRC, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    a = np.array([float(r["a_us"]) for r in rows])
    b = np.array([float(r["b_us"]) for r in rows])
    t = np.array([float(r["thrust_N"]) for r in rows])
    s = np.array([float(r["thrust_sem"]) for r in rows])
    return a, b, t, s


def report(a, b, t, s):
    x, y = norm(a, b)

    print("=== degree comparison (121 points) ===")
    print("%4s %6s %9s %9s %10s %9s %9s"
          % ("deg", "nterm", "RMS_N", "max|r|", "R2", "betw.run", "with.run"))
    for deg in (1, 2, 3, 4, 5):
        coef, r, r2 = fit(x, y, t, deg)
        within = sum(float(np.sum((r[a == av] - r[a == av].mean()) ** 2))
                     for av in np.unique(a))
        tot = float(np.sum(r ** 2))
        print("%4d %6d %9.4f %9.4f %10.6f %8.0f%% %8.0f%%"
              % (deg, len(terms(deg)), np.sqrt((r ** 2).mean()),
                 np.abs(r).max(), r2, 100 * (tot - within) / tot,
                 100 * within / tot))
    print("\nmedian per-point SEM = %.3f N (within-step noise only)"
          % np.median(s))

    coef, resid, r2 = fit(x, y, t, DEGREE)
    print("\n=== cubic fit ===")
    print("x = (A_us - %.0f) / %.0f    y = (B_us - %.0f) / %.0f"
          % (CENTER, SCALE, CENTER, SCALE))
    print(formula(coef))
    print("\nR2 = %.6f   RMS residual = %.4f N   max |residual| = %.4f N"
          % (r2, np.sqrt((resid ** 2).mean()), np.abs(resid).max()))

    print("\nper-run (per-A) mean residual -- the dominant error term:")
    for av in np.unique(a):
        print("  A=%4d  %+.3f N" % (av, resid[a == av].mean()))
    within = sum(float(np.sum((resid[a == av] - resid[a == av].mean()) ** 2))
                 for av in np.unique(a))
    print("  -> removing these offsets drops RMS %.3f -> %.3f N"
          % (np.sqrt((resid ** 2).mean()), np.sqrt(within / len(resid))))
    return coef, resid, r2


def plot(a, b, t, coef, resid, r2):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(16, 5.6))

    # -- measured surface -------------------------------------------------- #
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    av, bv = np.unique(a), np.unique(b)
    grid = np.full((len(av), len(bv)), np.nan)
    for ai, aa in enumerate(av):
        for bi, bb in enumerate(bv):
            m = (a == aa) & (b == bb)
            if m.any():
                grid[ai, bi] = t[m][0]
    BB, AA = np.meshgrid(bv, av)
    ax.plot_surface(BB, AA, grid, cmap="magma", edgecolor="0.3", lw=0.25,
                    alpha=0.92, rstride=1, cstride=1)
    ax.scatter(b, a, t, s=7, c="k", depthshade=False)
    ax.set_title("measured\n121 step means, 4 s dwells", fontsize=10)

    # -- cubic surface ----------------------------------------------------- #
    ax2 = fig.add_subplot(1, 3, 2, projection="3d")
    bd = np.linspace(bv.min(), bv.max(), 60)
    ad = np.linspace(av.min(), av.max(), 60)
    BD, AD = np.meshgrid(bd, ad)
    xd, yd = norm(AD, BD)
    ZD = design(xd.ravel(), yd.ravel(), DEGREE) @ coef
    ZD = ZD.reshape(AD.shape)
    ax2.plot_surface(BD, AD, ZD, cmap="magma", edgecolor="none", alpha=0.9)
    ax2.scatter(b, a, t, s=7, c="k", depthshade=False)
    ax2.set_title("bivariate cubic\nR$^2$=%.5f, RMS %.3f N" % (r2,
                  np.sqrt((resid ** 2).mean())), fontsize=10)

    for axx in (ax, ax2):
        axx.set_xlabel("B  [$\\mu$s]", fontsize=9)
        axx.set_ylabel("A  [$\\mu$s]", fontsize=9)
        axx.set_zlabel("thrust  [N]", fontsize=9)
        axx.view_init(elev=24, azim=-128)
        axx.tick_params(labelsize=7.5)

    # -- residuals --------------------------------------------------------- #
    ax3 = fig.add_subplot(1, 3, 3)
    rg = np.full((len(av), len(bv)), np.nan)
    for ai, aa in enumerate(av):
        for bi, bb in enumerate(bv):
            m = (a == aa) & (b == bb)
            if m.any():
                rg[ai, bi] = resid[m][0]
    lim = np.abs(resid).max()
    im = ax3.imshow(rg, origin="lower", cmap="RdBu_r", vmin=-lim, vmax=lim,
                    extent=[bv.min() - 50, bv.max() + 50,
                            av.min() - 50, av.max() + 50], aspect="auto")
    ax3.set_xlabel("B  [$\\mu$s]")
    ax3.set_ylabel("A  [$\\mu$s]")
    ax3.set_title("cubic residual\nbanded by row = per-run offset, not curvature",
                  fontsize=10)
    fig.colorbar(im, ax=ax3, label="measured - cubic  [N]")

    fig.suptitle("Coaxial thrust over the (A, B) PWM plane -- "
                 "raw/pwm 2026-07-31, no voltage correction", y=1.02)
    fig.tight_layout()

    out = os.path.join(ROOT, "out", "pwm_grid_surface.png")
    fig.savefig(out, dpi=155, bbox_inches="tight")
    print("\nwrote %s" % out)


def main():
    a, b, t, s = load()
    coef, resid, r2 = report(a, b, t, s)

    out = os.path.join(ROOT, "out", "pwm_cubic_coeffs.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["i_exp_x_A", "j_exp_y_B", "coeff"])
        for c, (i, j) in zip(coef, terms(DEGREE)):
            w.writerow([i, j, "%.6f" % c])
    print("wrote %s" % out)

    plot(a, b, t, coef, resid, r2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
