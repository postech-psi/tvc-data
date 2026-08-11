"""
The (A, B) -> thrust surface in 3D: measured data, and the cubic laid over it.

Everything here is three-axis. The left panel is the measured grid alone, the
right panel is the same 121 measured points with the fitted cubic surface drawn
through them, so the fit is judged against the data it came from rather than
side by side with it.

Model and coefficients are `pwm_grid_surface.py`'s -- imported, not duplicated:

    x = (A - 1500)/500,  y = (B - 1500)/500
    F(x,y) = sum c_ij x^i y^j  over i + j <= 3

No voltage correction, by request.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pwm_grid_surface import DEGREE, design, fit, formula, load, norm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    a, b, t, _s = load()
    x, y = norm(a, b)
    coef, resid, r2 = fit(x, y, t, DEGREE)
    rms = float(np.sqrt((resid ** 2).mean()))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    av, bv = np.unique(a), np.unique(b)
    grid = np.full((len(av), len(bv)), np.nan)
    for i, aa in enumerate(av):
        for j, bb in enumerate(bv):
            m = (a == aa) & (b == bb)
            if m.any():
                grid[i, j] = t[m][0]
    BB, AA = np.meshgrid(bv, av)

    # Dense evaluation of the fit, for the overlay surface.
    bd = np.linspace(bv.min(), bv.max(), 70)
    ad = np.linspace(av.min(), av.max(), 70)
    BD, AD = np.meshgrid(bd, ad)
    xd, yd = norm(AD, BD)
    ZD = (design(xd.ravel(), yd.ravel(), DEGREE) @ coef).reshape(AD.shape)

    fig = plt.figure(figsize=(15, 6.5))

    # -- measured only ----------------------------------------------------- #
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot_wireframe(BB, AA, grid, color="0.55", lw=0.7, alpha=0.9)
    sc = ax.scatter(b, a, t, c=t, cmap="magma", s=26, depthshade=False,
                    edgecolor="k", linewidth=0.3)
    ax.set_title("measured\n121 step means (4 s dwells)", fontsize=11)
    fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.10, label="thrust  [N]")

    # -- cubic drawn through the measured points --------------------------- #
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    ax2.plot_surface(BD, AD, ZD, cmap="magma", alpha=0.62, linewidth=0,
                     antialiased=True, rstride=1, cstride=1)
    # Stems make the residual visible in 3D: each measured point is joined to
    # the surface directly below/above it, so over- and under-shoot read at a
    # glance instead of hiding behind the surface.
    zfit = design(x, y, DEGREE) @ coef
    for bi, ai, ti, zi in zip(b, a, t, zfit):
        ax2.plot([bi, bi], [ai, ai], [zi, ti], color="0.25", lw=0.6, alpha=0.8)
    above = t >= zfit
    ax2.scatter(b[above], a[above], t[above], c="#c62828", s=20,
                depthshade=False, edgecolor="k", linewidth=0.3,
                label="measured above fit")
    ax2.scatter(b[~above], a[~above], t[~above], c="#1565c0", s=20,
                depthshade=False, edgecolor="k", linewidth=0.3,
                label="measured below fit")
    ax2.set_title("bivariate cubic mapped onto the measured points\n"
                  "R$^2$=%.5f, RMS %.3f N, max |resid| %.2f N"
                  % (r2, rms, np.abs(resid).max()), fontsize=11)
    ax2.legend(loc="upper left", fontsize=8, framealpha=0.9)

    for axx in (ax, ax2):
        axx.set_xlabel("rotor B command  [$\\mu$s]", fontsize=9.5, labelpad=8)
        axx.set_ylabel("rotor A command  [$\\mu$s]", fontsize=9.5, labelpad=8)
        axx.set_zlabel("thrust  [N]", fontsize=9.5, labelpad=6)
        axx.view_init(elev=22, azim=-131)
        axx.tick_params(labelsize=8)
        axx.set_zlim(min(0, t.min()) - 0.5, t.max() + 0.8)

    fig.suptitle("Coaxial thrust over the (A, B) PWM plane -- "
                 "raw/pwm 2026-07-31, no voltage correction", y=0.99,
                 fontsize=12.5)
    fig.tight_layout()

    out = os.path.join(ROOT, "out", "pwm_grid_surface3d.png")
    fig.savefig(out, dpi=155, bbox_inches="tight")
    print(formula(coef))
    print("R2 = %.6f   RMS = %.4f N" % (r2, rms))
    print("wrote %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
