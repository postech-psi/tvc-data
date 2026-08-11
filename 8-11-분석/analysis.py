"""
Coaxial 모터 추력/토크 분석 — 0단계(정상상태 추출) + 1단계((A,B) 맵)

전처리 규약은 8-11-분석/전처리이유.md 를 그대로 구현한다.
- 2.2 phase 라벨로 세그먼트
- 2.3 앞 TRANSIENT_DROP_S 초(스핀업 과도구간) 제거
- 2.4 정상성(표류) 검사
- 2.5 추력 = -Fz, 토크 = Tz
- 2.6 SEM = s/sqrt(n)
- 2.7 자기상관 보정 SEM = s/sqrt(n_eff),  n_eff = n/tau

입력 : motor_raw/2026-07-31/A*_B1000-2000_*/loadcell.csv
출력 : 8-11-분석/out/steady_state.csv           (121점 대표값 표)
        8-11-분석/out/torque_zero_crossing.csv   (토크 상쇄 궤적)
        8-11-분석/out/*.png                       (발표용 그림)
"""
import csv
import glob
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- 경로/파라미터 -----------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "motor_raw", "2026-07-31")
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

TRANSIENT_DROP_S = 1.0     # 2.3 과도구간으로 버릴 앞부분 (초)
DRIFT_FLAG_N = 0.30        # 2.4 이 값(N)보다 큰 추력 표류는 flag
MIN_SAMPLES = 30           # 정착 창이 이보다 짧으면 flag

DWELL_RE = re.compile(r"^A(\d+)_B(\d+)$")   # dwell phase 라벨 형식


# ---- 기초 통계 (전처리이유.md 1부·2부 구현) ---------------------------------
def integrated_autocorr(x):
    """2.7: 적분자기상관시간 tau 와 유효표본수 n_eff.

    rho_k = sum_{i=1..n-k}(x_i-xbar)(x_{i+k}-xbar) / sum_{i=1..n}(x_i-xbar)^2
    tau   = 1 + 2 * sum_{k>=1}(1 - k/n) rho_k   (처음 rho_k<=0 에서 합산 종료)
    n_eff = n / tau
    """
    x = np.asarray(x, float)
    n = len(x)
    xc = x - x.mean()
    denom = np.dot(xc, xc)
    if n < 2 or denom == 0.0:
        return 1.0, float(n)          # 상수/한 점이면 보정 없음
    # rho_0..rho_{n-1}  (biased ACF, 분모는 전체 제곱합 → rho_0 = 1)
    acf = np.correlate(xc, xc, mode="full")[n - 1:] / denom
    s = 0.0
    for k in range(1, n):
        if acf[k] <= 0.0:             # 초기 양의 상관 구간까지만 합산
            break
        s += (1.0 - k / n) * acf[k]
    tau = 1.0 + 2.0 * s
    return tau, n / tau


def drift_total(t, x):
    """2.4: 최소제곱 직선의 기울기로 창 전체 표류량(단위: x의 단위)을 잰다.

    beta  = Cov(t,x)/Cov(t,t) = sum(t-tbar)(x-xbar) / sum(t-tbar)^2
    drift = |beta| * (t_max - t_min)   (창 양 끝 사이 직선이 변한 총량)
    """
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    tc = t - t.mean()
    Stt = np.dot(tc, tc)
    if Stt == 0.0:
        return 0.0, 0.0
    beta = np.dot(tc, x - x.mean()) / Stt
    return beta, abs(beta) * (t.max() - t.min())


def summarize(t, x):
    """정착 창의 한 신호(추력 또는 토크)에 대한 대표값 묶음."""
    x = np.asarray(x, float)
    n = len(x)
    mean = x.mean()
    s = x.std(ddof=1) if n > 1 else 0.0        # 1.3 표본표준편차 (n-1)
    tau, n_eff = integrated_autocorr(x)         # 2.7
    sem = s / np.sqrt(n) if n > 0 else np.nan   # 2.6 (독립 가정)
    sem_corr = s / np.sqrt(n_eff) if n_eff > 0 else np.nan  # 2.7 (보정)
    beta, drift = drift_total(t, x)             # 2.4
    return dict(n=n, mean=mean, s=s, tau=tau, n_eff=n_eff,
                sem=sem, sem_corr=sem_corr, drift=drift)


# ---- CSV 로더 ---------------------------------------------------------------
def load_loadcell(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r)
        idx = {name: i for i, name in enumerate(header)}
        phase, t, fz, tz, a, b = [], [], [], [], [], []
        for row in r:
            phase.append(row[idx["phase"]])
            t.append(float(row[idx["t_epoch"]]))
            fz.append(float(row[idx["Fz"]]))
            tz.append(float(row[idx["Tz"]]))
            a.append(int(row[idx["a_cmd_us"]]))
            b.append(int(row[idx["b_cmd_us"]]))
    return (np.array(phase), np.array(t), np.array(fz),
            np.array(tz), np.array(a), np.array(b))


# ---- 0단계: 정상상태 추출 ---------------------------------------------------
def stage0():
    runs = sorted(glob.glob(os.path.join(DATA, "A*_B1000-2000_*")))
    runs = [r for r in runs if os.path.isdir(r)]
    rows = []
    for run in runs:
        phase, t, fz, tz, a_us, b_us = load_loadcell(os.path.join(run, "loadcell.csv"))
        for lab in [p for p in dict.fromkeys(phase) if DWELL_RE.match(p)]:  # 2.2
            m = phase == lab
            tt = t[m]
            order = np.argsort(tt)                 # 시간순 정렬 보장
            tt = tt[order]
            thrust = -fz[m][order]                 # 2.5 추력 = -Fz
            torque = tz[m][order]                  # 2.5 토크 =  Tz
            t_rel = tt - tt[0]
            settled = t_rel >= TRANSIENT_DROP_S    # 2.3 과도구간 제거
            ts, F, T = t_rel[settled], thrust[settled], torque[settled]

            sF = summarize(ts, F)
            sT = summarize(ts, T)

            flags = []
            if sF["n"] < MIN_SAMPLES:
                flags.append("short")
            if sF["drift"] > DRIFT_FLAG_N:         # 2.4 판정
                flags.append("drift")

            rows.append(dict(
                a_us=int(a_us[m][0]), b_us=int(b_us[m][0]),
                n=sF["n"],
                thrust_N=sF["mean"], thrust_s=sF["s"],
                thrust_sem=sF["sem"], thrust_sem_corr=sF["sem_corr"],
                thrust_tau=sF["tau"], thrust_neff=sF["n_eff"], thrust_drift=sF["drift"],
                torque_Nm=sT["mean"], torque_s=sT["s"],
                torque_sem_corr=sT["sem_corr"], torque_tau=sT["tau"],
                flags="|".join(flags), run=os.path.basename(run),
            ))
    rows.sort(key=lambda d: (d["a_us"], d["b_us"]))

    cols = ["a_us", "b_us", "n", "thrust_N", "thrust_s", "thrust_sem",
            "thrust_sem_corr", "thrust_tau", "thrust_neff", "thrust_drift",
            "torque_Nm", "torque_s", "torque_sem_corr", "torque_tau",
            "flags", "run"]
    with open(os.path.join(OUT, "steady_state.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in cols})
    return rows


# ---- 1단계: (A,B) 맵 + 그림 -------------------------------------------------
def to_grid(rows, key):
    A = sorted({r["a_us"] for r in rows})
    B = sorted({r["b_us"] for r in rows})
    M = np.full((len(A), len(B)), np.nan)
    look = {(r["a_us"], r["b_us"]): r for r in rows}
    for i, a in enumerate(A):
        for j, b in enumerate(B):
            if (a, b) in look:
                M[i, j] = look[(a, b)][key]
    return np.array(A), np.array(B), M


def torque_zero_crossing(rows):
    """A별로 토크 Tz=0 이 되는 B를 인접점 선형보간으로 찾는다 (반동토크 상쇄)."""
    A, B, Tq = to_grid(rows, "torque_Nm")
    out = []
    for i, a in enumerate(A):
        y = Tq[i]
        for j in range(len(B) - 1):
            y0, y1 = y[j], y[j + 1]
            if np.isnan(y0) or np.isnan(y1):
                continue
            if y0 == 0.0:
                out.append((a, float(B[j])))
            elif y0 * y1 < 0.0:               # 부호가 바뀌면 그 사이에서 0을 지남
                b0, b1 = B[j], B[j + 1]
                b_zero = b0 + (b1 - b0) * (0.0 - y0) / (y1 - y0)
                out.append((a, float(b_zero)))
    with open(os.path.join(OUT, "torque_zero_crossing.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["a_us", "b_zero_us"])
        w.writerows(out)
    return out


# ---- 그림 -------------------------------------------------------------------
plt.rcParams.update({"figure.dpi": 130, "font.size": 11,
                     "axes.grid": True, "grid.alpha": 0.3})


def _colors(n):
    return plt.cm.viridis(np.linspace(0, 0.92, n))


def plot_family(rows, key, sem_key, ylabel, title, fname, zero_line=False):
    A, B, M = to_grid(rows, key)
    _, _, S = to_grid(rows, sem_key) if sem_key else (None, None, None)
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    for i, a in enumerate(A):
        c = _colors(len(A))[i]
        if S is not None:
            ax.errorbar(B, M[i], yerr=S[i], color=c, lw=1.6, marker="o", ms=3,
                        capsize=2, label=f"A={a}")
        else:
            ax.plot(B, M[i], color=c, lw=1.6, marker="o", ms=3, label=f"A={a}")
    if zero_line:
        ax.axhline(0, color="0.4", lw=1, ls="--")
    ax.set_xlabel("Command B [µs]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(ncol=2, fontsize=8, title="rotor A hold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


def plot_heatmap(rows, key, title, fname, cmap, zero_contour=False):
    A, B, M = to_grid(rows, key)
    fig, ax = plt.subplots(figsize=(6.6, 5.2))
    im = ax.imshow(M, origin="lower", aspect="auto", cmap=cmap,
                   extent=[B.min() - 50, B.max() + 50, A.min() - 50, A.max() + 50])
    if zero_contour:
        cs = ax.contour(B, A, M, levels=[0.0], colors="white", linewidths=2)
        ax.clabel(cs, fmt="Tz=0", fontsize=9)
    fig.colorbar(im, ax=ax, label=title)
    ax.set_xlabel("Command B [µs]")
    ax.set_ylabel("Command A [µs]")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


def plot_zero_crossing(rows, zc):
    A, B, Tq = to_grid(rows, "torque_Nm")
    fig, ax = plt.subplots(figsize=(6.6, 5.2))
    im = ax.imshow(Tq, origin="lower", aspect="auto", cmap="RdBu_r",
                   vmin=-np.nanmax(np.abs(Tq)), vmax=np.nanmax(np.abs(Tq)),
                   extent=[B.min() - 50, B.max() + 50, A.min() - 50, A.max() + 50])
    if zc:
        za, zb = zip(*zc)
        ax.plot(zb, za, "k-o", lw=2, ms=4, label="Tz = 0 (torque balance)")
        ax.legend(loc="upper right", fontsize=9)
    fig.colorbar(im, ax=ax, label="Torque Tz [N·m]")
    ax.set_xlabel("Command B [µs]")
    ax.set_ylabel("Command A [µs]")
    ax.set_title("Reaction torque over (A, B) and its zero locus")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "torque_balance.png"))
    plt.close(fig)


def plot_neff(rows):
    """발표용: 자기상관 보정이 왜 필요한지 — n vs n_eff, SEM 비교."""
    neff = np.array([r["thrust_neff"] for r in rows])
    tau = np.array([r["thrust_tau"] for r in rows])
    sem = np.array([r["thrust_sem"] for r in rows])
    semc = np.array([r["thrust_sem_corr"] for r in rows])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    a1.hist(tau, bins=20, color="#4C78A8")
    a1.set_xlabel(r"$\tau$ (integrated autocorr. time)")
    a1.set_ylabel("operating points")
    a1.set_title(f"Autocorrelation: mean τ = {tau.mean():.1f}  →  n_eff ≈ n/{tau.mean():.1f}")
    a2.scatter(sem, semc, s=18, color="#E45756")
    lim = max(sem.max(), semc.max()) * 1.05
    a2.plot([0, lim], [0, lim], "0.5", ls="--", lw=1)
    a2.set_xlim(0, lim); a2.set_ylim(0, lim)
    a2.set_xlabel("naive SEM  s/√n  [N]")
    a2.set_ylabel("corrected SEM  s/√n_eff  [N]")
    a2.set_title(f"Correction inflates thrust error bar ×{np.nanmean(semc/sem):.1f}")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "sem_correction.png"))
    plt.close(fig)


# ---- 메인 -------------------------------------------------------------------
def main():
    rows = stage0()
    zc = torque_zero_crossing(rows)

    plot_family(rows, "thrust_N", "thrust_sem_corr", "Thrust [N]",
                "Thrust vs command B (one curve per rotor-A hold)",
                "thrust_vs_b.png")
    plot_family(rows, "torque_Nm", "torque_sem_corr", "Torque Tz [N·m]",
                "Reaction torque vs command B (one curve per rotor-A hold)",
                "torque_vs_b.png", zero_line=True)
    plot_heatmap(rows, "thrust_N", "Thrust [N]", "thrust_map.png", "viridis")
    plot_heatmap(rows, "torque_Nm", "Torque Tz [N·m]", "torque_map.png",
                 "RdBu_r", zero_contour=True)
    plot_zero_crossing(rows, zc)
    plot_neff(rows)

    # 콘솔 요약
    flagged = [r for r in rows if r["flags"]]
    taus = np.array([r["thrust_tau"] for r in rows])
    print(f"runs processed : {len({r['run'] for r in rows})}")
    print(f"operating pts  : {len(rows)}  (expected 121)")
    print(f"flagged pts    : {len(flagged)}  -> {[(r['a_us'], r['b_us'], r['flags']) for r in flagged]}")
    print(f"thrust τ (autocorr time): mean {taus.mean():.2f}, "
          f"range {taus.min():.2f}–{taus.max():.2f}  (n_eff = n/τ)")
    print(f"torque Tz=0 locus points: {len(zc)}")
    print(f"outputs written to: {OUT}")


if __name__ == "__main__":
    main()
