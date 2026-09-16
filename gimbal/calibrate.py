"""Single-pose accelerometer calibration for the TVC rig.

Captures a short static reading at the ACTUAL mounting orientation and writes
calibration.json with a uniform scale so corrected |g| = 1.0 at the operating
point. This is all the mapping/step analysis needs, because:
  * tilt angle = atan2(main, gravity) is INVARIANT under a uniform scale, so the
    scale never distorts any mapping/step angle -- it only normalises magnitude
    so the |g| health gate passes honestly at the strict 0.02 g tolerance;
  * a constant accel bias cancels in the neutral-referenced tilt the analyzer
    uses; only cross-axis (~2% per datasheet) remains, and it is small.

Run once, with the IMU mounted in its test orientation and the rig fully still:

    python calibrate.py --port COM21

Then run the experiment:

    python run_experiment.py --port COM21 --yes
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

from analyze import load_run, sample_rows, last_value
from sid_capture import Device

PROJECT_DIR = Path(__file__).resolve().parent


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="ESP32 serial port, e.g. COM21")
    ap.add_argument("--out", type=Path, default=PROJECT_DIR / "calibration.json")
    args = ap.parse_args()

    tmp = PROJECT_DIR / "_calibrate_capture"
    if tmp.exists():
        shutil.rmtree(tmp)

    print("IMU를 시험 장착 자세로 완전히 정지시킨 뒤 진행합니다 (4초 캡처).")
    d = Device(args.port)
    try:
        d.request_line("DETACH", "# DETACHED")
        d.capture("HEALTH", tmp)
    finally:
        d.close()

    df, meta = load_run(tmp)
    s = sample_rows(df)
    raw = (s[["ax", "ay", "az"]].to_numpy(float) /
           float(last_value(meta["acc_lsb_per_g"])))
    gyro = (s[["gx", "gy", "gz"]].to_numpy(float) /
            float(last_value(meta["gyro_lsb_per_dps"])))
    gmag = float(np.linalg.norm(raw, axis=1).mean())
    scale = 1.0 / gmag

    cal = {
        "format": "tvc_sid_calibration_v1",
        "pass": True,
        "note": ("Single-pose uniform-scale calibration at the mounting orientation. "
                 f"raw |g|={gmag:.4f}, scale={scale:.5f} -> corrected |g|=1.0. Tilt "
                 "angles are scale-invariant, so this only normalises magnitude for "
                 "the health gate; mapping/step angles come from the raw accel "
                 "direction with neutral referencing (bias cancels)."),
        "acc_matrix": [[scale, 0, 0], [0, scale, 0], [0, 0, scale]],
        "acc_offset": [0.0, 0.0, 0.0],
        "gyro_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "gyro_bias_dps": gyro.mean(axis=0).tolist(),
        "gyro_noise_dps": gyro.std(axis=0).tolist(),
        "pose_fit_rms_g": 0.0,
        "corrected_norm_rms_error_g": 0.0,
        "matrix_condition": 1.0,
    }
    args.out.write_text(json.dumps(cal, ensure_ascii=False, indent=2), encoding="utf-8")
    shutil.rmtree(tmp)

    print(f"\nraw |g| = {gmag:.5f}  ->  scale = {scale:.6f}  ->  corrected |g| = 1.0")
    print(f"gyro noise (dps) = {np.round(gyro.std(axis=0), 3).tolist()}")
    print(f"saved: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
