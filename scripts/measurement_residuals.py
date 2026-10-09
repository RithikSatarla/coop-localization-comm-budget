"""Diagnostic: range/bearing measurement residuals against groundtruth.

For every landmark and robot-to-robot observation the predicted range and
bearing are computed from the interpolated groundtruth poses (and the
landmark groundtruth positions) and subtracted from the measured values.
Robust statistics (median, MAD-based standard deviation) and the inlier
fraction are reported per dataset, per target kind and per range band.

This is a *diagnostic* of the sensor, used only to document how the a-priori
filter noise parameters in src/localize.py compare with the data. Groundtruth
is not used inside any estimator.

Usage:
    python scripts/measurement_residuals.py            # datasets 1-4 -> results/measurement_residuals.csv
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.loader import load_dataset, wrap_angle  # noqa: E402

RANGE_BANDS = ((0.0, 2.0), (2.0, 4.0), (4.0, 10.0))


def residuals(ds):
    rows = []
    for r, rd in ds.robots.items():
        for step, subject, kind, rng, brg in rd.meas[["step", "subject", "kind", "range",
                                                        "bearing"]].itertuples(index=False):
            if not rd.gt_valid[step]:
                continue
            gx, gy, gth = rd.gt[step]
            if kind == "landmark":
                tx, ty = ds.landmarks[int(subject)]
            else:
                other = ds.robots[int(subject)]
                if not other.gt_valid[step]:
                    continue
                tx, ty = other.gt[step, :2]
            d = math.hypot(tx - gx, ty - gy)
            rows.append((kind, rng, rng - d,
                         float(wrap_angle(brg - (math.atan2(ty - gy, tx - gx) - gth)))))
    return pd.DataFrame(rows, columns=["kind", "range", "range_resid", "bearing_resid"])


def robust_std(x):
    x = np.asarray(x)
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def summarize(df, dataset):
    out = []
    inlier = (df["range_resid"].abs() < 1.0) & (df["bearing_resid"].abs() < 0.3)
    for kind in ("landmark", "robot"):
        for lo, hi in (("all", "all"),) + RANGE_BANDS:
            sel = df["kind"] == kind
            if lo != "all":
                sel &= (df["range"] >= lo) & (df["range"] < hi)
            n = int(sel.sum())
            if n == 0:
                continue
            s = df[sel & inlier]
            out.append(dict(
                dataset=dataset, kind=kind,
                range_band=("all" if lo == "all" else f"{lo:g}-{hi:g} m"),
                n=n, inlier_fraction=round(float(inlier[sel].mean()), 4),
                range_bias_m=round(float(s["range_resid"].median()), 4),
                range_std_m=round(float(robust_std(s["range_resid"])), 4),
                bearing_bias_rad=round(float(s["bearing_resid"].median()), 5),
                bearing_std_rad=round(float(robust_std(s["bearing_resid"])), 5),
            ))
    return out


def main(argv):
    ids = [int(a) for a in argv[1:]] or [1, 2, 3, 4]
    rows = []
    for k in ids:
        ds = load_dataset(k)
        rows += summarize(residuals(ds), k)
        print(f"dataset {k} done", flush=True)
    df = pd.DataFrame(rows)
    out = ROOT / "results" / "measurement_residuals.csv"
    out.parent.mkdir(exist_ok=True)
    df.to_csv(out, index=False)
    print(df.to_string(index=False))
    print(f"\nwritten {out}")


if __name__ == "__main__":
    main(sys.argv)
