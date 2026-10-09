"""Diagnostic for the dataset-9 (first 500 s) failure of the fixed-parameter EKF.

On datasets 1-4 the landmark EKF reaches 0.08-0.26 m RMSE per robot, but on the
first 500 s of dataset 9 it drifts to several metres for three of the five
robots. This script isolates the cause by running the landmark-only EKF per
robot with the protocol's innovation gate (chi-square 9.21) and with the gate
disabled, counting accepted and rejected landmark updates in each case. It
also reports the peak commanded speeds, which are higher in dataset 9 than in
datasets 1-4.

The protocol itself is not changed: the sweep results in results/dataset9_500s
were produced with the same parameters as datasets 1-4, and this script only
documents why they are not comparable to datasets 1-4 or to published
dataset-9 numbers.

Usage:
    python scripts/dataset9_gate_check.py     # -> results/dataset9_500s/gate_diagnosis.csv
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.loader import load_dataset  # noqa: E402
from src.localize import EKF, NoiseParams, _meas_by_step, score  # noqa: E402

MAX_DURATION = 500.0
GATES = {"protocol gate (9.21)": 9.21, "gate disabled": float("inf")}


def run_landmark_ekf(ds, r, gate):
    rd = ds.robots[r]
    params = NoiseParams(gate_chi2=gate)
    ekf = EKF(rd.gt[0], params)
    meas = _meas_by_step(rd, kind="landmark")
    n = len(rd.v)
    traj = np.empty((n, 3))
    traj[0] = ekf.x
    accepted = rejected = 0
    for k in range(1, n):
        ekf.predict(rd.v[k - 1], rd.w[k - 1], ds.dt)
        for subject, rr, bb in meas.get(k, ()):
            if ekf.update_range_bearing(rr, bb, ds.landmarks[subject]):
                accepted += 1
            else:
                rejected += 1
        traj[k] = ekf.x
    s = score(traj, rd)
    return dict(rmse_xy=s["rmse_xy"], median_xy=s["median_xy"], max_xy=s["max_xy"],
                landmark_updates_accepted=accepted, landmark_updates_rejected=rejected)


def main():
    rows = []
    for k in (9, 1):
        ds = load_dataset(k, max_duration=MAX_DURATION)
        for r, rd in ds.robots.items():
            for name, gate in GATES.items():
                res = run_landmark_ekf(ds, r, gate)
                rows.append(dict(dataset=k, window_s=MAX_DURATION, robot=r, gate=name,
                                 v_max_mps=float(np.abs(rd.v).max()),
                                 w_max_radps=float(np.abs(rd.w).max()), **res))
                print(f"dataset {k} robot {r} {name}: RMSE {res['rmse_xy']:.3f} m, "
                      f"rejected {res['landmark_updates_rejected']} of "
                      f"{res['landmark_updates_accepted'] + res['landmark_updates_rejected']}",
                      flush=True)
    df = pd.DataFrame(rows)
    out = ROOT / "results" / "dataset9_500s" / "gate_diagnosis.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"written {out}")


if __name__ == "__main__":
    main()
