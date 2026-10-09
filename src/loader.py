"""Load UTIAS MR.CLAM datasets into pandas DataFrames and resample onto a uniform grid.

Raw file layout (verified against the official dataset page,
https://asrl.utias.utoronto.ca/datasets/mrclam/index.html):

  Barcodes.dat              subject #, barcode #
  Landmark_Groundtruth.dat  subject #, x [m], y [m], x std-dev [m], y std-dev [m]
  Robot<N>_Groundtruth.dat  time [s], x [m], y [m], orientation [rad]
  Robot<N>_Odometry.dat     time [s], forward velocity [m/s], angular velocity [rad/s]
  Robot<N>_Measurement.dat  time [s], barcode # of measured subject, range [m], bearing [rad]

Robots are subjects 1-5 and landmarks are subjects 6-20. The measurement file
records the *barcode* number, which must be mapped back to a subject through
Barcodes.dat; the mapping differs between datasets. A handful of measurements
carry barcodes that are not in Barcodes.dat (mis-detections); they are dropped.

Resampling conventions
----------------------
* The uniform grid runs from the latest first-groundtruth time to the earliest
  last-groundtruth time across the five robots, in steps of ``dt`` seconds.
* Odometry commands are zero-order held onto the grid. If the most recent
  command is older than ``odom_hold_max`` seconds (the logs contain occasional
  one-second gaps and, in dataset 3, robot 4's odometry stops 288 s early), the
  velocity is set to zero rather than holding a stale command.
* Groundtruth is linearly interpolated (heading is unwrapped first). Grid
  points that fall inside a groundtruth dropout window, defined as a gap
  longer than ``gt_gap_max`` seconds between consecutive Vicon samples, are
  marked invalid and excluded from scoring. The number of masked seconds per
  robot is recorded in ``RobotData.gt_dropout_seconds``.
* Measurements are assigned to the nearest grid step.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

ROBOT_IDS = (1, 2, 3, 4, 5)
LANDMARK_IDS = tuple(range(6, 21))
ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def dataset_folder(k: int, data_dir: Path = DATA_DIR) -> Path:
    """Locate the extracted folder for dataset ``k`` (handles the MRSLAM_Dataset4 quirk)."""
    for name in (f"MRCLAM_Dataset{k}", f"MRSLAM_Dataset{k}"):
        p = Path(data_dir) / name
        if p.is_dir():
            return p
    raise FileNotFoundError(
        f"dataset {k} not found under {data_dir}; run scripts/download_mrclam.py"
    )


def read_dat(path: Path, columns: list[str]) -> pd.DataFrame:
    """Read a whitespace-delimited .dat file with '#' comment lines."""
    return pd.read_csv(path, sep=r"\s+", comment="#", header=None, names=columns,
                       engine="python")


@dataclass
class RawRobot:
    robot_id: int
    odometry: pd.DataFrame      # time, v, w
    measurements: pd.DataFrame  # time, barcode, subject, kind, range, bearing
    groundtruth: pd.DataFrame   # time, x, y, theta


@dataclass
class RawDataset:
    dataset_id: int
    folder: Path
    barcodes: dict               # subject -> barcode
    barcode_to_subject: dict     # barcode -> subject
    landmarks: pd.DataFrame      # subject, x, y, x_std, y_std
    robots: dict                 # robot_id -> RawRobot
    n_unknown_barcode: int       # measurements dropped because the barcode is unknown


def load_raw(k: int, data_dir: Path = DATA_DIR) -> RawDataset:
    folder = dataset_folder(k, data_dir)
    bc = read_dat(folder / "Barcodes.dat", ["subject", "barcode"]).astype(int)
    barcodes = dict(zip(bc["subject"], bc["barcode"]))
    b2s = {b: s for s, b in barcodes.items()}
    landmarks = read_dat(folder / "Landmark_Groundtruth.dat",
                         ["subject", "x", "y", "x_std", "y_std"])
    landmarks["subject"] = landmarks["subject"].astype(int)
    robots = {}
    n_unknown = 0
    for r in ROBOT_IDS:
        od = read_dat(folder / f"Robot{r}_Odometry.dat", ["time", "v", "w"])
        ms = read_dat(folder / f"Robot{r}_Measurement.dat",
                      ["time", "barcode", "range", "bearing"])
        gt = read_dat(folder / f"Robot{r}_Groundtruth.dat", ["time", "x", "y", "theta"])
        ms["barcode"] = ms["barcode"].astype(int)
        ms["subject"] = ms["barcode"].map(b2s).fillna(-1).astype(int)
        kind = np.where(ms["subject"].between(1, 5), "robot",
                        np.where(ms["subject"].between(6, 20), "landmark", "unknown"))
        ms["kind"] = kind
        n_unknown += int((ms["kind"] == "unknown").sum())
        ms = ms[ms["kind"] != "unknown"].reset_index(drop=True)
        # A robot never observes itself; such rows would be barcode mis-reads.
        ms = ms[ms["subject"] != r].reset_index(drop=True)
        for df in (od, ms, gt):
            df.sort_values("time", inplace=True, kind="mergesort")
            df.reset_index(drop=True, inplace=True)
        robots[r] = RawRobot(r, od, ms, gt)
    return RawDataset(k, folder, barcodes, b2s, landmarks, robots, n_unknown)


@dataclass
class RobotData:
    robot_id: int
    v: np.ndarray           # (N,) forward velocity on the grid [m/s]
    w: np.ndarray           # (N,) angular velocity on the grid [rad/s]
    odom_valid: np.ndarray  # (N,) False where no recent odometry command exists
    gt: np.ndarray          # (N,3) interpolated groundtruth x, y, theta
    gt_valid: np.ndarray    # (N,) False inside groundtruth dropout windows
    meas: pd.DataFrame      # step, time, subject, kind, range, bearing
    gt_dropout_seconds: float
    gt_dropout_windows: list = field(default_factory=list)  # (t_start, t_end) relative


@dataclass
class Dataset:
    dataset_id: int
    dt: float
    t: np.ndarray            # (N,) grid time relative to t0 [s]
    t0: float                # absolute start time [s]
    landmarks: dict          # subject -> np.array([x, y])
    landmark_std: dict       # subject -> np.array([x_std, y_std])
    robots: dict             # robot_id -> RobotData
    n_unknown_barcode: int

    @property
    def n_steps(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0] + self.dt)


def _zero_order_hold(src_t, src_val, grid_t, hold_max):
    idx = np.searchsorted(src_t, grid_t, side="right") - 1
    valid = idx >= 0
    idx_c = np.clip(idx, 0, len(src_t) - 1)
    age = grid_t - src_t[idx_c]
    valid &= age <= hold_max
    out = np.where(valid, src_val[idx_c], 0.0)
    return out, valid


def _gt_dropout_windows(gt_t, gap_max):
    gaps = np.diff(gt_t)
    idx = np.nonzero(gaps > gap_max)[0]
    return [(gt_t[i], gt_t[i + 1]) for i in idx]


def resample(raw: RawDataset, dt: float = 0.02, gt_gap_max: float = 0.5,
             odom_hold_max: float = 1.5, max_duration: float | None = None) -> Dataset:
    """Resample a raw dataset onto a uniform time grid (see module docstring).

    ``max_duration`` truncates the grid to the first ``max_duration`` seconds
    after the common start (used for the 500 s window of dataset 9).
    """
    t_start = max(rr.groundtruth["time"].iloc[0] for rr in raw.robots.values())
    t_end = min(rr.groundtruth["time"].iloc[-1] for rr in raw.robots.values())
    if max_duration is not None:
        t_end = min(t_end, t_start + float(max_duration))
    n = int(np.floor((t_end - t_start) / dt)) + 1
    t_abs = t_start + dt * np.arange(n)
    t_rel = dt * np.arange(n)

    landmarks = {int(s): np.array([x, y]) for s, x, y in
                 raw.landmarks[["subject", "x", "y"]].itertuples(index=False)}
    landmark_std = {int(s): np.array([sx, sy]) for s, sx, sy in
                    raw.landmarks[["subject", "x_std", "y_std"]].itertuples(index=False)}

    robots = {}
    for r, rr in raw.robots.items():
        od_t = rr.odometry["time"].to_numpy()
        v, odom_valid = _zero_order_hold(od_t, rr.odometry["v"].to_numpy(), t_abs, odom_hold_max)
        w, _ = _zero_order_hold(od_t, rr.odometry["w"].to_numpy(), t_abs, odom_hold_max)

        gt_t = rr.groundtruth["time"].to_numpy()
        gx = np.interp(t_abs, gt_t, rr.groundtruth["x"].to_numpy())
        gy = np.interp(t_abs, gt_t, rr.groundtruth["y"].to_numpy())
        gth = wrap_angle(np.interp(t_abs, gt_t, np.unwrap(rr.groundtruth["theta"].to_numpy())))
        gt = np.column_stack([gx, gy, gth])

        gt_valid = (t_abs >= gt_t[0]) & (t_abs <= gt_t[-1])
        windows = _gt_dropout_windows(gt_t, gt_gap_max)
        for a, b in windows:
            gt_valid &= ~((t_abs > a) & (t_abs < b))
        dropout_seconds = float((~gt_valid).sum() * dt)

        ms = rr.measurements.copy()
        step = np.rint((ms["time"].to_numpy() - t_start) / dt).astype(int)
        keep = (step >= 0) & (step < n)
        ms = ms.loc[keep].copy()
        ms["step"] = step[keep]
        ms = ms[["step", "time", "subject", "kind", "range", "bearing"]]
        ms.sort_values(["step", "subject"], inplace=True, kind="mergesort")
        ms.reset_index(drop=True, inplace=True)

        robots[r] = RobotData(
            robot_id=r, v=v, w=w, odom_valid=odom_valid, gt=gt, gt_valid=gt_valid,
            meas=ms, gt_dropout_seconds=dropout_seconds,
            gt_dropout_windows=[(a - t_start, b - t_start) for a, b in windows],
        )

    return Dataset(raw.dataset_id, dt, t_rel, t_start, landmarks, landmark_std,
                   robots, raw.n_unknown_barcode)


def load_dataset(k: int, dt: float = 0.02, data_dir: Path = DATA_DIR, **kw) -> Dataset:
    """Convenience wrapper: load raw files for dataset ``k`` and resample them."""
    return resample(load_raw(k, data_dir), dt=dt, **kw)


def summarize(ds: Dataset) -> pd.DataFrame:
    """Per-robot summary table used in the docs (counts, dropout, measurement mix)."""
    rows = []
    for r, rd in ds.robots.items():
        n_lm = int((rd.meas["kind"] == "landmark").sum())
        n_rob = int((rd.meas["kind"] == "robot").sum())
        rr = rd.meas.loc[rd.meas["kind"] == "robot", "range"]
        rows.append(dict(
            dataset=ds.dataset_id, robot=r, duration_s=round(ds.duration, 1),
            landmark_obs=n_lm, robot_obs=n_rob,
            robot_obs_per_min=round(60.0 * n_rob / ds.duration, 2),
            robot_range_median_m=round(float(rr.median()), 2) if len(rr) else np.nan,
            robot_range_max_m=round(float(rr.max()), 2) if len(rr) else np.nan,
            gt_dropout_s=round(rd.gt_dropout_seconds, 1),
            odom_missing_s=round(float((~rd.odom_valid).sum() * ds.dt), 1),
        ))
    return pd.DataFrame(rows)
