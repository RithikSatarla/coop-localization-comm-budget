"""Run every experiment and produce results/*.csv, figures/*.{pdf,png} and results/tables.md.

Protocol
--------
For each dataset (1-4 by default) and each regime:

  full_map   every robot fuses landmark measurements (the standard MR.CLAM
             "cooperative localization with a known map" task);
  map_blind  only robots 1 and 2 fuse landmark measurements; robots 3-5 have
             odometry and teammate messages only (a heterogeneous team).

1. Base methods: dead reckoning, EKF with landmarks, cooperative EKF with
   unconstrained communication. A fusion ablation (naive inflation instead of
   covariance intersection) is run alongside.
2. Communication sweeps on the cooperative EKF: message drop probability,
   comm radius on measured range, maximum teammate-update rate, and a simple
   covariance-threshold trigger. Drop probabilities are repeated over several
   seeds; the other three families are deterministic and run once.
3. Message accounting: candidates (robot-to-robot observations), messages
   delivered, fused, gated and declined are logged for every run.

Usage
-----
    python run_experiments.py                      # datasets 1-4, 5 seeds, 4 workers
    python run_experiments.py --datasets 1 2       # subset
    python run_experiments.py --quick              # dataset 1, 1 seed (smoke test)
    python run_experiments.py --replot             # regenerate figures/tables from results/
    python run_experiments.py --datasets 9 --max-duration 500 \
        --results results/dataset9_500s --figures figures/dataset9_500s
"""
from __future__ import annotations

import argparse
import json
import math
import platform
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from src import plotting  # noqa: E402
from src.analysis import (knee_point, matched_budget, summarize_over_datasets,  # noqa: E402
                          summarize_sweep, to_markdown)
from src.comm import (COMM_RADII, DROP_PROBS, RATE_INTERVALS, TRIGGER_TAUS, DropPolicy,  # noqa: E402
                      EventTriggeredPolicy, FullComm, NoComm, RadiusPolicy, RatePolicy,
                      setting_label)
from src.loader import load_dataset, summarize  # noqa: E402
from src.localize import (NoiseParams, dead_reckoning, ekf_cooperative, ekf_landmarks,  # noqa: E402
                          score)

REGIMES = {"full_map": None, "map_blind": (1, 2)}   # regime -> robots that may use landmarks
METHODS = ("Dead reckoning", "EKF landmarks", "EKF cooperative")
FAMILIES = ("drop", "radius", "rate", "trigger")
FAMILY_TITLES = {"drop": "Message drop probability", "radius": "Comm radius",
                 "rate": "Max update rate (minimum interval between teammate updates)",
                 "trigger": "Covariance-threshold trigger (tau, m^2)"}
FAMILY_XLABELS = {"drop": "message drop probability p", "radius": "comm radius (measured range)",
                  "rate": "minimum interval between teammate updates",
                  "trigger": "trigger threshold tau on position-covariance trace [m$^2$]"}
PLOT_DATASET = 1
PLOT_ROBOTS = {"full_map": 1, "map_blind": 3}
TRAJ_STRIDE = 10            # trajectories are stored every 10th grid step (0.2 s) for plotting


def _policy(family: str, value: float):
    if family == "drop":
        return DropPolicy(value)
    if family == "radius":
        return RadiusPolicy(value)
    if family == "rate":
        return RatePolicy(value)
    if family == "trigger":
        return EventTriggeredPolicy(value)
    raise ValueError(family)


def _family_settings(family: str):
    return {"drop": DROP_PROBS, "radius": COMM_RADII, "rate": RATE_INTERVALS,
            "trigger": TRIGGER_TAUS}[family]


def _rows(ds, regime, method, trajs, log, extra=None):
    lm_robots = REGIMES[regime]
    out = []
    for r in sorted(ds.robots):
        row = dict(dataset=ds.dataset_id, regime=regime, method=method, robot=r,
                   blind=bool(lm_robots is not None and r not in lm_robots))
        row.update(score(trajs[r], ds.robots[r]))
        row.update(dict(
            messages_sent_total=log.sent if log else 0,
            candidates_total=log.candidates if log else 0,
            fused_total=log.fused if log else 0,
            gated_total=log.gated if log else 0,
            declined_total=log.declined if log else 0,
            messages_sent_robot=log.per_robot_sent[r] if log else 0,
            candidates_robot=log.per_robot_candidates[r] if log else 0,
            duration_s=ds.duration,
        ))
        if extra:
            row.update(extra)
        out.append(row)
    return out


def run_dataset(k: int, dt: float, seeds: list[int], max_duration: float | None = None) -> dict:
    """All runs for one dataset. Executed in a worker process."""
    t_start = time.time()
    ds = load_dataset(k, dt=dt, max_duration=max_duration)
    params = NoiseParams()
    base, sweep, ablation = [], [], []
    traj_store = {}
    n_runs = 0

    def log_line(msg):
        print(f"[dataset {k}] {msg}", flush=True)

    for regime, lm_robots in REGIMES.items():
        lm_set = None if lm_robots is None else set(lm_robots)
        dr = {r: dead_reckoning(ds.robots[r], dt) for r in ds.robots}
        base += _rows(ds, regime, "Dead reckoning", dr, None)
        # Landmark-only EKF through the joint code path with no messages; identical to
        # the per-robot ekf_landmarks (asserted below for the full-map regime).
        ekfl, log_none = ekf_cooperative(ds, NoComm(), params, landmark_robots=lm_set)
        n_runs += 1
        if lm_set is None:
            ref = ekf_landmarks(ds.robots[1], ds.landmarks, dt, params)
            if not np.allclose(ref, ekfl[1], atol=1e-9):
                raise RuntimeError("joint no-comm EKF differs from per-robot landmark EKF")
        base += _rows(ds, regime, "EKF landmarks", ekfl, log_none)
        coop, log_full = ekf_cooperative(ds, FullComm(), params, landmark_robots=lm_set)
        n_runs += 1
        base += _rows(ds, regime, "EKF cooperative", coop, log_full)
        log_line(f"{regime}: base methods done "
                 f"(candidates {log_full.candidates}, fused {log_full.fused})")

        naive, log_naive = ekf_cooperative(ds, FullComm(), params, landmark_robots=lm_set,
                                           fusion="naive")
        n_runs += 1
        ablation += _rows(ds, regime, "EKF cooperative (naive)", naive, log_naive)
        ablation += _rows(ds, regime, "EKF cooperative (CI)", coop, log_full)

        if k == PLOT_DATASET:
            r = PLOT_ROBOTS[regime]
            s = slice(None, None, TRAJ_STRIDE)
            estimates = {"Dead reckoning": dr[r][s], "EKF cooperative": coop[r][s]}
            if lm_set is None or r in lm_set:
                estimates = {"Dead reckoning": dr[r][s], "EKF landmarks": ekfl[r][s],
                             "EKF cooperative": coop[r][s]}
            traj_store[regime] = dict(
                robot=r, t=ds.t[s], gt=ds.robots[r].gt[s], gt_valid=ds.robots[r].gt_valid[s],
                estimates=estimates,
                landmarks=np.array([ds.landmarks[i] for i in sorted(ds.landmarks)]),
            )

        for family in FAMILIES:
            fam_seeds = seeds if family == "drop" else [0]
            for value in _family_settings(family):
                pol = _policy(family, value)
                label = setting_label(pol)
                for seed in fam_seeds:
                    trajs, log = ekf_cooperative(ds, pol, params, seed=seed, landmark_robots=lm_set)
                    n_runs += 1
                    sweep += _rows(ds, regime, "EKF cooperative", trajs, log,
                                   extra=dict(family=family, setting=label,
                                              value=(math.inf if math.isinf(value) else value),
                                              seed=seed))
                team = np.mean([row["rmse_xy"] for row in sweep
                                if row["regime"] == regime and row["family"] == family
                                and row["setting"] == label])
                log_line(f"{regime}: {family} {label}: delivered {log.sent}/{log.candidates}, "
                         f"team RMSE {team:.3f} m")

    elapsed = time.time() - t_start
    log_line(f"finished {n_runs} filter runs in {elapsed / 60:.1f} min")
    return dict(dataset=k, base=base, sweep=sweep, ablation=ablation,
                summary=summarize(ds), traj=traj_store, n_runs=n_runs, elapsed_s=elapsed,
                n_steps=ds.n_steps, duration_s=ds.duration,
                n_unknown_barcode=ds.n_unknown_barcode)


# ----------------------------------------------------------------------------
# Raw result files
# ----------------------------------------------------------------------------

def save_raw(results: list[dict], resdir: Path):
    resdir.mkdir(parents=True, exist_ok=True)
    base = pd.DataFrame([r for res in results for r in res["base"]])
    sweep = pd.DataFrame([r for res in results for r in res["sweep"]])
    ablation = pd.DataFrame([r for res in results for r in res["ablation"]])
    dsum = pd.concat([res["summary"] for res in results], ignore_index=True)
    base.to_csv(resdir / "base_methods.csv", index=False)
    ablation.to_csv(resdir / "fusion_ablation.csv", index=False)
    dsum.to_csv(resdir / "dataset_summary.csv", index=False)
    for fam in FAMILIES:
        sweep[sweep.family == fam].to_csv(resdir / f"sweep_{fam}.csv", index=False)
    traj = {}
    for res in results:
        for regime, ts in res["traj"].items():
            p = f"{regime}/"
            traj[p + "robot"] = np.array(ts["robot"])
            traj[p + "t"] = ts["t"].astype(np.float32)
            traj[p + "gt"] = ts["gt"].astype(np.float32)
            traj[p + "gt_valid"] = ts["gt_valid"]
            traj[p + "landmarks"] = ts["landmarks"].astype(np.float32)
            for name, arr in ts["estimates"].items():
                traj[p + "est/" + name] = arr.astype(np.float32)
    if traj:
        np.savez_compressed(resdir / f"trajectories_d{PLOT_DATASET}.npz", **traj)
    return base, sweep, ablation, dsum


def load_raw(resdir: Path):
    base = pd.read_csv(resdir / "base_methods.csv")
    ablation = pd.read_csv(resdir / "fusion_ablation.csv")
    dsum = pd.read_csv(resdir / "dataset_summary.csv")
    parts = [pd.read_csv(resdir / f"sweep_{fam}.csv") for fam in FAMILIES
             if (resdir / f"sweep_{fam}.csv").exists()]
    sweep = pd.concat(parts, ignore_index=True)
    return base, sweep, ablation, dsum


def load_trajectories(resdir: Path) -> dict:
    path = resdir / f"trajectories_d{PLOT_DATASET}.npz"
    if not path.exists():
        return {}
    out = {}
    with np.load(path) as z:
        for key in z.files:
            regime, rest = key.split("/", 1)
            ts = out.setdefault(regime, {"estimates": {}})
            if rest.startswith("est/"):
                ts["estimates"][rest[4:]] = z[key].astype(float)
            elif rest == "robot":
                ts["robot"] = int(z[key])
            else:
                ts[rest] = z[key]
    return out


# ----------------------------------------------------------------------------
# Derived outputs: summaries, knees, figures, tables
# ----------------------------------------------------------------------------

def _knee_rows(pts: pd.DataFrame, regime: str, scope: str):
    """Knee of the accuracy-vs-messages points ``pts`` (one scope = pooled or one family)."""
    res = knee_point(pts["messages_per_robot_min"].to_numpy(), pts["rmse_xy"].to_numpy())
    full_pts = pts[pts["messages_per_robot_min"] == pts["messages_per_robot_min"].max()]
    none_pts = pts[pts["messages_per_robot_min"] == 0]
    common = dict(
        scope=scope, regime=regime,
        rmse_xy_full_comm=float(full_pts["rmse_xy"].iloc[0]) if len(full_pts) else float("nan"),
        rmse_xy_no_comm=float(none_pts["rmse_xy"].iloc[0]) if len(none_pts) else float("nan"),
        messages_per_robot_min_full_comm=float(full_pts["messages_per_robot_min"].iloc[0])
        if len(full_pts) else float("nan"),
    )
    if res is None:
        return None, dict(knee_found=False, **common)
    i, kx, ky = res
    row = pts.iloc[i]
    gain_total = common["rmse_xy_no_comm"] - common["rmse_xy_full_comm"]
    return (kx, ky), dict(
        knee_found=True, family=row["family"], setting=row["setting"],
        messages_per_robot_min=kx, rmse_xy=ky, message_fraction=float(row["message_fraction"]),
        fraction_of_gain=(common["rmse_xy_no_comm"] - ky) / gain_total if gain_total > 0 else float("nan"),
        **common)


def write_outputs(base, sweep, ablation, dsum, traj_store, resdir: Path, figdir: Path):
    resdir.mkdir(parents=True, exist_ok=True)
    figdir.mkdir(parents=True, exist_ok=True)
    families_present = [f for f in FAMILIES if (sweep["family"] == f).any()]

    # --- base summary: mean over robots per dataset/regime/method --------------
    base_summary = (base.groupby(["regime", "dataset", "method"], sort=False)
                    .agg(rmse_xy=("rmse_xy", "mean"), rmse_xy_max_robot=("rmse_xy", "max"),
                         median_xy=("median_xy", "mean"), rmse_theta=("rmse_theta", "mean"),
                         messages=("messages_sent_total", "first"),
                         candidates=("candidates_total", "first"))
                    .reset_index())
    blind_only = (base[base.blind].groupby(["regime", "dataset", "method"], sort=False)
                  .agg(rmse_xy_blind=("rmse_xy", "mean")).reset_index())
    base_summary = base_summary.merge(blind_only, how="left", on=["regime", "dataset", "method"])
    base_summary.to_csv(resdir / "base_methods_summary.csv", index=False)

    # --- sweep summaries -------------------------------------------------------
    sweep_summary = summarize_sweep(sweep)
    sweep_summary.to_csv(resdir / "sweeps_summary_per_dataset.csv", index=False)
    sweep_mean = summarize_over_datasets(sweep_summary)
    sweep_mean.to_csv(resdir / "sweeps_summary.csv", index=False)

    # --- accuracy vs messages: pooled knee per regime, plus per-family knees -----
    knees = {}
    knee_rows = []
    for regime in REGIMES:
        pts = sweep_mean[sweep_mean.regime == regime].reset_index(drop=True)
        if pts.empty:
            continue
        knee, row = _knee_rows(pts, regime, "pooled")
        knees[regime] = knee
        knee_rows.append(row)
        for fam in families_present:
            fpts = pts[pts.family == fam].reset_index(drop=True)
            # a family without a zero-message setting borrows the no-comm point
            if not (fpts["messages_per_robot_min"] == 0).any():
                none_pt = pts[pts["messages_per_robot_min"] == 0].head(1)
                fpts = pd.concat([fpts, none_pt], ignore_index=True)
            _, frow = _knee_rows(fpts, regime, fam)
            knee_rows.append(frow)
    knee_df = pd.DataFrame(knee_rows)
    knee_df.to_csv(resdir / "knee_points.csv", index=False)
    acc = sweep_mean[["regime", "family", "setting", "value", "messages_per_robot_min",
                      "message_fraction", "rmse_xy", "rmse_xy_blind", "rmse_xy_std_datasets"]]
    acc.to_csv(resdir / "accuracy_vs_messages.csv", index=False)
    matched = matched_budget(sweep_mean) if "trigger" in families_present else pd.DataFrame()
    if not matched.empty:
        matched.to_csv(resdir / "matched_budget.csv", index=False)

    # --- figures ---------------------------------------------------------------
    figs = {}
    for regime, ts in traj_store.items():
        r = ts["robot"]
        if regime == "full_map":
            title = f"Dataset {PLOT_DATASET}, robot {r}: full map"
        else:
            title = f"Dataset {PLOT_DATASET}, robot {r}: map-blind (no landmark access)"
        figs[f"trajectory_{regime}"] = plotting.plot_trajectory(
            ts["t"], ts["gt"], ts["gt_valid"], ts["estimates"], ts["landmarks"], title,
            f"trajectory_d{PLOT_DATASET}_r{r}_{regime}", figdir)
    figs["rmse_base_methods"] = plotting.plot_base_bars(base, "rmse_base_methods", figdir)
    baseline = base_summary[base_summary.method == "EKF landmarks"]
    for fam in families_present:
        figs[f"rmse_vs_{fam}"] = plotting.plot_sweep(
            sweep_summary[sweep_summary.family == fam], fam, FAMILY_XLABELS[fam],
            f"rmse_vs_{fam}", figdir, baseline=baseline)
    figs["accuracy_vs_messages"] = plotting.plot_accuracy_vs_messages(
        sweep_mean, knees, "accuracy_vs_messages", figdir)

    # --- markdown tables -------------------------------------------------------
    md = ["## Dataset summary\n", to_markdown(dsum, digits=2)]
    md.append("\n## Base methods: team position RMSE [m], mean over the five robots\n")
    pivot = base_summary.pivot_table(index=["regime", "dataset"], columns="method",
                                     values="rmse_xy", sort=False)[list(METHODS)].reset_index()
    md.append(to_markdown(pivot, digits=3))
    md.append("\n## Base methods: per-robot position RMSE [m]\n")
    per_robot = base.pivot_table(index=["regime", "dataset", "robot", "blind"], columns="method",
                                 values="rmse_xy", sort=False)[list(METHODS)].reset_index()
    md.append(to_markdown(per_robot, digits=3))
    md.append("\n## Base methods: heading RMSE [rad], mean over robots\n")
    pivot_th = base_summary.pivot_table(index=["regime", "dataset"], columns="method",
                                        values="rmse_theta", sort=False)[list(METHODS)].reset_index()
    md.append(to_markdown(pivot_th, digits=3))
    md.append("\n## Fusion ablation: covariance intersection (CI) vs naive inflation, "
              "unconstrained communication, team position RMSE [m]\n")
    abl = (ablation.groupby(["regime", "dataset", "method"], sort=False)
           .agg(rmse_xy=("rmse_xy", "mean"), rmse_xy_max_robot=("rmse_xy", "max"),
                fused=("fused_total", "first"), gated=("gated_total", "first"),
                declined=("declined_total", "first"))
           .reset_index())
    md.append(to_markdown(abl, digits=3))
    for fam in families_present:
        md.append(f"\n## Sweep: {FAMILY_TITLES[fam]} (mean over datasets)\n")
        sub = sweep_mean[sweep_mean.family == fam][
            ["regime", "setting", "messages_per_robot_min", "message_fraction", "rmse_xy",
             "rmse_xy_std_datasets", "rmse_xy_blind", "rmse_theta"]]
        md.append(to_markdown(sub, digits=3))
        md.append("\nPer dataset (team position RMSE [m]):\n")
        pv = sweep_summary[sweep_summary.family == fam].pivot_table(
            index=["regime", "setting"], columns="dataset", values="rmse_xy", sort=False)
        pv.columns = [f"dataset {c}" for c in pv.columns]
        md.append(to_markdown(pv.reset_index(), digits=3))
    md.append("\n## Accuracy vs messages: knee points (pooled over families and per family)\n")
    md.append(to_markdown(knee_df, digits=3))
    if not matched.empty:
        md.append("\n## Matched-budget comparison: covariance-threshold trigger vs the nearest "
                  "delivered-message rate of each other family\n")
        md.append(to_markdown(matched, digits=3))
    (resdir / "tables.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    return base_summary, sweep_mean, knee_df, figs, knees


def write_metadata(results, args, wall_s, figs, knees, resdir: Path):
    meta = dict(
        datasets=[res["dataset"] for res in results],
        dt=args.dt, max_duration=args.max_duration, seeds=list(range(args.seeds)),
        drop_probs=list(DROP_PROBS),
        comm_radii=[("inf" if math.isinf(v) else v) for v in COMM_RADII],
        rate_intervals=[("inf" if math.isinf(v) else v) for v in RATE_INTERVALS],
        trigger_taus=list(TRIGGER_TAUS),
        regimes={k: (list(v) if v else "all") for k, v in REGIMES.items()},
        noise_params={k: (list(v) if isinstance(v, tuple) else v)
                      for k, v in asdict(NoiseParams()).items()},
        filter_runs=int(sum(res["n_runs"] for res in results)),
        per_dataset_elapsed_s={res["dataset"]: round(res["elapsed_s"], 1) for res in results},
        per_dataset_steps={res["dataset"]: res["n_steps"] for res in results},
        per_dataset_duration_s={res["dataset"]: round(res["duration_s"], 1) for res in results},
        unknown_barcode_measurements_dropped={res["dataset"]: res["n_unknown_barcode"]
                                              for res in results},
        wall_time_s=round(wall_s, 1), workers=args.workers,
        python=platform.python_version(), platform=platform.platform(),
        numpy=np.__version__, pandas=pd.__version__,
        figures={k: Path(v["png"]).name for k, v in figs.items()},
        knees={k: (list(v) if v else None) for k, v in knees.items()},
    )
    (resdir / "run_metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--dt", type=float, default=0.02, help="resampling step [s]")
    ap.add_argument("--max-duration", type=float, default=None,
                    help="use only the first N seconds of each dataset")
    ap.add_argument("--seeds", type=int, default=5, help="number of seeds for the drop sweep")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--results", type=Path, default=ROOT / "results")
    ap.add_argument("--figures", type=Path, default=ROOT / "figures")
    ap.add_argument("--quick", action="store_true", help="dataset 1 only, one seed")
    ap.add_argument("--replot", action="store_true",
                    help="regenerate summaries, figures and tables from existing results/")
    args = ap.parse_args(argv)

    if args.replot:
        base, sweep, ablation, dsum = load_raw(args.results)
        traj_store = load_trajectories(args.results)
        base_summary, sweep_mean, knee_df, figs, knees = write_outputs(
            base, sweep, ablation, dsum, traj_store, args.results, args.figures)
        print("Regenerated", len(figs), "figures and results/tables.md from", args.results)
        print(knee_df.to_string(index=False))
        return

    if args.quick:
        args.datasets, args.seeds = [1], 1
    seeds = list(range(args.seeds))
    t0 = time.time()
    print(f"datasets {args.datasets}, dt={args.dt}, max_duration={args.max_duration}, "
          f"seeds={seeds}, workers={args.workers}", flush=True)
    workers = max(1, min(args.workers, len(args.datasets)))
    if workers == 1:
        results = [run_dataset(k, args.dt, seeds, args.max_duration) for k in args.datasets]
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            futures = [ex.submit(run_dataset, k, args.dt, seeds, args.max_duration)
                       for k in args.datasets]
            results = [f.result() for f in futures]
    results.sort(key=lambda r: r["dataset"])
    wall = time.time() - t0

    base, sweep, ablation, dsum = save_raw(results, args.results)
    traj_store = {regime: ts for res in results for regime, ts in res["traj"].items()}
    base_summary, sweep_mean, knee_df, figs, knees = write_outputs(
        base, sweep, ablation, dsum, traj_store, args.results, args.figures)
    write_metadata(results, args, wall, figs, knees, args.results)

    print("\nBase methods (team position RMSE [m]):")
    print(base_summary.pivot_table(index=["regime", "dataset"], columns="method", values="rmse_xy",
                                   sort=False)[list(METHODS)].round(3).to_string())
    print("\nKnees:")
    print(knee_df.to_string(index=False))
    print(f"\nTotal wall time {wall / 60:.1f} min; outputs in {args.results} and {args.figures}")


if __name__ == "__main__":
    main()
