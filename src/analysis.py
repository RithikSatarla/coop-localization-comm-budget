"""Aggregation helpers: sweep summaries, knee detection and markdown tables."""
from __future__ import annotations

import math

import numpy as np
import pandas as pd


def knee_point(x, y, min_rel_drop: float = 0.05):
    """Locate the knee of a decreasing accuracy-vs-cost curve (Kneedle-style).

    ``x`` is the communication cost (messages), ``y`` the error. Points are
    sorted by ``x``; the lower envelope (running minimum of ``y``) is taken so
    that non-monotone scatter does not create spurious knees. Both axes are
    normalised to [0, 1] and the knee is the envelope point with the largest
    vertical distance *below* the chord joining the first and last points.

    Returns ``(index_into_original_arrays, x_knee, y_knee)`` or ``None`` when
    fewer than three distinct points exist, when the curve is not
    convex-decreasing anywhere (no point lies below the chord), or when the
    total drop of the envelope is smaller than ``min_rel_drop`` of its starting
    value (a flat curve has no knee).
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    order = np.argsort(x, kind="mergesort")
    xs, ys = x[order], y[order]
    env = np.minimum.accumulate(ys)
    keep = np.r_[True, np.diff(xs) > 0] & (ys <= env + 1e-12)
    idx = order[keep]
    xs, ys = xs[keep], ys[keep]
    if len(xs) < 3 or xs[-1] <= xs[0] or ys[0] <= ys[-1]:
        return None
    if ys[0] <= 0 or (ys[0] - ys.min()) / ys[0] < min_rel_drop:
        return None
    xn = (xs - xs[0]) / (xs[-1] - xs[0])
    yn = (ys - ys[-1]) / (ys[0] - ys[-1])
    chord = 1.0 - xn
    dist = chord - yn
    i = int(np.argmax(dist))
    if dist[i] <= 0:
        return None
    return int(idx[i]), float(xs[i]), float(ys[i])


def messages_per_robot_minute(messages_total: float, n_robots: int, duration_s: float) -> float:
    return 60.0 * messages_total / (n_robots * duration_s)


def summarize_sweep(df: pd.DataFrame) -> pd.DataFrame:
    """Collapse per-robot sweep rows to one row per regime/family/setting/dataset.

    Mean RMSE over robots and seeds; messages delivered per robot per minute.
    """
    g = df.groupby(["regime", "family", "setting", "value", "dataset"], sort=False)
    out = g.agg(
        rmse_xy=("rmse_xy", "mean"),
        rmse_xy_blind=("rmse_xy", lambda s: float(s[df.loc[s.index, "blind"]].mean())
                       if df.loc[s.index, "blind"].any() else float("nan")),
        rmse_theta=("rmse_theta", "mean"),
        messages_total=("messages_sent_total", "mean"),
        candidates_total=("candidates_total", "mean"),
        fused_total=("fused_total", "mean"),
        duration_s=("duration_s", "first"),
        n_robots=("robot", "nunique"),
        n_seeds=("seed", "nunique"),
    ).reset_index()
    out["messages_per_robot_min"] = [
        messages_per_robot_minute(m, n, d)
        for m, n, d in zip(out["messages_total"], out["n_robots"], out["duration_s"])
    ]
    out["message_fraction"] = out["messages_total"] / out["candidates_total"].replace(0, np.nan)
    return out


def summarize_over_datasets(summary: pd.DataFrame) -> pd.DataFrame:
    """Average the per-dataset sweep summary over datasets (one row per setting)."""
    g = summary.groupby(["regime", "family", "setting", "value"], sort=False)
    out = g.agg(
        rmse_xy=("rmse_xy", "mean"),
        rmse_xy_std_datasets=("rmse_xy", "std"),
        rmse_xy_blind=("rmse_xy_blind", "mean"),
        rmse_theta=("rmse_theta", "mean"),
        messages_total=("messages_total", "mean"),
        messages_per_robot_min=("messages_per_robot_min", "mean"),
        message_fraction=("message_fraction", "mean"),
        n_datasets=("dataset", "nunique"),
    ).reset_index()
    return out


def _fmt(v, digits=3):
    if isinstance(v, float):
        if math.isnan(v):
            return ""
        if math.isinf(v):
            return "inf"
        if abs(v) >= 1000:
            return f"{v:.0f}"
        if v != 0 and abs(v) < 1e-3:
            return f"{v:.3g}"
        return f"{v:.{digits}f}".rstrip("0").rstrip(".") if digits > 0 else f"{v:.0f}"
    return str(v)


def to_markdown(df: pd.DataFrame, digits: int = 3, index: bool = False) -> str:
    """Render a DataFrame as a GitHub-flavoured markdown table (no external deps)."""
    if index:
        df = df.reset_index()
    cols = [str(c) for c in df.columns]
    lines = ["| " + " | ".join(cols) + " |", "|" + "|".join(["---"] * len(cols)) + "|"]
    for row in df.itertuples(index=False):
        lines.append("| " + " | ".join(_fmt(v, digits) for v in row) + " |")
    return "\n".join(lines)
