"""Figure generation. Every figure is written as both PDF and PNG and the PNG is
re-opened with Pillow to make sure it is a valid image."""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.lines  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from PIL import Image  # noqa: E402

# Fixed categorical palette (assigned by entity, never cycled).
C_BLUE, C_ORANGE, C_AQUA, C_YELLOW, C_MAGENTA, C_VIOLET = (
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7")
C_INK, C_MUTED, C_GRID = "#0b0b0b", "#898781", "#e1e0d9"

METHOD_COLORS = {
    "Dead reckoning": C_ORANGE,
    "EKF landmarks": C_BLUE,
    "EKF cooperative": C_AQUA,
}
FAMILY_STYLE = {
    "drop": dict(color=C_BLUE, marker="o", label="message drop probability"),
    "radius": dict(color=C_ORANGE, marker="s", label="comm radius"),
    "rate": dict(color=C_AQUA, marker="^", label="max update rate"),
    "trigger": dict(color=C_VIOLET, marker="D", label="covariance-threshold trigger"),
}
REGIME_TITLES = {
    "full_map": "Full map: all robots observe landmarks",
    "map_blind": "Map-blind team: only robots 1-2 observe landmarks",
}

plt.rcParams.update({
    "font.size": 9,
    "axes.titlesize": 9.5,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "axes.edgecolor": C_MUTED,
    "axes.labelcolor": C_INK,
    "xtick.color": C_INK,
    "ytick.color": C_INK,
    "axes.grid": True,
    "grid.color": C_GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.dpi": 100,
    "savefig.dpi": 200,
    "pdf.fonttype": 42,
})


def save_figure(fig, stem: str, figdir: Path) -> dict:
    figdir = Path(figdir)
    figdir.mkdir(parents=True, exist_ok=True)
    pdf = figdir / f"{stem}.pdf"
    png = figdir / f"{stem}.png"
    fig.savefig(pdf, bbox_inches="tight")
    fig.savefig(png, bbox_inches="tight")
    plt.close(fig)
    with Image.open(png) as im:
        im.verify()
    with Image.open(png) as im:
        w, h = im.size
    if w < 100 or h < 100:
        raise RuntimeError(f"{png} is unexpectedly small ({w}x{h})")
    return {"pdf": str(pdf), "png": str(png), "size": (w, h)}


def _style_axes(ax):
    ax.grid(True, which="major", axis="y")
    ax.grid(False, axis="x")


def _plain_log_ticks(ax, axis="y"):
    """Log axis with plain decimal tick labels (0.2, 0.5, 1, 2 ...) instead of powers."""
    fmt = matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}")
    a = ax.yaxis if axis == "y" else ax.xaxis
    a.set_major_formatter(fmt)
    a.set_minor_formatter(fmt)
    a.set_minor_locator(matplotlib.ticker.LogLocator(base=10, subs=(2.0, 5.0), numticks=12))


def _choose_scale(ax, values, log_ratio=5.0):
    """Log scale when the spread is large; otherwise linear with the baseline at zero so
    that small differences are not visually exaggerated."""
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v) & (v > 0)]
    if v.size == 0:
        return
    if v.max() / v.min() > log_ratio:
        ax.set_yscale("log")
        _plain_log_ticks(ax)
    else:
        ax.set_ylim(0, 1.18 * v.max())


def _annotate_grouped(ax, xs, ys, labels, xtol=0.03, ytol=0.015, show=None):
    """Annotate points, merging labels of points that coincide (within tolerances).

    ``show`` optionally restricts annotation to labels for which ``show(label)``
    is true; the other points stay unlabeled (they remain in the legend by marker).
    """
    xs = np.asarray(xs, float)
    ys = np.asarray(ys, float)
    keep = np.array([show(l) if show else True for l in labels], bool)
    used = ~keep
    xspan = max(xs.max() - xs.min(), 1e-9)
    n_drawn = 0
    for i in np.argsort(xs):
        if used[i]:
            continue
        close = (~used) & (np.abs(np.log1p(xs) - np.log1p(xs[i])) <= xtol * np.log1p(xspan)) \
            & (np.abs(ys - ys[i]) <= ytol * max(abs(ys[i]), 1e-9) + 1e-12)
        used |= close
        text = ", ".join(labels[j] for j in np.nonzero(close)[0])
        # Alternate above/below the marker so neighbouring labels do not collide.
        offset = (5, 4) if n_drawn % 2 == 0 else (5, -10)
        ax.annotate(text, (xs[i], ys[i]), xytext=offset, textcoords="offset points",
                    fontsize=6.5, color=C_MUTED)
        n_drawn += 1


# Labels drawn on the pooled accuracy-vs-messages figure. Every rate and trigger
# setting is labeled (they are the matched-budget comparison); for the drop and
# radius families only the settings discussed in the text are labeled to keep
# the figure legible. All values are in the tables.
def _label_shown(label: str) -> bool:
    if label.startswith("tau=") or label.endswith(" s") or label == "inf":
        return True
    return label in {"p=1", "p=0.9", "p=0.5", "2 m", "4 m"}


def plot_trajectory(t, gt, gt_valid, estimates: dict, landmarks: dict, title: str,
                    stem: str, figdir: Path) -> dict:
    """Ground truth versus estimated trajectories for one robot, plus error over time."""
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.0),
                                  gridspec_kw={"width_ratios": [1.1, 1.0]})
    lm = np.asarray(list(landmarks.values()) if isinstance(landmarks, dict) else landmarks,
                    dtype=float)
    ax.scatter(lm[:, 0], lm[:, 1], marker="^", s=28, color=C_MUTED, zorder=2,
               label="landmarks", linewidths=0)
    ax.plot(gt[gt_valid, 0], gt[gt_valid, 1], color=C_INK, lw=1.4, label="ground truth", zorder=3)
    for name, traj in estimates.items():
        ax.plot(traj[:, 0], traj[:, 1], color=METHOD_COLORS[name], lw=0.9, alpha=0.9,
                label=name, zorder=4)
    ax.plot(gt[0, 0], gt[0, 1], marker="o", color=C_INK, ms=5, zorder=5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.grid(True)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False)

    for name, traj in estimates.items():
        err = np.hypot(traj[:, 0] - gt[:, 0], traj[:, 1] - gt[:, 1])
        err = np.where(gt_valid, err, np.nan)
        ax2.plot(t, err, color=METHOD_COLORS[name], lw=0.9, label=name)
    ax2.set_yscale("log")
    _plain_log_ticks(ax2)
    ax2.set_ylim(bottom=0.01)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("position error [m]")
    ax2.set_title("Position error over time (log scale)")
    ax2.legend(frameon=False, loc="lower right")
    fig.tight_layout()
    return save_figure(fig, stem, figdir)


def plot_base_bars(df: pd.DataFrame, stem: str, figdir: Path) -> dict:
    """Mean RMSE per dataset for the three base methods; per-robot values as dots.

    ``df`` has one row per (regime, dataset, method, robot) with ``rmse_xy``.
    """
    regimes = [r for r in REGIME_TITLES if r in set(df["regime"])]
    fig, axes = plt.subplots(1, len(regimes), figsize=(4.8 * len(regimes), 3.6), squeeze=False)
    methods = list(METHOD_COLORS)
    width = 0.26
    for ax, regime in zip(axes[0], regimes):
        sub = df[df["regime"] == regime]
        datasets = sorted(sub["dataset"].unique())
        x = np.arange(len(datasets))
        for mi, method in enumerate(methods):
            means = [sub[(sub.dataset == d) & (sub.method == method)]["rmse_xy"].mean()
                     for d in datasets]
            xpos = x + (mi - 1) * width
            ax.bar(xpos, means, width=width * 0.92, color=METHOD_COLORS[method],
                   label=method, linewidth=0, zorder=2)
            for d, xp in zip(datasets, xpos):
                vals = sub[(sub.dataset == d) & (sub.method == method)]["rmse_xy"].to_numpy()
                jitter = (np.arange(len(vals)) - (len(vals) - 1) / 2) * 0.03
                ax.scatter(np.full(len(vals), xp) + jitter, vals, s=9, color=C_INK,
                           alpha=0.7, zorder=3, linewidths=0)
        ax.set_yscale("log")
        _plain_log_ticks(ax)
        ax.set_ylim(bottom=0.05)
        ax.set_xticks(x)
        ax.set_xticklabels([f"dataset {d}" for d in datasets])
        ax.set_ylabel("position RMSE [m]")
        ax.set_title(REGIME_TITLES[regime])
        _style_axes(ax)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, -0.14), title="bar = mean over the 5 robots, dots = individual robots")
    fig.tight_layout()
    return save_figure(fig, stem, figdir)


def plot_sweep(summary: pd.DataFrame, family: str, xlabel: str, stem: str, figdir: Path,
               baseline: pd.DataFrame | None = None) -> dict:
    """RMSE versus one communication constraint: faint per-dataset lines, bold mean.

    ``summary`` is the per-dataset sweep summary (one row per regime/setting/dataset)
    filtered to ``family``; settings keep their sweep order. ``baseline`` holds the
    landmark-only EKF mean RMSE per regime/dataset for the dashed reference line.
    """
    regimes = [r for r in REGIME_TITLES if r in set(summary["regime"])]
    fig, axes = plt.subplots(1, len(regimes), figsize=(4.8 * len(regimes), 3.6), squeeze=False)
    style = FAMILY_STYLE[family]
    for ax, regime in zip(axes[0], regimes):
        sub = summary[summary["regime"] == regime]
        settings = list(dict.fromkeys(sub["setting"]))
        x = np.arange(len(settings))
        datasets = sorted(sub["dataset"].unique())
        mat = np.full((len(datasets), len(settings)), np.nan)
        for i, d in enumerate(datasets):
            for j, s in enumerate(settings):
                v = sub[(sub.dataset == d) & (sub.setting == s)]["rmse_xy"]
                if len(v):
                    mat[i, j] = float(v.iloc[0])
            ax.plot(x, mat[i], color=style["color"], alpha=0.3, lw=0.9, marker=style["marker"],
                    ms=3.5, zorder=2)
            ax.annotate(f"D{d}", (x[-1], mat[i, -1]), xytext=(4, 0), textcoords="offset points",
                        fontsize=7, color=C_MUTED, va="center")
        mean = np.nanmean(mat, axis=0)
        ax.plot(x, mean, color=style["color"], lw=2.2, marker=style["marker"], ms=6,
                label="mean over datasets", zorder=4)
        if baseline is not None:
            b = baseline[baseline["regime"] == regime]["rmse_xy"].mean()
            ax.axhline(b, color=C_MUTED, ls="--", lw=1.0, label="EKF landmarks only (no messages)")
        ax.set_xticks(x)
        tick_labels = [s.replace("tau=", "") for s in settings] if family == "trigger" else settings
        ax.set_xticklabels(tick_labels, rotation=(30 if len(settings) > 6 else 0),
                           ha=("right" if len(settings) > 6 else "center"))
        ax.set_xlabel(xlabel)
        ax.set_ylabel("team position RMSE [m]")
        ax.set_title(REGIME_TITLES[regime])
        _choose_scale(ax, mat[np.isfinite(mat)])
        _style_axes(ax)
    handles, labels = axes[0][0].get_legend_handles_labels()
    handles.append(matplotlib.lines.Line2D([], [], color=style["color"], alpha=0.3, lw=0.9,
                                           marker=style["marker"], ms=3.5))
    labels.append("individual datasets (D1-D4)")
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, -0.05))
    fig.tight_layout()
    return save_figure(fig, stem, figdir)


def plot_accuracy_vs_messages(points: pd.DataFrame, knees: dict, stem: str, figdir: Path) -> dict:
    """Combined accuracy-versus-messages curve pooling all three constraint families.

    ``points`` has one row per (regime, family, setting) averaged over datasets with
    ``messages_per_robot_min`` and ``rmse_xy``. ``knees`` maps regime to
    ``(x, y)`` of the knee (or None).
    """
    regimes = [r for r in REGIME_TITLES if r in set(points["regime"])]
    fig, axes = plt.subplots(1, len(regimes), figsize=(4.8 * len(regimes), 3.7), squeeze=False)
    for ax, regime in zip(axes[0], regimes):
        sub = points[points["regime"] == regime].sort_values("messages_per_robot_min")
        xs = sub["messages_per_robot_min"].to_numpy()
        ys = sub["rmse_xy"].to_numpy()
        env = np.minimum.accumulate(ys)
        ax.plot(xs, env, color=C_MUTED, lw=1.2, zorder=1, label="lower envelope")
        for fam, st in FAMILY_STYLE.items():
            f = sub[sub["family"] == fam]
            ax.scatter(f["messages_per_robot_min"], f["rmse_xy"], color=st["color"],
                       marker=st["marker"], s=34, zorder=3, label=st["label"], linewidths=0)
        _annotate_grouped(ax, xs, ys, [str(s) for s in sub["setting"]], show=_label_shown)
        knee = knees.get(regime)
        shared_handles, shared_labels = ax.get_legend_handles_labels()
        if knee is not None:
            kx, ky = knee
            h = ax.scatter([kx], [ky], marker="*", s=200, color=C_INK, zorder=5, linewidths=0)
            ax.legend([h], [f"knee: {kx:.1f} msg/robot/min, {ky:.2f} m"], frameon=False,
                      loc="upper right")
        else:
            ax.legend([matplotlib.lines.Line2D([], [], ls="none")],
                      ["no knee: curve is flat within noise"], frameon=False, loc="upper right",
                      handlelength=0)
        ax.set_xscale("symlog", linthresh=1.0)
        ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        _choose_scale(ax, ys)
        ax.set_xlabel("messages delivered per robot per minute (symlog)")
        ax.set_ylabel("team position RMSE [m]")
        ax.set_title(REGIME_TITLES[regime])
        _style_axes(ax)
    fig.legend(shared_handles, shared_labels, loc="lower center", ncol=4, frameon=False,
               bbox_to_anchor=(0.5, -0.06))
    fig.tight_layout()
    return save_figure(fig, stem, figdir)
