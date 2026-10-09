"""Localization estimators for the MR.CLAM data.

Three estimators share one unicycle motion model and one range-bearing
measurement model:

(a) ``dead_reckoning``   integrates odometry only.
(b) ``ekf_landmarks``    EKF fusing odometry with range/bearing to landmarks
                         whose positions are known from Landmark_Groundtruth.dat.
(c) ``ekf_cooperative``  the same EKF, run jointly for all five robots, that
                         additionally fuses robot-to-robot range/bearing. When
                         robot i observes teammate j, j's *current estimate* is
                         used as the observed point and the measurement noise is
                         inflated by j's own position covariance (plus a fixed
                         margin), so a poorly localized teammate contributes
                         little. Cross-covariances between robots are not
                         tracked (the decoupled, "naive" cooperative EKF).

Ground truth enters only through the initial pose and through scoring. The
estimators never read ground truth after the first step.

State is x = [x, y, theta]. Odometry noise is specified as spectral densities
so results are insensitive to the resampling step ``dt``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .loader import Dataset, RobotData, wrap_angle

TWO_PI = 2.0 * math.pi


@dataclass
class NoiseParams:
    """Filter tuning. Values fixed a priori; identical for every dataset and method."""
    sigma_v: float = 0.05        # forward-velocity noise density [m/s/sqrt(Hz)]
    sigma_w: float = 0.10        # angular-velocity noise density [rad/s/sqrt(Hz)]
    sigma_theta_floor: float = 0.002  # heading random walk when stationary [rad/sqrt(s)]
    sigma_r: float = 0.15        # range noise [m]
    sigma_b: float = 0.05        # bearing noise [rad]
    gate_chi2: float = 9.21      # Mahalanobis gate, chi-square 2 dof, 99 %
    p0_xy: float = 0.05          # initial position std [m]
    p0_theta: float = 0.05       # initial heading std [rad]
    # Candidate omega values for split covariance intersection (teammate fusion).
    ci_omegas: tuple = tuple(np.round(np.arange(0.05, 1.0, 0.05), 2))


@dataclass
class CommLog:
    """Message accounting for one cooperative run."""
    candidates: int = 0   # robot-to-robot observations that could have triggered a message
    sent: int = 0         # messages actually delivered (passed the comm constraint)
    fused: int = 0        # delivered messages that updated the receiver's state
    gated: int = 0        # delivered messages rejected by the innovation gate
    declined: int = 0     # delivered messages carrying no usable information (CI)
    per_robot_sent: dict = field(default_factory=dict)      # receiver -> count
    per_robot_candidates: dict = field(default_factory=dict)


def _wrap(a: float) -> float:
    return (a + math.pi) % TWO_PI - math.pi


class EKF:
    """Three-state extended Kalman filter for one unicycle robot."""

    def __init__(self, x0: np.ndarray, params: NoiseParams):
        self.x = np.array(x0, dtype=float)
        self.P = np.diag([params.p0_xy ** 2, params.p0_xy ** 2, params.p0_theta ** 2])
        self.p = params
        self.R = np.diag([params.sigma_r ** 2, params.sigma_b ** 2])

    def predict(self, v: float, w: float, dt: float) -> None:
        x, y, th = self.x
        c, s = math.cos(th), math.sin(th)
        self.x[0] = x + v * c * dt
        self.x[1] = y + v * s * dt
        self.x[2] = _wrap(th + w * dt)
        # Jacobian of the motion model w.r.t. the state.
        F = np.array([[1.0, 0.0, -v * s * dt],
                      [0.0, 1.0, v * c * dt],
                      [0.0, 0.0, 1.0]])
        # Noise on (v, w) enters through G; densities are scaled by dt.
        qv = self.p.sigma_v ** 2 * dt
        qw = self.p.sigma_w ** 2 * dt
        Q = np.array([[qv * c * c, qv * c * s, 0.0],
                      [qv * c * s, qv * s * s, 0.0],
                      [0.0, 0.0, qw + self.p.sigma_theta_floor ** 2 * dt]])
        self.P = F @ self.P @ F.T + Q

    def _linearize(self, r: float, b: float, point: np.ndarray):
        x, y, th = self.x
        dx = point[0] - x
        dy = point[1] - y
        q = dx * dx + dy * dy
        if q < 1e-6:
            return None
        rng = math.sqrt(q)
        nu = np.array([r - rng, _wrap(b - _wrap(math.atan2(dy, dx) - th))])
        H = np.array([[-dx / rng, -dy / rng, 0.0],
                      [dy / q, -dx / q, -1.0]])
        return nu, H

    def _apply(self, nu, H, P_prior, R_eff) -> bool:
        """Gated Kalman update of the state with prior covariance ``P_prior``."""
        S = H @ P_prior @ H.T + R_eff
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        d2 = float(nu @ S_inv @ nu)
        if d2 > self.p.gate_chi2:
            return False
        K = P_prior @ H.T @ S_inv
        self.x = self.x + K @ nu
        self.x[2] = _wrap(self.x[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ P_prior @ I_KH.T + K @ R_eff @ K.T   # Joseph form
        return True

    def update_range_bearing(self, r: float, b: float, point: np.ndarray) -> bool:
        """Fuse a range/bearing measurement to a point with exactly known position.

        Returns True if the measurement passed the innovation gate.
        """
        lin = self._linearize(r, b, point)
        if lin is None:
            return False
        nu, H = lin
        return self._apply(nu, H, self.P, self.R)

    def update_teammate(self, r: float, b: float, point: np.ndarray,
                        point_cov: np.ndarray, fusion: str = "ci") -> bool:
        """Fuse a range/bearing measurement to a teammate whose position is estimated.

        ``point`` and ``point_cov`` are the teammate's current position estimate
        and its 2x2 covariance. Because the two robots' errors may be
        correlated through earlier exchanges and the filter does not track
        cross-covariances, the teammate information is inflated:

        * ``fusion="ci"``  split covariance intersection (Julier & Uhlmann 2001;
          used for cooperative localization by Carrillo-Arce et al. 2013). The
          own prior is scaled by 1/omega and the teammate covariance by
          1/(1-omega); the independent sensor noise R is left unscaled. omega
          is chosen on a grid to minimize the posterior trace. The result is
          consistent for any unknown cross-correlation.
        * ``fusion="naive"``  the teammate covariance is simply added to R
          (no inflation); known to be overconfident. Kept as an ablation.

        Returns one of ``"fused"`` (state updated), ``"gated"`` (rejected by the
        innovation gate) or ``"declined"`` (CI only: no omega reduces the
        posterior trace below the prior trace, so the message carries no usable
        information and the state is left unchanged).
        """
        lin = self._linearize(r, b, point)
        if lin is None:
            return "gated"
        nu, H = lin
        Hp = -H[:, :2]                       # d h / d point = - d h / d (x, y)
        Rj = Hp @ point_cov @ Hp.T           # teammate uncertainty seen through h
        if fusion == "naive":
            return "fused" if self._apply(nu, H, self.P, self.R + Rj) else "gated"
        if fusion != "ci":
            raise ValueError(f"unknown fusion mode {fusion!r}")
        prior_trace = self.P[0, 0] + self.P[1, 1] + self.P[2, 2]
        best = None
        for omega in self.p.ci_omegas:
            P_w = self.P / omega
            R_w = self.R + Rj / (1.0 - omega)
            S = H @ P_w @ H.T + R_w
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            K = P_w @ H.T @ S_inv
            P_new = (np.eye(3) - K @ H) @ P_w
            cost = P_new[0, 0] + P_new[1, 1] + P_new[2, 2]
            if best is None or cost < best[0]:
                best = (cost, P_w, R_w)
        if best is None or best[0] >= prior_trace:
            return "declined"
        _, P_w, R_w = best
        return "fused" if self._apply(nu, H, P_w, R_w) else "gated"


def _meas_by_step(rd: RobotData, kind: str | None = None):
    """Group a robot's measurements by grid step -> list of (subject, range, bearing)."""
    ms = rd.meas if kind is None else rd.meas[rd.meas["kind"] == kind]
    out = {}
    for step, subject, r, b in ms[["step", "subject", "range", "bearing"]].itertuples(index=False):
        out.setdefault(int(step), []).append((int(subject), float(r), float(b)))
    return out


def dead_reckoning(rd: RobotData, dt: float) -> np.ndarray:
    """Integrate odometry from the groundtruth initial pose. Returns (N,3) trajectory."""
    n = len(rd.v)
    th = wrap_angle(rd.gt[0, 2] + np.concatenate([[0.0], np.cumsum(rd.w[:-1] * dt)]))
    # Heading used for the displacement over step k is the heading at the start of step k.
    x = rd.gt[0, 0] + np.concatenate([[0.0], np.cumsum(rd.v[:-1] * np.cos(th[:-1]) * dt)])
    y = rd.gt[0, 1] + np.concatenate([[0.0], np.cumsum(rd.v[:-1] * np.sin(th[:-1]) * dt)])
    traj = np.column_stack([x, y, th])
    assert traj.shape == (n, 3)
    return traj


def ekf_landmarks(rd: RobotData, landmarks: dict, dt: float,
                  params: NoiseParams | None = None) -> np.ndarray:
    """Single-robot EKF with odometry and known-landmark range/bearing. Returns (N,3)."""
    params = params or NoiseParams()
    n = len(rd.v)
    ekf = EKF(rd.gt[0], params)
    meas = _meas_by_step(rd, kind="landmark")
    traj = np.empty((n, 3))
    traj[0] = ekf.x
    for k in range(1, n):
        ekf.predict(rd.v[k - 1], rd.w[k - 1], dt)
        for subject, r, b in meas.get(k, ()):
            ekf.update_range_bearing(r, b, landmarks[subject])
        traj[k] = ekf.x
    return traj


def ekf_cooperative(ds: Dataset, policy, params: NoiseParams | None = None,
                    seed: int = 0, return_cov: bool = False,
                    landmark_robots=None, fusion: str = "ci"):
    """Joint EKF run for all robots with robot-to-robot fusion governed by ``policy``.

    ``policy`` is a :class:`src.comm.CommPolicy`. For each robot-to-robot
    observation (robot i sees teammate j at step k) the policy decides whether
    j's state message reaches i. Teammate states are snapshotted after the
    landmark updates of step k and before any robot-to-robot update, so the
    outcome does not depend on robot ordering.

    ``landmark_robots`` restricts which robots may use landmark measurements
    (default: all). Robots outside the set are "map-blind": they localize from
    odometry and teammate messages only. This models a heterogeneous team in
    which only some robots have access to absolute positioning.

    Returns ``(trajectories, comm_log)`` where ``trajectories`` maps robot id to
    an (N,3) array. With ``return_cov=True`` a third item maps robot id to the
    (N,) trace of the position covariance.
    """
    params = params or NoiseParams()
    rng = np.random.default_rng(seed)
    policy.reset()
    n = ds.n_steps
    dt = ds.dt
    ids = sorted(ds.robots)
    if landmark_robots is None:
        landmark_robots = set(ids)
    landmark_robots = set(landmark_robots)
    filters = {r: EKF(ds.robots[r].gt[0], params) for r in ids}
    lm_meas = {r: (_meas_by_step(ds.robots[r], kind="landmark") if r in landmark_robots else {})
               for r in ids}
    rb_meas = {r: _meas_by_step(ds.robots[r], kind="robot") for r in ids}
    traj = {r: np.empty((n, 3)) for r in ids}
    cov = {r: np.empty(n) for r in ids} if return_cov else None
    for r in ids:
        traj[r][0] = filters[r].x
        if return_cov:
            cov[r][0] = filters[r].P[0, 0] + filters[r].P[1, 1]
    log = CommLog(per_robot_sent={r: 0 for r in ids}, per_robot_candidates={r: 0 for r in ids})
    landmarks = ds.landmarks
    v = {r: ds.robots[r].v for r in ids}
    w = {r: ds.robots[r].w for r in ids}

    for k in range(1, n):
        t = ds.t[k]
        for r in ids:
            f = filters[r]
            f.predict(v[r][k - 1], w[r][k - 1], dt)
            for subject, rr, bb in lm_meas[r].get(k, ()):
                f.update_range_bearing(rr, bb, landmarks[subject])
        # Snapshot teammate estimates before any robot-to-robot update this step.
        snap = None
        for r in ids:
            obs = rb_meas[r].get(k)
            if not obs:
                continue
            if snap is None:
                snap = {j: (filters[j].x[:2].copy(), filters[j].P[:2, :2].copy()) for j in ids}
            f = filters[r]
            for j, rr, bb in obs:
                log.candidates += 1
                log.per_robot_candidates[r] += 1
                # Trace of the receiver's current position covariance, evaluated just
                # before this candidate fusion (it shrinks after every accepted fusion).
                pos_var = float(f.P[0, 0] + f.P[1, 1])
                if not policy.allow(receiver=r, sender=j, t=t, measured_range=rr, rng=rng,
                                    receiver_pos_var=pos_var):
                    continue
                log.sent += 1
                log.per_robot_sent[r] += 1
                pj, Pj = snap[j]
                outcome = f.update_teammate(rr, bb, pj, Pj, fusion=fusion)
                if outcome == "fused":
                    log.fused += 1
                elif outcome == "gated":
                    log.gated += 1
                else:
                    log.declined += 1
        for r in ids:
            traj[r][k] = filters[r].x
            if return_cov:
                cov[r][k] = filters[r].P[0, 0] + filters[r].P[1, 1]
    if return_cov:
        return traj, log, cov
    return traj, log


# ----------------------------------------------------------------------------
# Scoring
# ----------------------------------------------------------------------------

def score(traj: np.ndarray, rd: RobotData) -> dict:
    """Error metrics against groundtruth, excluding groundtruth dropout windows."""
    m = rd.gt_valid
    e = traj[m, :2] - rd.gt[m, :2]
    d = np.hypot(e[:, 0], e[:, 1])
    eth = np.abs(wrap_angle(traj[m, 2] - rd.gt[m, 2]))
    return dict(
        rmse_xy=float(np.sqrt(np.mean(d ** 2))),
        mean_xy=float(np.mean(d)),
        median_xy=float(np.median(d)),
        p95_xy=float(np.percentile(d, 95)),
        max_xy=float(np.max(d)),
        rmse_theta=float(np.sqrt(np.mean(eth ** 2))),
        final_xy=float(d[-1]),
        n_scored=int(m.sum()),
        n_masked=int((~m).sum()),
    )
