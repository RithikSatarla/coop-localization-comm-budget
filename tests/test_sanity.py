"""Sanity tests on a synthetic team (no dataset download needed).

Run with:  python -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.comm import DropPolicy, FullComm, NoComm, RadiusPolicy, RatePolicy  # noqa: E402
from src.loader import Dataset, RobotData, wrap_angle  # noqa: E402
from src.localize import (EKF, NoiseParams, dead_reckoning, ekf_cooperative,  # noqa: E402
                          ekf_landmarks, score)
from src.analysis import knee_point  # noqa: E402


def make_synthetic(n_robots=3, duration=90.0, dt=0.02, seed=0, odom_noise=(0.02, 0.02),
                   odom_bias=(0.01, 0.005), meas_every=25, rr_every=25,
                   sigma_r=0.05, sigma_b=0.01, landmark_range=7.0):
    """Robots drive circles; odometry is noisy and biased; measurements are noisy.

    Ground truth is produced by the same discrete unicycle recursion the
    estimators use, so with noise-free odometry dead reckoning is exact.
    """
    rng = np.random.default_rng(seed)
    n = int(round(duration / dt))
    t = dt * np.arange(n)
    landmarks = {6 + i: np.array([5.0 * math.cos(2 * math.pi * i / 6),
                                  5.0 * math.sin(2 * math.pi * i / 6)]) for i in range(6)}
    gts = {}
    cmds = {}
    for r in range(1, n_robots + 1):
        radius = 1.5 + 0.6 * r
        w_true = 0.12
        v_true = w_true * radius
        phase = 2 * math.pi * r / n_robots
        x = np.empty((n, 3))
        x[0] = [radius * math.cos(phase), radius * math.sin(phase), wrap_angle(phase + math.pi / 2)]
        for k in range(1, n):
            px, py, th = x[k - 1]
            x[k] = [px + v_true * math.cos(th) * dt, py + v_true * math.sin(th) * dt,
                    wrap_angle(th + w_true * dt)]
        gts[r] = x
        v_cmd = v_true + odom_bias[0] + odom_noise[0] * rng.standard_normal(n)
        w_cmd = w_true + odom_bias[1] + odom_noise[1] * rng.standard_normal(n)
        cmds[r] = (v_cmd, w_cmd)
    robots = {}
    for r in range(1, n_robots + 1):
        rows = []
        gt = gts[r]
        for k in range(meas_every, n, meas_every):
            px, py, th = gt[k]
            for lid, (lx, ly) in landmarks.items():
                d = math.hypot(lx - px, ly - py)
                if d <= landmark_range:
                    rows.append((k, t[k], lid, "landmark", d + sigma_r * rng.standard_normal(),
                                 wrap_angle(math.atan2(ly - py, lx - px) - th
                                            + sigma_b * rng.standard_normal())))
        for k in range(rr_every // 2, n, rr_every):
            px, py, th = gt[k]
            for j in range(1, n_robots + 1):
                if j == r:
                    continue
                jx, jy = gts[j][k, :2]
                d = math.hypot(jx - px, jy - py)
                rows.append((k, t[k], j, "robot", d + sigma_r * rng.standard_normal(),
                             wrap_angle(math.atan2(jy - py, jx - px) - th
                                        + sigma_b * rng.standard_normal())))
        meas = pd.DataFrame(rows, columns=["step", "time", "subject", "kind", "range", "bearing"])
        meas.sort_values(["step", "subject"], inplace=True, kind="mergesort")
        meas.reset_index(drop=True, inplace=True)
        v_cmd, w_cmd = cmds[r]
        robots[r] = RobotData(robot_id=r, v=v_cmd, w=w_cmd, odom_valid=np.ones(n, bool),
                              gt=gt, gt_valid=np.ones(n, bool), meas=meas,
                              gt_dropout_seconds=0.0)
    landmark_std = {k: np.zeros(2) for k in landmarks}
    return Dataset(dataset_id=0, dt=dt, t=t, t0=0.0, landmarks=landmarks,
                   landmark_std=landmark_std, robots=robots, n_unknown_barcode=0)


class TestBasics(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(float(wrap_angle(3 * math.pi)), -math.pi)
        self.assertAlmostEqual(float(wrap_angle(-3.5 * math.pi)), 0.5 * math.pi)
        self.assertTrue(np.all(np.abs(wrap_angle(np.linspace(-20, 20, 1001))) <= math.pi))

    def test_dead_reckoning_exact_with_perfect_odometry(self):
        ds = make_synthetic(odom_noise=(0.0, 0.0), odom_bias=(0.0, 0.0))
        for r, rd in ds.robots.items():
            traj = dead_reckoning(rd, ds.dt)
            err = np.hypot(traj[:, 0] - rd.gt[:, 0], traj[:, 1] - rd.gt[:, 1])
            self.assertLess(err.max(), 1e-9)

    def test_score_ignores_masked_groundtruth(self):
        ds = make_synthetic()
        rd = ds.robots[1]
        traj = rd.gt.copy()
        s_clean = score(traj, rd)
        self.assertAlmostEqual(s_clean["rmse_xy"], 0.0)
        rd.gt_valid[100:200] = False
        traj[100:200, :2] += 50.0
        s_masked = score(traj, rd)
        self.assertAlmostEqual(s_masked["rmse_xy"], 0.0)
        self.assertEqual(s_masked["n_masked"], 100)


class TestEstimators(unittest.TestCase):
    def setUp(self):
        self.ds = make_synthetic()

    def test_landmark_ekf_beats_dead_reckoning(self):
        for r, rd in self.ds.robots.items():
            dr = score(dead_reckoning(rd, self.ds.dt), rd)["rmse_xy"]
            ekf = score(ekf_landmarks(rd, self.ds.landmarks, self.ds.dt), rd)["rmse_xy"]
            self.assertLess(ekf, 0.15)
            self.assertLess(ekf, dr / 3)

    def test_cooperative_identities(self):
        ds = self.ds
        per_robot = {r: ekf_landmarks(ds.robots[r], ds.landmarks, ds.dt) for r in ds.robots}
        none, log_none = ekf_cooperative(ds, NoComm())
        self.assertEqual(log_none.sent, 0)
        self.assertGreater(log_none.candidates, 0)
        for r in ds.robots:
            np.testing.assert_allclose(none[r], per_robot[r], atol=1e-12)
        full, log_full = ekf_cooperative(ds, FullComm())
        p0, log_p0 = ekf_cooperative(ds, DropPolicy(0.0))
        p1, log_p1 = ekf_cooperative(ds, DropPolicy(1.0))
        self.assertEqual(log_full.sent, log_full.candidates)
        self.assertEqual(log_p0.sent, log_full.sent)
        self.assertEqual(log_p1.sent, 0)
        for r in ds.robots:
            np.testing.assert_array_equal(full[r], p0[r])
            np.testing.assert_array_equal(none[r], p1[r])
        a, la = ekf_cooperative(ds, DropPolicy(0.5), seed=3)
        b, lb = ekf_cooperative(ds, DropPolicy(0.5), seed=3)
        c, lc = ekf_cooperative(ds, DropPolicy(0.5), seed=4)
        self.assertEqual(la.sent, lb.sent)
        for r in ds.robots:
            np.testing.assert_array_equal(a[r], b[r])
        self.assertNotEqual(la.sent, lc.sent)
        self.assertLess(abs(la.sent / la.candidates - 0.5), 0.1)

    def test_map_blind_robot_benefits_from_messages(self):
        ds = self.ds
        anchored = {1}
        none, _ = ekf_cooperative(ds, NoComm(), landmark_robots=anchored)
        full, log = ekf_cooperative(ds, FullComm(), landmark_robots=anchored)
        self.assertGreater(log.fused, 0)
        for r in (2, 3):
            e_none = score(none[r], ds.robots[r])["rmse_xy"]
            e_full = score(full[r], ds.robots[r])["rmse_xy"]
            self.assertLess(e_full, e_none / 2)
            self.assertLess(e_full, 0.3)
        # The anchored robot must not be degraded by fusing poorly localised teammates.
        e1_none = score(none[1], ds.robots[1])["rmse_xy"]
        e1_full = score(full[1], ds.robots[1])["rmse_xy"]
        self.assertLess(e1_full, e1_none * 1.5 + 0.02)

    def test_covariance_intersection_is_more_conservative_than_naive(self):
        params = NoiseParams()
        prior = EKF(np.array([0.0, 0.0, 0.0]), params)
        prior.P = np.diag([0.2 ** 2, 0.2 ** 2, 0.05 ** 2])
        point = np.array([2.0, 1.0])
        point_cov = np.diag([0.15 ** 2, 0.15 ** 2])
        r = math.hypot(*point)
        b = math.atan2(point[1], point[0])
        naive = EKF(prior.x.copy(), params)
        naive.P = prior.P.copy()
        ci = EKF(prior.x.copy(), params)
        ci.P = prior.P.copy()
        self.assertEqual(naive.update_teammate(r, b, point, point_cov, fusion="naive"), "fused")
        self.assertEqual(ci.update_teammate(r, b, point, point_cov, fusion="ci"), "fused")
        self.assertGreaterEqual(np.trace(ci.P), np.trace(naive.P))
        self.assertLess(np.trace(ci.P), np.trace(prior.P))

    def test_uninformative_teammate_is_declined(self):
        params = NoiseParams()
        f = EKF(np.array([0.0, 0.0, 0.0]), params)
        f.P = np.diag([0.01 ** 2, 0.01 ** 2, 0.01 ** 2])
        point = np.array([2.0, 1.0])
        huge = np.diag([10.0 ** 2, 10.0 ** 2])
        outcome = f.update_teammate(math.hypot(*point), math.atan2(1.0, 2.0), point, huge, "ci")
        self.assertEqual(outcome, "declined")


class TestCommPolicies(unittest.TestCase):
    def test_drop_fraction(self):
        rng = np.random.default_rng(0)
        pol = DropPolicy(0.75)
        n = 20000
        allowed = sum(pol.allow(1, 2, 0.0, 1.0, rng) for _ in range(n))
        self.assertLess(abs(allowed / n - 0.25), 0.02)

    def test_radius(self):
        pol = RadiusPolicy(2.0)
        rng = np.random.default_rng(0)
        self.assertTrue(pol.allow(1, 2, 0.0, 1.99, rng))
        self.assertFalse(pol.allow(1, 2, 0.0, 2.01, rng))
        self.assertTrue(RadiusPolicy(math.inf).allow(1, 2, 0.0, 1e9, rng))

    def test_rate(self):
        pol = RatePolicy(5.0)
        rng = np.random.default_rng(0)
        pol.reset()
        self.assertTrue(pol.allow(1, 2, 0.0, 1.0, rng))
        self.assertFalse(pol.allow(1, 3, 4.9, 1.0, rng))   # same receiver, too soon
        self.assertTrue(pol.allow(2, 1, 4.9, 1.0, rng))    # other receiver is independent
        self.assertTrue(pol.allow(1, 3, 5.0, 1.0, rng))
        self.assertFalse(RatePolicy(math.inf).allow(1, 2, 0.0, 1.0, rng))
        pol.reset()
        self.assertTrue(pol.allow(1, 2, 100.0, 1.0, rng))


class TestAnalysis(unittest.TestCase):
    def test_knee_on_convex_curve(self):
        x = np.array([0, 1, 2, 4, 8, 16, 32, 64], float)
        y = 0.2 + 2.0 / (1.0 + x)
        res = knee_point(x, y)
        self.assertIsNotNone(res)
        _, kx, ky = res
        self.assertTrue(1.0 <= kx <= 8.0)

    def test_knee_none_on_flat(self):
        x = np.array([0, 1, 2, 3], float)
        y = np.array([1.0, 1.0, 1.0, 1.0])
        self.assertIsNone(knee_point(x, y))


if __name__ == "__main__":
    unittest.main()
