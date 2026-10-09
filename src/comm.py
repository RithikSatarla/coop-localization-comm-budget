"""Communication constraints for the cooperative EKF.

A robot-to-robot range/bearing observation (robot i sees teammate j) can only
be fused if j's current state estimate reaches i. Each policy below decides,
per observation, whether that message is delivered. Three families are
implemented, matching the sweeps in ``run_experiments.py``:

* :class:`DropPolicy`    every message is lost independently with probability p.
* :class:`RadiusPolicy`  a message is delivered only if the measured range to
                         the teammate is within the comm radius (metres).
* :class:`RatePolicy`    each receiving robot may fuse at most one teammate
                         message per ``min_interval`` seconds.

All randomness comes from the ``numpy.random.Generator`` passed in by the
caller, so a run is fully determined by its seed.
"""
from __future__ import annotations

import math

import numpy as np


class CommPolicy:
    """Base class. ``allow`` is called once per robot-to-robot observation."""

    name = "base"

    def reset(self) -> None:
        """Clear any per-run state (called at the start of every run)."""

    def allow(self, receiver: int, sender: int, t: float, measured_range: float,
              rng: np.random.Generator) -> bool:
        raise NotImplementedError

    def describe(self) -> dict:
        return {"policy": self.name}


class FullComm(CommPolicy):
    """Unconstrained cooperative EKF: every message is delivered."""

    name = "full"

    def allow(self, receiver, sender, t, measured_range, rng) -> bool:
        return True


class NoComm(CommPolicy):
    """No robot-to-robot fusion at all (equivalent to the landmark-only EKF)."""

    name = "none"

    def allow(self, receiver, sender, t, measured_range, rng) -> bool:
        return False


class DropPolicy(CommPolicy):
    """Independent message loss with probability ``p``."""

    name = "drop"

    def __init__(self, p: float):
        if not 0.0 <= p <= 1.0:
            raise ValueError("drop probability must be in [0, 1]")
        self.p = float(p)

    def allow(self, receiver, sender, t, measured_range, rng) -> bool:
        if self.p >= 1.0:
            return False
        if self.p <= 0.0:
            return True
        return bool(rng.random() >= self.p)

    def describe(self) -> dict:
        return {"policy": self.name, "p": self.p}


class RadiusPolicy(CommPolicy):
    """Deliver only when the measured range is within ``radius`` metres (inf = unlimited)."""

    name = "radius"

    def __init__(self, radius: float):
        if radius <= 0.0:
            raise ValueError("comm radius must be positive")
        self.radius = float(radius)

    def allow(self, receiver, sender, t, measured_range, rng) -> bool:
        return measured_range <= self.radius

    def describe(self) -> dict:
        return {"policy": self.name, "radius": self.radius}


class RatePolicy(CommPolicy):
    """At most one teammate fusion per receiving robot every ``min_interval`` seconds.

    ``min_interval = 0`` means unlimited; ``math.inf`` means never.
    """

    name = "rate"

    def __init__(self, min_interval: float):
        if min_interval < 0.0:
            raise ValueError("min_interval must be non-negative")
        self.min_interval = float(min_interval)
        self._last: dict[int, float] = {}

    def reset(self) -> None:
        self._last = {}

    def allow(self, receiver, sender, t, measured_range, rng) -> bool:
        if math.isinf(self.min_interval):
            return False
        last = self._last.get(receiver)
        if last is not None and (t - last) < self.min_interval:
            return False
        self._last[receiver] = t
        return True

    def describe(self) -> dict:
        return {"policy": self.name, "min_interval": self.min_interval}


# Sweep definitions used by run_experiments.py.
DROP_PROBS = (0.0, 0.25, 0.5, 0.75, 0.9, 1.0)
COMM_RADII = (1.0, 2.0, 4.0, 8.0, math.inf)
RATE_INTERVALS = (1.0, 5.0, 10.0, 30.0, math.inf)


def setting_label(policy: CommPolicy) -> str:
    """Human-readable label for a sweep setting, used in CSVs and figures."""
    d = policy.describe()
    if policy.name == "drop":
        return f"p={d['p']:g}"
    if policy.name == "radius":
        return "unlimited" if math.isinf(d["radius"]) else f"{d['radius']:g} m"
    if policy.name == "rate":
        return "inf" if math.isinf(d["min_interval"]) else f"{d['min_interval']:g} s"
    return policy.name
