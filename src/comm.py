"""Communication constraints for the cooperative EKF.

A robot-to-robot range/bearing observation (robot i sees teammate j) can only
be fused if j's current state estimate reaches i. Each policy below decides,
per observation, whether that message is delivered. Four families are
implemented, matching the sweeps in ``run_experiments.py``:

* :class:`DropPolicy`    every message is lost independently with probability p.
* :class:`RadiusPolicy`  a message is delivered only if the measured range to
                         the teammate is within the comm radius (metres).
* :class:`RatePolicy`    each receiving robot may fuse at most one teammate
                         message per ``min_interval`` seconds.
* :class:`EventTriggeredPolicy`  a simple covariance-threshold trigger: the
                         receiving robot fuses a teammate message only while
                         the trace of its own position covariance exceeds
                         ``tau`` square metres.

All randomness comes from the ``numpy.random.Generator`` passed in by the
caller, so a run is fully determined by its seed. The filter passes the
receiver's current position-covariance trace to ``allow`` so that
state-dependent policies can use it; the other policies ignore it.
"""
from __future__ import annotations

import math

import numpy as np


class CommPolicy:
    """Base class. ``allow`` is called once per robot-to-robot observation.

    ``receiver_pos_var`` is the trace of the receiving robot's current 2x2
    position covariance [m^2], evaluated just before the candidate fusion.
    """

    name = "base"

    def reset(self) -> None:
        """Clear any per-run state (called at the start of every run)."""

    def allow(self, receiver: int, sender: int, t: float, measured_range: float,
              rng: np.random.Generator, receiver_pos_var: float = 0.0) -> bool:
        raise NotImplementedError

    def describe(self) -> dict:
        return {"policy": self.name}


class FullComm(CommPolicy):
    """Unconstrained cooperative EKF: every message is delivered."""

    name = "full"

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
        return True


class NoComm(CommPolicy):
    """No robot-to-robot fusion at all (equivalent to the landmark-only EKF)."""

    name = "none"

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
        return False


class DropPolicy(CommPolicy):
    """Independent message loss with probability ``p``."""

    name = "drop"

    def __init__(self, p: float):
        if not 0.0 <= p <= 1.0:
            raise ValueError("drop probability must be in [0, 1]")
        self.p = float(p)

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
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

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
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

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
        if math.isinf(self.min_interval):
            return False
        last = self._last.get(receiver)
        if last is not None and (t - last) < self.min_interval:
            return False
        self._last[receiver] = t
        return True

    def describe(self) -> dict:
        return {"policy": self.name, "min_interval": self.min_interval}


class EventTriggeredPolicy(CommPolicy):
    """Simple covariance-threshold trigger.

    The receiving robot fuses a teammate message only while the trace of its
    own position covariance exceeds ``tau`` [m^2]. Because a fusion shrinks
    that covariance, the trigger switches itself off until the covariance has
    grown again through odometry, so messages are concentrated in the periods
    in which the receiver is uncertain. It is a receiver-side request ("ask
    for help when uncertain"): a message is counted only when the request is
    made and served. The rule is deterministic and uses only the receiver's
    own filter state.

    This is deliberately the simplest member of the event-triggered family. It
    is not a reimplementation of any published adaptive event-triggered
    scheme (no innovation test, no adaptive threshold, no implicit information
    from the absence of a message).
    """

    name = "trigger"

    def __init__(self, tau: float):
        if tau < 0.0:
            raise ValueError("tau must be non-negative")
        self.tau = float(tau)

    def allow(self, receiver, sender, t, measured_range, rng, receiver_pos_var=0.0) -> bool:
        return receiver_pos_var > self.tau

    def describe(self) -> dict:
        return {"policy": self.name, "tau": self.tau}


# Sweep definitions used by run_experiments.py.
DROP_PROBS = (0.0, 0.25, 0.5, 0.75, 0.9, 1.0)
COMM_RADII = (1.0, 2.0, 4.0, 8.0, math.inf)
RATE_INTERVALS = (1.0, 5.0, 10.0, 30.0, math.inf)
# Covariance-threshold trigger thresholds [m^2]: a geometric grid chosen from a
# calibration run on dataset 1 so that the delivered-message rate spans roughly
# 1 to 50 messages per robot per minute in the map-blind regime (tau <= 0.001
# delivers every message; tau = 3 delivers about 3 per robot per minute).
TRIGGER_TAUS = (0.003, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0)


def setting_label(policy: CommPolicy) -> str:
    """Human-readable label for a sweep setting, used in CSVs and figures."""
    d = policy.describe()
    if policy.name == "drop":
        return f"p={d['p']:g}"
    if policy.name == "radius":
        return "unlimited" if math.isinf(d["radius"]) else f"{d['radius']:g} m"
    if policy.name == "rate":
        return "inf" if math.isinf(d["min_interval"]) else f"{d['min_interval']:g} s"
    if policy.name == "trigger":
        return f"tau={d['tau']:g}"
    return policy.name
