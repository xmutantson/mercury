#!/usr/bin/env python3
"""Mechanical post-connect warm gate for real-audio cohort cells.

The gate consumes monotonic delivered-byte counter samples.  A cell may open its
scored window only after the floor and K consecutive stable sliding-window rates.
No/unstable delivery through the ceiling is an under-warmed, unscorable cell.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median


# Single-cell cold precook is approximately 30 s.  The 50 s floor adds 20 s of
# margin and remains inside the policy's 45--60 s range.
WARM_FLOOR_S = 50.0
WARM_CEILING_S = 180.0
WARM_SAMPLE_S = 3.0
WARM_RATE_WINDOW_S = 12.0
WARM_STABLE_SAMPLES = 4
WARM_RATE_TOLERANCE = 0.15
WARM_MAX_TX_AHEAD_BYTES = 262_144


@dataclass(frozen=True)
class WarmDecision:
    confirmed: bool
    under_warmed: bool
    warm_seconds: float
    rate_Bps: float | None
    stable_samples: int


class DeliveredRateWarmGate:
    """State machine driven by elapsed-seconds/cumulative-byte observations."""

    def __init__(
        self,
        floor_s=WARM_FLOOR_S,
        ceiling_s=WARM_CEILING_S,
        sample_s=WARM_SAMPLE_S,
        rate_window_s=WARM_RATE_WINDOW_S,
        stable_samples=WARM_STABLE_SAMPLES,
        tolerance=WARM_RATE_TOLERANCE,
    ):
        if not (0 <= floor_s < ceiling_s):
            raise ValueError("warm gate requires 0 <= floor < ceiling")
        if sample_s <= 0 or rate_window_s < sample_s:
            raise ValueError("warm sample/window must be positive and window >= sample")
        if stable_samples < 1 or not (0 <= tolerance < 1):
            raise ValueError("invalid warm stability parameters")
        # CLI knobs may tighten the gate for commissioning, never weaken the
        # scored-cohort policy.  This prevents an environment override from
        # turning the artifact back into a prose convention.
        if floor_s < WARM_FLOOR_S:
            raise ValueError(f"warm floor may not be below {WARM_FLOOR_S:g}s")
        if ceiling_s > WARM_CEILING_S:
            raise ValueError(f"warm ceiling may not exceed {WARM_CEILING_S:g}s")
        if not 2.0 <= sample_s <= 5.0:
            raise ValueError("warm sample interval must remain in the 2--5s policy range")
        if rate_window_s < WARM_RATE_WINDOW_S:
            raise ValueError(
                f"warm rate window may not be below {WARM_RATE_WINDOW_S:g}s")
        if stable_samples < WARM_STABLE_SAMPLES:
            raise ValueError(
                f"warm stable sample count may not be below {WARM_STABLE_SAMPLES}")
        if tolerance > WARM_RATE_TOLERANCE:
            raise ValueError(
                f"warm rate tolerance may not exceed {WARM_RATE_TOLERANCE:g}")
        self.floor_s = float(floor_s)
        self.ceiling_s = float(ceiling_s)
        self.sample_s = float(sample_s)
        self.rate_window_s = float(rate_window_s)
        self.stable_samples = int(stable_samples)
        self.tolerance = float(tolerance)
        self._samples = deque()
        self._rates = deque(maxlen=self.stable_samples)
        self._last_sample_s = None
        self._last_bytes = None
        self._decision = None

    @property
    def decision(self):
        return self._decision

    def observe(self, elapsed_s, delivered_bytes):
        """Observe a cumulative counter; return a terminal decision or ``None``.

        Calls faster than sample_s are harmless and ignored, except that the
        ceiling is always enforced at the first call at/after it.
        """
        elapsed_s = float(elapsed_s)
        delivered_bytes = int(delivered_bytes)
        if self._decision is not None:
            return self._decision
        if elapsed_s < 0 or delivered_bytes < 0:
            raise ValueError("warm observations must be non-negative")
        if (self._last_bytes is not None
                and delivered_bytes < self._last_bytes):
            raise ValueError("delivered-byte counter moved backwards")

        if elapsed_s >= self.ceiling_s:
            self._decision = WarmDecision(
                confirmed=False,
                under_warmed=True,
                warm_seconds=self.ceiling_s,
                rate_Bps=self._rates[-1] if self._rates else None,
                stable_samples=len(self._rates),
            )
            return self._decision

        if (self._last_sample_s is not None
                and elapsed_s - self._last_sample_s < self.sample_s):
            return None
        self._last_sample_s = elapsed_s
        self._last_bytes = delivered_bytes
        self._samples.append((elapsed_s, delivered_bytes))

        # Retain one point at/before the trailing-window boundary so its delta
        # provides a real window rate; discard anything older.
        boundary = elapsed_s - self.rate_window_s
        while len(self._samples) >= 2 and self._samples[1][0] <= boundary:
            self._samples.popleft()
        if not self._samples:
            return None
        base_s, base_bytes = self._samples[0]
        span_s = elapsed_s - base_s
        if span_s < self.rate_window_s * 0.95:
            return None
        rate = (delivered_bytes - base_bytes) / span_s
        self._rates.append(rate)

        if elapsed_s < self.floor_s or len(self._rates) < self.stable_samples:
            return None
        center = median(self._rates)
        stable = center > 0 and all(
            abs(value - center) <= center * self.tolerance
            for value in self._rates
        )
        if stable:
            self._decision = WarmDecision(
                confirmed=True,
                under_warmed=False,
                warm_seconds=elapsed_s,
                rate_Bps=rate,
                stable_samples=len(self._rates),
            )
            return self._decision
        return None


def scored_delta(total, at_window_start, under_warmed):
    """Return a non-negative scored counter delta, or None if no window opened."""
    if under_warmed or at_window_start is None:
        return None
    return max(0, total - at_window_start)


def warm_gate_constants():
    return {
        "floor_s": WARM_FLOOR_S,
        "ceiling_s": WARM_CEILING_S,
        "sample_s": WARM_SAMPLE_S,
        "rate_window_s": WARM_RATE_WINDOW_S,
        "stable_samples": WARM_STABLE_SAMPLES,
        "rate_tolerance_fraction": WARM_RATE_TOLERANCE,
        "max_tx_ahead_bytes": WARM_MAX_TX_AHEAD_BYTES,
    }
