"""Ideal two-joint pointing in a fixed Front–Right–Down root; degrees throughout.

At baseline the lower axis is lateral, the upper axis vertical, and the plate's
chosen forward vector is +X. Orientation is Ry(pitch) Rz(yaw), not Rz Ry.
No astronomy, serial I/O, translation, or telescope-to-plate transform.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pointing:
    """Baseline-relative joint offsets ↔ root azimuth/elevation.

    Signs map increasing measured joint offsets to nose-up pitch / rightward yaw.
    They must be explicit: command polarity alone does not establish FRD polarity.
    Each joint limit must be below 90°, giving one nonsingular local IK branch.
    """

    pitch_sign: int
    yaw_sign: int
    limit: float = 22.5

    def __post_init__(self):
        if any(type(s) is not int or s not in (-1, 1) for s in (self.pitch_sign, self.yaw_sign)):
            raise ValueError("FRD signs must each be +1 or -1")
        if not math.isfinite(self.limit) or not 0 < self.limit < 90:
            raise ValueError("Joint limit must be finite and between 0 and 90 degrees")

    def _joints(self, lower, upper):
        if not all(math.isfinite(q) and abs(q) <= self.limit + 1e-10 for q in (lower, upper)):
            raise ValueError("Pointing lies outside the configured joint workspace")
        return tuple(max(-self.limit, min(self.limit, q)) for q in (lower, upper))

    def direction(self, lower: float, upper: float) -> tuple[float, float, float]:
        """Return the plate-forward unit vector in fixed FRD coordinates."""
        lower, upper = self._joints(lower, upper)
        p, y = math.radians(self.pitch_sign * lower), math.radians(self.yaw_sign * upper)
        return math.cos(p)*math.cos(y), math.sin(y), -math.sin(p)*math.cos(y)

    def forward(self, lower: float, upper: float) -> tuple[float, float]:
        """Return (azimuth, elevation); azimuth positive right, elevation positive up."""
        x, y, z = self.direction(lower, upper)
        return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(-z, math.hypot(x, y)))

    def inverse(self, azimuth: float, elevation: float) -> tuple[float, float]:
        """Return (lower, upper) on the configured local branch; reject unreachable targets."""
        if not math.isfinite(azimuth) or not math.isfinite(elevation) or not -90 <= elevation <= 90:
            raise ValueError("Require finite azimuth and elevation in [-90, 90]")
        a, e = math.radians(azimuth % 360), math.radians(elevation)
        x, y, z = math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), -math.sin(e)
        if x <= 0:
            raise ValueError("Target is outside the front-facing joint workspace")
        p, yaw = math.atan2(-z, x), math.atan2(y, math.hypot(x, z))
        return self._joints(self.pitch_sign * math.degrees(p), self.yaw_sign * math.degrees(yaw))
