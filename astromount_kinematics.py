"""Ideal two-joint pointing in a fixed Front–Right–Down root; degrees throughout.

At baseline the lower axis is lateral, the upper axis vertical, and the plate's
chosen forward vector is +X. Orientation is Ry(pitch) Rz(yaw), not Rz Ry.

Active rotations of column vectors, from plate coordinates into the fixed root:
p = pitch_sign * lower, y = yaw_sign * upper (converted to radians for trig).
With cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y):

            [ cp   0   sp ]              [ cy  -sy   0 ]
    Ry(p) = [  0   1    0 ]      Rz(y) = [ sy   cy   0 ]
            [-sp   0   cp ]              [  0    0   1 ]

                         [ cp*cy  -cp*sy   sp ]
    R = Ry(p) @ Rz(y) =  [    sy      cy    0 ]
                         [-sp*cy   sp*sy   cp ]

The lower joint carries the upper joint's axis. The plate-forward direction is
d = R @ [1, 0, 0]^T = [cp*cy, sy, -sp*cy]^T.
Root azimuth = atan2(d_y, d_x); elevation = atan2(-d_z, hypot(d_x, d_y)).

No astronomy, serial I/O, translation, or telescope-to-plate transform.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pointing:
    """Baseline-relative joint offsets ↔ root azimuth/elevation.

    Signs map increasing measured joint offsets to nose-up pitch / rightward yaw.
    They must be explicit: command polarity alone does not establish FRD polarity.
    IK selects the front-facing branch; FK accepts any finite joint angles.
    """

    pitch_sign: int
    yaw_sign: int

    def __post_init__(self):
        if any(type(s) is not int or s not in (-1, 1) for s in (self.pitch_sign, self.yaw_sign)):
            raise ValueError("FRD signs must each be +1 or -1")

    def _joints(self, lower, upper):
        if not all(math.isfinite(q) for q in (lower, upper)):
            raise ValueError("Joint angles must be finite")
        return lower, upper

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
        if x <= 1e-12:
            raise ValueError("Target is outside the front-facing joint workspace")
        p, yaw = math.atan2(-z, x), math.atan2(y, math.hypot(x, z))
        return self._joints(self.pitch_sign * math.degrees(p), self.yaw_sign * math.degrees(yaw))
