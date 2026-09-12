"""Baseline-centered spherical spiral and latest-target planner; no serial I/O."""

import math
from time import monotonic, sleep

from astromount_control import ControlError


def spiral(fraction, radius=5, turns=2):
    """Az/el: center → cone rim → center, with turns on each leg.

    Radius is the angular distance from root +X, not an az/el rectangle.
    First-test radius is restricted to 5 degrees.
    """
    if not all(math.isfinite(v) for v in (fraction, radius, turns)) or not 0 < radius <= 5 or turns <= 0:
        raise ValueError('Require finite inputs, 0 < radius <= 5 degrees, and positive turns')
    u = max(0, min(1, fraction))
    r = math.radians(radius * (1 - abs(2*u - 1)))
    phase = 4 * math.pi * turns * u
    x, y, z = math.cos(r), math.sin(r)*math.cos(phase), -math.sin(r)*math.sin(phase)
    return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(-z, math.hypot(x, y)))


def trace(worker, frame, *, duration=120, radius=5, turns=2, settle_timeout=30):
    """Reach baseline, stream at 10 Hz, then settle at baseline.

    Caller owns the worker context (stop on exit). Elapsed-time targets skip
    missed ticks; they are never queued. Existing worker faults propagate.
    """
    if not all(math.isfinite(v) and v > 0 for v in (duration, settle_timeout)):
        raise ValueError('Duration and settle timeout must be finite and positive')
    spiral(0, radius, turns)
    worker.arm_pointing(frame, azimuth=0, elevation=0)
    for moving in (False, True, False):
        start = monotonic()
        while True:
            now, snapshot = monotonic(), worker.snapshot
            if snapshot.fault or not snapshot.armed:
                raise ControlError(snapshot.fault or f'Worker is {snapshot.mode.value}')
            if moving and now - start >= duration:
                break
            if not moving and snapshot.arrived:
                break
            if not moving and now - start >= settle_timeout:
                raise ControlError('Baseline arrival timed out')
            az, el = spiral((now-start)/duration, radius, turns) if moving else (0, 0)
            worker.set_pointing(frame, azimuth=az, elevation=el, issued_at=now)
            sleep(max(0, .1 - (monotonic() - now)))
        # Clear arrival for the old spiral target before testing final settling.
        worker.set_pointing(frame, azimuth=0, elevation=0)
    return worker.snapshot.state
