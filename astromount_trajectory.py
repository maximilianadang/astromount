"""Az/el trajectory planners using the existing latest-target worker."""

import math
from dataclasses import asdict, replace
from time import monotonic, sleep

from astromount_control import ControlError, Worker, require_status


def sweep(control, frame, waypoints, *, delta=False, log=None, cancel=None):
    """Stream time-interpolated az/el; cumulative deltas, no waypoint stops.

    Durations schedule targets, not guaranteed arrival. The caller owns Mount;
    Worker owns serial I/O during streaming and stops on every exit.
    """
    if cancel is not None and cancel.is_set(): raise InterruptedError('Sweep canceled')
    settings = control.settings
    current = control.read()
    if log: log('initial', state=asdict(current), measured_azel=current.pointing(frame))
    require_status(current.status, stationary=True)
    origin = current.pointing(frame)
    segments, elapsed, start = [], 0., origin
    for az, el, duration in waypoints:
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('Duration must be finite and positive')
        if delta: az, el = start[0] + az, start[1] + el
        # Normalize equivalent azimuths onto the front-facing IK branch.
        target = frame.forward(*frame.inverse(az, el))
        elapsed += duration
        segments.append((elapsed, start, target, duration))
        start = target
    if not segments: raise ValueError('Sweep requires at least one waypoint')
    control.settings = replace(settings, max_speed=settings.speed_limit, timeout=elapsed + settings.timeout)
    try:
        with Worker(control) as worker:
            worker.arm_pointing(frame, azimuth=origin[0], elevation=origin[1])
            began, index, final_sent = monotonic(), 0, False
            if log: log('plan', began_monotonic_s=began, segments=segments, settings=asdict(control.settings), heartbeat=worker.heartbeat)
            while True:
                if cancel is not None and cancel.is_set(): raise InterruptedError('Sweep canceled')
                now, snapshot = monotonic(), worker.snapshot
                if log:
                    log('feedback', snapshot=asdict(snapshot),
                        measured_azel=snapshot.state.pointing(frame) if snapshot.state else None)
                if snapshot.fault or not snapshot.armed:
                    raise ControlError(snapshot.fault or f'Worker is {snapshot.mode.value}')
                age = now - began
                if final_sent and snapshot.arrived: return snapshot.state
                if age >= elapsed + settings.timeout: raise ControlError('Sweep arrival timed out')
                while index < len(segments)-1 and age >= segments[index][0]: index += 1
                end, start, target, duration = segments[index]
                fraction = min(1., max(0., (age - end + duration) / duration))
                az, el = (a + fraction*(b-a) for a, b in zip(start, target))
                sequence = worker.set_pointing(frame, azimuth=az, elevation=el, issued_at=now)
                if log: log('target', issued_at=now, sequence=sequence, segment=index, commanded_azel=(az, el))
                final_sent = age >= elapsed
                (cancel.wait if cancel is not None else sleep)(max(0, settings.period - (monotonic() - now)))
    finally:
        control.settings = settings


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
