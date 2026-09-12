"""Bounded joint-space P control. Importing/constructing objects performs no I/O."""

from dataclasses import dataclass, replace
from enum import Enum
import json
import math
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic, sleep

from astromount import Mount, decode_coordinates


class ControlError(RuntimeError):
    """Control cannot continue; the run attempts to stop before propagating errors."""


def require_status(status, *, stationary=False):
    """Shared control/capture preflight; this does not send commands."""
    if not all(c in status for c in ('nNG' if stationary else 'nG')) or any(c in status for c in 'ZLSsTt'):
        raise ControlError(f'Require EQ mode, tracking off, no faults{" and stationary" if stationary else ""}: {status!r}')


def _wrap(angle):
    return (angle + 180) % 360 - 180


@dataclass(frozen=True)
class Reference:
    """Mechanical-model reference; valid only while firmware/base reference persists."""

    hour_angle_degrees: float
    declination_degrees: float

    def __post_init__(self):
        if not all(math.isfinite(v) and -180 <= v <= 180 for v in self.angles):
            raise ValueError("Reference angles must be finite and in [-180, 180]")

    @property
    def angles(self):
        return self.hour_angle_degrees, self.declination_degrees

    @classmethod
    def from_baseline(cls, path: str | Path, *, beyond_pole: bool = False):
        """Load a saved sky baseline with an explicitly selected reference branch.

        The original +90 DEC baseline uses the default branch. This does not
        establish physical homing, direction polarity, or validity after reboot.
        """
        return cls.from_queries(json.loads(Path(path).read_text())["queries"], beyond_pole=beyond_pole)

    @classmethod
    def from_queries(cls, data, *, beyond_pole=False):
        """Decode a clock-bracketed baseline without filesystem access."""
        h, d = decode_coordinates(data)
        return cls(_wrap(h - 180), _wrap(180 - d)) if beyond_pole else cls(h, d)

    def offsets(self, hour_angle, declination, previous=(0.0, 0.0)):
        """Nearest continuous mechanical branch, expressed relative to this zero."""
        if not all(math.isfinite(v) for v in (hour_angle, declination, *previous)) or not -90 <= declination <= 90:
            raise ControlError("Invalid joint sample")
        candidates = ((hour_angle, declination), (hour_angle - 180, 180 - declination))
        offsets = [tuple(_wrap(v - zero) for v, zero in zip(pair, self.angles)) for pair in candidates]
        return min(offsets, key=lambda pair: sum((v - old)**2 for v, old in zip(pair, previous)))


@dataclass(frozen=True)
class Settings:
    kp: float = 0.5                 # (deg/s) per degree of error
    max_speed: float = 0.1          # deg/s, common cap for either axis
    deadband: float = 0.01          # degrees
    excursion: float = 22.5         # degrees from Reference on each axis
    margin: float = 0.25            # stop inside excursion boundary
    period: float = 0.1             # seconds; requested sampling period
    max_sample_age: float = 0.25    # seconds, including complete read transaction
    timeout: float = 300            # seconds per target
    progress_timeout: float = 5     # seconds without measurable commanded motion
    settle_samples: int = 3

    def __post_init__(self):
        values = (self.kp, self.max_speed, self.deadband, self.excursion,
                  self.margin, self.period, self.max_sample_age, self.timeout, self.progress_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Controller settings must be finite and positive")
        if not self.deadband < self.margin < self.excursion < 90 or self.max_speed > 6:
            raise ValueError("Require deadband < margin < excursion < 90 and max_speed <= 6")
        if self.kp * self.deadband < 0.01 * (360 / 86164.0905):
            raise ValueError("Gain/deadband demand speeds below firmware resolution")
        if self.margin <= self.max_speed * (self.period + self.max_sample_age) + self.deadband:
            raise ValueError("Margin must exceed nominal sampling travel plus deadband")
        if type(self.settle_samples) is not int or self.settle_samples < 1:
            raise ValueError("settle_samples must be a positive integer")


@dataclass(frozen=True)
class State:
    ra_degrees: float
    dec_degrees: float
    started_at: float
    finished_at: float
    status: str

    @property
    def angles(self):
        return self.ra_degrees, self.dec_degrees

    def pointing(self, frame):
        """Model-estimated fixed-root az/el from measured joints, in degrees."""
        return frame.forward(*self.angles)


def duration_settings(settings, current, target, duration):
    """Settings for a nominal move duration; arrival still requires settling."""
    if not math.isfinite(duration) or duration <= 0: raise ValueError('Duration must be finite and positive')
    require_status(current.status, stationary=True)
    distance = max(abs(a-b) for a, b in zip(target, current.angles))
    speed = min(3, distance / duration) if distance > settings.deadband else settings.max_speed
    if speed < .01 * (360 / 86164.0905): raise ValueError('Duration demands a speed below firmware resolution')
    return replace(settings, max_speed=speed, timeout=duration + settings.timeout)


class Controller:
    """Concurrent two-axis P control with axis-local rate updates.

    positive_directions must be calibrated for increasing model RA/DEC, not sky
    RA or screen directions. Own the Mount for a run; do not use another client.
    No goto, home, sync, or tracking configuration is performed.
    """

    def __init__(self, mount: Mount, reference: Reference, *, positive_directions,
                 settings: Settings = Settings()):
        if len(positive_directions) != 2 or positive_directions[0] not in ("east", "west") or positive_directions[1] not in ("north", "south"):
            raise ValueError("Provide calibrated positive directions: (east/west, north/south)")
        self.mount, self.reference, self.settings = mount, reference, settings
        self.positive_directions = tuple(positive_directions)
        self._previous = None
        self._action = [0.0, 0.0]
        self._progress = [None, None]
        self._interval_motion = [False, False]  # Commanded motion since the last accepted sample.

    def read(self) -> State:
        """Read and check estimated joint offsets; sends getters only."""
        start = monotonic()
        h, d, status = self.mount.joint_sample()
        end = monotonic()
        s = self.settings
        if end - start > s.max_sample_age:
            raise ControlError("Position transaction exceeded max_sample_age")
        require_status(status)
        old = self._previous
        angles = self.reference.offsets(h, d, old.angles if old else (0, 0))
        if any(abs(v) >= s.excursion - s.margin for v in angles):
            raise ControlError("Position reached the excursion stopping boundary")
        if old:
            # Reject discontinuities, wrong-axis motion and gross overspeed.
            allowance = s.max_speed * (end - old.started_at) * 1.1 + s.deadband
            for axis, (new, previous) in enumerate(zip(angles, old.angles)):
                bound = allowance if self._interval_motion[axis] else s.deadband
                if abs(new - previous) > bound:
                    raise ControlError(f"Unexpected axis displacement or coordinate discontinuity: axis={axis}, "
                                       f"previous={previous:.6f}, measured={new:.6f}, bound={bound:.6f}, "
                                       f"interval_motion={self._interval_motion[axis]}, command={self._action[axis]:.6f}, "
                                       f"elapsed={end-old.started_at:.6f}, status={status}")
        for axis, velocity in enumerate(self._action):
            if not velocity or self._progress[axis] is None:
                continue
            direction_origin, origin, since = self._progress[axis]
            progress = (angles[axis] - direction_origin) * (1 if velocity > 0 else -1)
            if progress < -s.deadband:
                raise ControlError("Motion opposes the calibrated command direction")
            if abs(angles[axis] - origin) >= s.deadband:
                self._progress[axis] = (angles[axis], angles[axis], end)
            elif end - since > s.progress_timeout:
                raise ControlError("No measurable progress; feedback or motion may have stalled")
        state = State(*angles, start, end, status)
        self._previous = state
        self._interval_motion = [bool(v) for v in self._action]
        return state

    def _drive(self, action, permit=lambda: True):
        if not permit(): return False
        for axis, velocity in enumerate(action):
            previous = self._action[axis]
            if velocity == previous: continue
            if not permit(): return False
            if previous:
                self.mount.stop(self._direction(axis, previous))
                self._action[axis] = 0.0
            if velocity:
                if not permit():  # Stop/other-axis I/O may outlive the target or lease.
                    return False
                if not previous or previous * velocity <= 0:
                    origin, since = self._progress[axis][1:] if self._progress[axis] else (self._previous.angles[axis], monotonic())
                    self._progress[axis] = (self._previous.angles[axis], origin, since)
                self._interval_motion[axis] = True  # Preserve even if dispatch fails or restart is cancelled later.
                self.mount.jog(self._direction(axis, velocity), speed_degrees_s=abs(velocity))
                self._action[axis] = velocity
            else:
                self._progress[axis] = None
        return True

    def _direction(self, axis, velocity):
        direction = self.positive_directions[axis]
        return direction if velocity > 0 else {"east": "west", "west": "east", "north": "south", "south": "north"}[direction]

    def _stop(self):
        """Best-effort stop even if a framing/transport error closed the port."""
        try:
            if self.mount._serial.is_open:
                self.mount.stop()
            else:
                with Mount(self.mount._serial.port, baudrate=self.mount._serial.baudrate,
                           timeout=self.mount._timeout) as recovery:
                    recovery.stop()
        except Exception as exc:
            raise ControlError("Stop transmission failed; use the physical E-stop") from exc
        finally:
            self._action, self._progress = [0.0, 0.0], [None, None]

    def _target(self, ra_degrees, dec_degrees):
        target, s = (ra_degrees, dec_degrees), self.settings
        if not all(math.isfinite(v) and abs(v) < s.excursion - s.margin - s.deadband for v in target):
            raise ValueError("Target must be inside excursion minus margin and deadband")
        return target

    def _step(self, target, state, permit=lambda: True):
        s = self.settings
        def fresh_permit():
            if monotonic() - state.started_at > s.max_sample_age:
                raise ControlError("State became stale")
            return permit()
        error = tuple(t - p for t, p in zip(target, state.angles))
        velocities = tuple(0 if abs(e) <= s.deadband else max(-s.max_speed, min(s.max_speed, s.kp * e)) for e in error)
        return self._drive(velocities, fresh_permit) and not any(velocities) and "N" in state.status

    def run_pointing(self, frame, *, azimuth, elevation, on_sample=None, cancel=None):
        """Reach a fixed-root az/el target with concurrent joint control."""
        lower, upper = frame.inverse(azimuth, elevation)
        return self.run(ra_degrees=lower, dec_degrees=upper, on_sample=on_sample, cancel=cancel)

    def run(self, *, ra_degrees: float, dec_degrees: float, on_sample=None, cancel=None) -> State:
        """Reach absolute offsets from reference. Returns after stationary settling.

        cancel is an optional threading.Event; on_sample receives State each tick.
        Callbacks must not block. Timeout/cancellation/errors attempt stop and raise.
        """
        target, s = self._target(ra_degrees, dec_degrees), self.settings
        with self.mount._lock:
            self._previous = None
            self._action, self._progress = [0.0, 0.0], [None, None]
            deadline, tick, settled = monotonic() + s.timeout, monotonic(), 0
            first = True
            def permit():
                if cancel is not None and cancel.is_set():
                    raise ControlError("Cancelled")
                if monotonic() >= deadline:
                    raise ControlError("Target timed out")
                return True
            try:
                while True:
                    permit()
                    state = self.read()
                    if first and "N" not in state.status:
                        raise ControlError("Mount must be stationary before starting control")
                    first = False
                    if on_sample: on_sample(state)
                    settled = settled + 1 if self._step(target, state, permit) else 0
                    if settled >= s.settle_samples: return state
                    tick = max(tick + s.period, monotonic())
                    sleep(max(0, tick - monotonic()))
            finally:
                self._stop()


@dataclass(frozen=True)
class Target:
    angles: tuple[float, float]
    issued_at: float
    sequence: int


class Lifecycle(str, Enum):
    NEW = "new"
    DISARMED = "disarmed"
    ARMING = "arming"
    ARMED = "armed"
    STOPPING = "stopping"
    FAULT = "fault"
    CLOSING = "closing"
    CLOSED = "closed"


@dataclass(frozen=True)
class Snapshot:
    mode: Lifecycle = Lifecycle.NEW
    state: State | None = None       # Latest successfully validated measurement.
    target: Target | None = None     # Latest requested target.
    applied: Target | None = None    # Target of last committed control decision.
    arrived: bool = False            # Applies only to target; reset on replacement.
    fault: str | None = None

    @property
    def armed(self):
        return self.mode in (Lifecycle.ARMING, Lifecycle.ARMED)


class Worker:
    """Single-owner serial worker with one immutable lifecycle/mailbox record.

    arm supplies a fresh target; set_target refreshes it. Stop/fault require re-arm.
    Caller owns Mount lifetime and must not access Mount/Controller until close.
    """

    def __init__(self, controller: Controller, *, heartbeat: float = 0.5):
        if not math.isfinite(heartbeat) or heartbeat <= 0:
            raise ValueError("heartbeat must be finite and positive")
        self.controller, self.heartbeat = controller, heartbeat
        self._lock, self._wake = Lock(), Event()
        self._thread, self._sequence = None, 0
        self._snapshot = Snapshot()

    def start(self):
        with self._lock:
            if self._snapshot.mode != Lifecycle.NEW:
                raise RuntimeError("Worker cannot be restarted")
            self._snapshot = replace(self._snapshot, mode=Lifecycle.DISARMED)
            self._thread = Thread(target=self._loop, name="astromount-controller", daemon=True)
            self._thread.start()
        return self

    def __enter__(self): return self.start()

    def __exit__(self, *exc): self.close()

    @property
    def snapshot(self):
        """Coherent immutable state; never waits for serial I/O."""
        with self._lock:
            return self._snapshot

    def _publish(self, ra_degrees, dec_degrees, issued_at, arm):
        angles = self.controller._target(ra_degrees, dec_degrees)
        with self._lock:
            now, current = monotonic(), self._snapshot
            stamp = now if issued_at is None else issued_at
            if not math.isfinite(stamp) or not 0 <= now - stamp < self.heartbeat:
                raise ValueError("Target timestamp must be fresh local monotonic time")
            allowed = (Lifecycle.DISARMED, Lifecycle.FAULT) if arm else (Lifecycle.ARMING, Lifecycle.ARMED)
            if current.mode not in allowed:
                raise ControlError(f"Cannot {'arm' if arm else 'update'} while {current.mode.value}")
            if current.target:
                if stamp <= current.target.issued_at:
                    raise ValueError("Out-of-order target")
                if now - current.target.issued_at >= self.heartbeat:
                    raise ControlError("Heartbeat expired; wait for fault and re-arm")
            self._sequence += 1
            target = Target(angles, stamp, self._sequence)
            self._snapshot = replace(current, target=target,
                                     arrived=not arm and current.arrived and current.target.angles == angles,
                                     mode=Lifecycle.ARMING if arm else current.mode)
            return target.sequence

    def arm(self, *, ra_degrees, dec_degrees, issued_at=None):
        """Explicit re-arm with a fresh target; stationary preflight runs on worker."""
        return self._publish(ra_degrees, dec_degrees, issued_at, True)

    def set_target(self, *, ra_degrees, dec_degrees, issued_at=None):
        """Replace target/heartbeat without I/O; use local monotonic generation time."""
        return self._publish(ra_degrees, dec_degrees, issued_at, False)

    def stop(self):
        """Request stop, clear target, latch inactive. Not a physical E-stop."""
        with self._lock:
            if self._snapshot.mode not in (Lifecycle.NEW, Lifecycle.CLOSING, Lifecycle.CLOSED):
                self._snapshot = replace(self._snapshot, mode=Lifecycle.STOPPING, target=None, arrived=False)
        self._wake.set()

    def arm_pointing(self, frame, *, azimuth, elevation, issued_at=None):
        """Arm using fixed-root az/el degrees and an explicitly configured frame."""
        return self._publish(*frame.inverse(azimuth, elevation), issued_at, True)

    def set_pointing(self, frame, *, azimuth, elevation, issued_at=None):
        """Replace latest az/el target and heartbeat; no serial I/O or queue."""
        return self._publish(*frame.inverse(azimuth, elevation), issued_at, False)

    def close(self, timeout=3):
        with self._lock:
            mode = self._snapshot.mode
            if mode != Lifecycle.CLOSED:
                self._snapshot = replace(self._snapshot, target=None, arrived=False,
                                         mode=Lifecycle.CLOSED if mode == Lifecycle.NEW else Lifecycle.CLOSING)
        self._wake.set()
        if self._thread:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise ControlError("Worker did not stop; use physical E-stop")
        if self.snapshot.fault and "Stop transmission failed" in self.snapshot.fault:
            raise ControlError(self.snapshot.fault)

    def _halt(self, error=None):
        """One stop path for explicit stop, faults, and shutdown; fault survives stop."""
        with self._lock:
            current = self._snapshot
            self._snapshot = replace(current, target=None, arrived=False, fault=error or current.fault,
                                     mode=Lifecycle.CLOSING if current.mode == Lifecycle.CLOSING else Lifecycle.STOPPING)
        try:
            self.controller._stop()
        except Exception as exc:
            error = str(exc)
        with self._lock:
            current = self._snapshot
            fault = error or current.fault
            mode = Lifecycle.CLOSED if current.mode == Lifecycle.CLOSING else (
                Lifecycle.FAULT if fault else Lifecycle.DISARMED)
            self._snapshot = replace(current, mode=mode, target=None, applied=None, arrived=False, fault=fault)

    def _commit(self, target, state, deadline):
        """Dispatch boundary: cancel superseded decisions before serial writes."""
        with self._lock:
            current = self._snapshot
            if current.mode != Lifecycle.ARMED or current.target is not target:
                return False
            if monotonic() - target.issued_at >= self.heartbeat:
                raise ControlError("Heartbeat expired")
            if monotonic() >= deadline:
                raise ControlError("Continuous motion timed out")
            if monotonic() - state.started_at > self.controller.settings.max_sample_age:
                raise ControlError("State became stale")
            self._snapshot = replace(current, applied=target)
            return True

    def _loop(self):
        c, s = self.controller, self.controller.settings
        settled, deadline, last_angles, tick = 0, 0, None, monotonic()
        with c.mount._lock:
            try:
                while self.snapshot.mode not in (Lifecycle.CLOSING, Lifecycle.CLOSED):
                    self._wake.clear()
                    try:
                        if self.snapshot.mode == Lifecycle.STOPPING:
                            self._halt()
                        if self.snapshot.mode in (Lifecycle.CLOSING, Lifecycle.CLOSED):
                            break
                        state = c.read()
                        with self._lock:
                            current = replace(self._snapshot, state=state)
                            if current.mode == Lifecycle.ARMING:
                                if "N" not in state.status:
                                    raise ControlError("Mount must be stationary before arming")
                                current = replace(current, mode=Lifecycle.ARMED, fault=None)
                                settled, deadline, last_angles = 0, monotonic() + s.timeout, None
                            self._snapshot = current
                        if current.mode == Lifecycle.ARMED:
                            target = current.target
                            if target.angles != last_angles:
                                settled = 0
                            reached = c._step(target.angles, state, lambda: self._commit(target, state, deadline))
                            settled = settled + 1 if reached else 0
                            with self._lock:
                                if self._snapshot.mode == Lifecycle.ARMED and self._snapshot.target is target:
                                    arrived = settled >= s.settle_samples
                                    self._snapshot = replace(self._snapshot, arrived=arrived)
                                    if arrived:
                                        deadline = monotonic() + s.timeout
                            last_angles = target.angles
                    except Exception as exc:
                        if self.snapshot.mode != Lifecycle.FAULT:
                            self._halt(str(exc))
                    tick += s.period
                    if tick <= monotonic():
                        tick = monotonic() + s.period
                    self._wake.wait(max(0, tick - monotonic()))
            finally:
                with self._lock:
                    self._snapshot = replace(self._snapshot, mode=Lifecycle.CLOSING)
                self._halt()
