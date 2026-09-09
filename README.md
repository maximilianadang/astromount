# astromount

Small synchronous Python API for ZWO AM5/AM5N mounts. Two modules, one dependency
(`pyserial`), no background polling or service. A controller can import it directly.

## Install

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
```

On Ubuntu, creating a full virtual environment requires `python3-venv`.
Linux serial access requires membership in `dialout` (already configured here).
Use `/dev/serial/by-id/...` for a stable path across reconnects.
This workspace already has an installed `.venv`; use `.venv/bin/python` directly.

## Read position without moving

```python
from astromount import Mount

port = "/dev/serial/by-id/usb-ZWO_Systems_ZWO_Device_123456-if00"
with Mount(port) as mount:
    print(mount.identity())
    print(mount.position())
    print(mount.status())
    print(mount.tracking())
```

Opening/closing does not send commands. Empty firmware text becomes `None`.
Position contains `ra_hours`, `dec_degrees`, and monotonic `started_at` / `finished_at`
timestamps covering the two sequential queries. These are firmware coordinates,
not absolute encoder measurements or simultaneous joint-angle samples.

## Control

These calls **physically move the mount or change tracking**; they are examples,
not part of the read-only check above:

```python
mount.goto(ra_hours=5.5, dec_degrees=22.0)
mount.guide("north", duration_ms=100)
mount.move("east", rate=0)
mount.stop()
mount.set_tracking(False)
mount.home()
```

`goto` takes sky coordinates, requires the mount's time/site/alignment to be set,
and may enable tracking on arrival. It does not accept pitch/yaw or raw motor
angles. Use the horizon mapping below for ground-relative setpoints.
`move` selects one shared firmware speed index (0..9), then starts continuous
directional motion. `guide` uses the existing guide-rate setting and returns
without waiting for the pulse to finish. The external controller schedules pulses,
avoids overlap, and supplies independent feedback. Direction names refer to mount
directions, not calibrated image axes.

`stop` aborts slewing; disabling tracking is a separate operation. Closing a port
does not stop the mount. `home` also causes motion. Calls without firmware replies
confirm transmission only. Go-to acknowledgment means accepted, not arrived;
poll `status()` (`N` means not slewing, `H` means home; `G`/`Z` identify EQ/alt-az).

All operations share one lock; multi-command go-tos cannot interleave. Serial
access is exclusive among cooperating clients. Stop can wait behind an active
transaction, so this is not a hard-real-time emergency-stop mechanism. Timeouts,
invalid frames and transport errors close the connection; reconnect explicitly.
No command is automatically retried, since an unanswered motion command might
already have executed. A connection failure does not prove motion stopped.

## Bounded joint-position controller

`astromount_control` adds a synchronous proportional (P) velocity loop. Targets
are **absolute joint offsets from a saved reference**, in degrees—not celestial
RA/DEC, encoder readings, or calibrated world azimuth/elevation.

- Defaults: gain 0.5/s, speed cap 0.1°/s, requested sampling 10 Hz, deadband 0.02°.
- Excursion: ±22.5° per joint; stop boundary is ±22.25° with the default margin.
  Targets must be strictly inside ±22.23° to reserve deadband clearance.
- Reads combined firmware equatorial coordinates bracketed by sidereal time;
  reconstructs the nearest continuous mechanical branch relative to the baseline.
- Moves one axis at a time because firmware uses a shared speed setting. Stops
  before changing speed/direction; no go-to, homing, synchronization, or tracking setters.
- Requires EQ mode, tracking off, and a stationary start. Rejects stale reads,
  unexpected displacement, reversed motion, stalled progress, and reported faults.
- Returns after three stationary samples inside deadband. Timeout (300 s),
  cancellation, exceptions, and Ctrl-C attempt a stop, reconnecting if needed.

The deadband is a starting tolerance, not measured accuracy. Tune it using observed
readout resolution, jitter, and stopping behavior. Actual read frequency depends
on serial latency; transactions over 0.25 s abort by default.

After separately verifying the reference remains valid and calibrating command
polarity, the application code is:

```python
from astromount import Mount
from astromount_control import Controller, Reference

reference = Reference.from_baseline("baseline-2026-09-08T212838Z.json")
import json
from pathlib import Path
calibrated_directions = json.loads(Path("polarity-2026-09-08.json").read_text())["positive_directions"]
with Mount(port) as mount:
    control = Controller(mount, reference, positive_directions=calibrated_directions)
    print(control.read())                 # getters only
    for upper in (5, 0, -5, 0):           # RUNNING THIS COMMANDS MOTION
        control.run(ra_degrees=0, dec_degrees=upper)
```

`ra_degrees` means the lower joint's mechanical-model offset, and `dec_degrees`
the upper joint's offset. This sequence is relative to the **saved baseline**, not
whatever position the mount occupies when started. It may first reposition the
lower axis if that axis is not at baseline. Upper-axis rotation corresponds to
ground yaw only when its axis is vertical; arbitrary ground targets still require
the fixed-base rigid-body calibration. The older `mount.yaw()` below is a
different, astronomical mapping and is not the control interface for this setup.

For an external algorithm, call `run()` with successive absolute joint targets;
`on_sample(state)` supplies telemetry and `cancel=threading.Event()` allows
cancellation. Callbacks must return promptly and must not command the mount.
The controller owns the connection lock throughout a run; do not use another
client or thread to control motion. There is no integral term or application-side
acceleration ramp; this does not imply instantaneous physical starts/stops.

Local polarity commissioning on 2026-09-08 measured `("west", "south")` as
positive lower/upper directions, using separate approximately 0.092° jogs at
0.05°/s and reverse jogs back to the pre-test position. Stops were confirmed by
status and repeated position reads. This validates local jog polarity, not the
full P controller, the entire excursion envelope, or other pier-side branches.

**Further commissioning remains required:** saved coordinates are firmware-derived, not
independent absolute encoder measurements. Rebooting, syncing, homing, slipping,
or changing the manual axes can invalidate the reference. Branch reconstruction,
direction signs, and stop/restart behavior need controlled physical validation.
Software limits are not hard travel limits or collision protection: serial loss,
a blocked process, and stopping distance can defeat them. Keep the physical E-stop
available; a transmitted stop is not proof that the mount stopped.

## Streaming planner interface (no ROS dependency)

`Worker` runs the same controller step on an independent thread (requested 10 Hz).
It starts disarmed. `arm()` provides the first target; `set_target()` atomically
replaces both joint targets and refreshes a 0.5-second heartbeat. There are no
target or telemetry queues. Both methods return a sequence number without serial
I/O; `snapshot` returns immutable latest telemetry without waiting for the mount.

```python
from time import monotonic
from astromount_control import Worker

# Inside the existing Mount context, after constructing control:
with Worker(control, heartbeat=0.5) as worker:
    worker.arm(ra_degrees=0, dec_degrees=0)
    while planner_is_running():
        issued_at = monotonic()
        lower, upper = next_plan()  # Application planner, normally paced at 10 Hz.
        worker.set_target(ra_degrees=lower, dec_degrees=upper,
                          issued_at=issued_at)
        state = worker.snapshot
        # Consume state without blocking the controller worker.
```

This example commands physical motion when armed. All targets remain absolute
degree offsets from the unchanged saved reference. Use `Settings(max_speed=3,
margin=1.25)` on the underlying controller for the tested cap; other defaults are
unchanged. `run()` remains available for blocking, single-target applications;
do not call it or access the Mount/Controller while its Worker is running.

- The worker reads feedback, then takes the newest mailbox target. Missed ticks
  are skipped, not replayed. It rechecks target identity, lifecycle, heartbeat,
  and measurement age at dispatch, including after the stop before a changed jog.
  An update/stop before that boundary cancels the old decision. After commitment,
  one serial operation can still finish; commands already committed cannot be
  recalled. The mailbox lock is never held across serial I/O.
- Pass `issued_at` from **this process's monotonic clock**, captured when the
  planner starts generating the target. Old, duplicate/out-of-order timestamps,
  future timestamps, invalid angles, and expired leases are rejected without
  refreshing the heartbeat. Omitting it timestamps the call itself: use that only
  for directly generated targets, not delayed/queued work. One planner owns the
  stream; a future network adapter must handle source sessions and clock domains.
- Heartbeat loss or any control fault stops and latches disarmed. Target updates
  cannot restart motion; explicitly `arm()` again after resolving the cause.
  `snapshot.fault` reports failures. Faulted workers continue read attempts, but
  never automatically resume motion or clear the fault. Telemetry retains the
  last validated measurement and its timestamps if reads fail.
- `stop()` immediately clears the mailbox and requests a worker stop, asynchronously.
  Re-arm is rejected while that stop request is pending. It is not a physical
  E-stop; use telemetry to inspect stopped state. No automatic resumption.
- Arrival stops motion after the usual stationary settling checks, but the worker
  remains armed and monitors for drift/new targets. Keep publishing even an
  unchanged target to maintain the heartbeat.
- Updates do not reset progress monitoring. The existing 300-second timeout bounds
  continuous movement without settled arrival, even with changing targets; settled
  arrival renews that timer. Axis/sign changes reset only the direction-check
  origin, not the no-progress clock; measured joint displacement renews that clock.
- Context exit requests stop and joins the worker before the caller closes Mount.
  `close(timeout=3)` raises if it cannot join in time or final stop transmission
  fails. Shutdown is best effort, not a guarantee against process/USB failures.

### Lifecycle contract

`snapshot.mode` is the **single authoritative lifecycle state**. There are no
separate armed/active/pending-stop flags. `snapshot.armed` is derived from mode.

| Current mode | Event | Result |
|---|---|---|
| `new` | `start()` / context entry | `disarmed`; begin monitoring, no motion |
| `disarmed` or `fault` | `arm(fresh_target)` | `arming`; stationary preflight required |
| `arming` | Valid stationary feedback | `armed`; clear prior fault, execute latest target |
| `arming` or `armed` | `set_target(fresh_target)` | Replace mailbox, preserve lifecycle and progress timer |
| `armed` | Settled arrival | Remain `armed`; stop motors, keep monitoring/heartbeat |
| Any running mode | `stop()` | `stopping`; clear target, reject arm/update until stop processed |
| `stopping` | Stop transmission succeeds | `disarmed`, or `fault` if a fault was already latched |
| Any running non-fault mode | Read/control/heartbeat failure | `stopping` → `fault`; clear target, attempt stop |
| `fault` | Valid feedback or ordinary target update | Stay `fault`; update telemetry, reject target update |
| Any started mode | `close()` | `closing` → `closed`; attempt stop and join |
| `new` | `close()` | `closed`, no I/O |
| `closing` or `closed` | Arm/update/start | Rejected; worker cannot restart |

Repeated stop/close requests are safe. Stop does not erase a fault. A failed
preflight returns to `fault`; `arm()` accepting a request is not proof of readiness.
`closed` means the worker has exited, **not** proof of physical stopping; stop
transmission failures remain visible and are raised by `close()`.

Telemetry distinguishes `target` (latest requested target) from `applied` (target
of the last committed control decision), each with its own sequence and timestamp.
`state` is the last validated measurement. `arrived` refers to the requested target;
changing its angles clears it, while an unchanged heartbeat preserves it. These
fields are read atomically, without pretending a new target has already executed.

Heartbeat expiry is checked by the same serial worker, not an independent hardware
watchdog. Blocking I/O can delay it. At 3°/s a 0.5-second lease permits approximately
1.5° travel before expiry, plus scheduling/communication/stopping delays.

The streaming interface is tested offline with real threads and a simulated mount;
the earlier physical tests validated the shared blocking P loop, not this new
mailbox/heartbeat lifecycle. No hardware was accessed during this refactor.
Transition tests deliberately pause reads, control steps, and stop writes to
exercise replacement, expiry, stop, and shutdown interleavings. They also cover
fault monitoring/re-arm, no-progress across reversals, missed ticks, and failed
stop transmission. These tests cover those cases, not every possible scheduling
interleaving or physical/transport failure.

## Horizon targets and relative yaw

`PointingFrame` converts between RA/DEC and geometric azimuth/altitude using one
reversible rotation, with no extra dependency or hardware access:

```python
from astromount import PointingFrame

# Illustrative values, NOT this mount's measured setup:
frame = PointingFrame(latitude_degrees=40, sidereal_hours=6)
ra, dec = frame.equatorial(azimuth_degrees=90, altitude_degrees=30)
az, alt = frame.horizontal(ra_hours=ra, dec_degrees=dec)
```

The frame requires correct polar alignment, a calibrated mount pointing model,
and local sidereal time consistent with the mount at the sample/target epoch.
Sidereal time is NOT your clock's local time. Supply a fresh frame for each
operation; it does not advance automatically. Latitude alone cannot calibrate an
arbitrarily tilted base, home offset, or camera mounting error. Refraction is omitted.

With an open `mount`, these methods **send motion commands**:

```python
mount.goto_horizontal(frame, azimuth_degrees=90, altitude_degrees=30)
mount.yaw(+5, frame=frame)
mount.yaw(-5, frame=frame)
```

These are alternative examples, not a sequence to run immediately: each go-to
returns before arrival. `yaw` reads the reported position, adds degrees to azimuth,
preserves altitude, and reuses `goto_horizontal` / `goto`. Positive yaw increases
azimuth (north -> east); azimuth wraps at 360 degrees. The entire read/target
operation is locked. It requires equatorial mode with no active slew. Nonfinite
inputs, out-of-range latitudes/altitudes, and undefined output angles at coordinate
poles raise errors. Zero/full-turn yaw is a no-op.

These are **position setpoints at the frame's epoch**, not timed motor jogs, joint
angles, or a continuous ground hold. RA/DEC sky targets move relative to the ground
as time passes (Earth rotates about 0.0042 degrees per second); there can be drift
during the slew, and firmware may enable tracking afterward. For precise fixed
ground pointing, an external controller must refresh the mapping, verify arrival,
and manage tracking/corrections. No physical pointing accuracy or collision-free
path is inferred from a successful acknowledgment.

## Verification

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Tests use a pseudo-terminal mount emulator, including failure responses, timeouts,
coordinate validation, and concurrent command framing. Offline mapping tests cover
known sky directions, both hemispheres, angle wrapping, singularities, and 200
round trips. Simulated +5/-5 degree yaw verifies target command signs. Tests never
open the physical mount; horizon targeting has only been tested offline.
Controller tests additionally exercise the +5/0/-5/0 joint sequence, branch
crossings, bounded speed, invalid targets, stale feedback, reversed motion,
stalls, cancellation, and exception cleanup. No physical hardware was accessed
while implementing or testing this controller. Live control is not yet validated.
Earlier read-only hardware verification identified `AM5N`, firmware `1.6.3`;
the firmware getter is `:GV#` (corrected from `:GVN#`).

Protocol references: [INDI ZWO driver](https://github.com/indilib/indi/blob/master/drivers/telescope/lx200am5.cpp)
and [LX200 command implementation](https://github.com/indilib/indi/blob/master/drivers/telescope/lx200driver.cpp).
