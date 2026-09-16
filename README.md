# astromount

Small synchronous Python API for ZWO AM5/AM5N mounts. Three modules, one dependency
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

### Command-line pointing

Edit `config.json` for installation defaults: `port`, `baseline`, `polarity`,
`pitch_sign`, and `yaw_sign`. `astromount_config.py` is only their shared loader;
relative file paths resolve against the config directory, not the working directory.
All tools use these defaults where relevant, including every `motion_tests.py`
subcommand. Explicit CLI overrides take precedence. `zero_mount.py` uses the port
but creates a baseline rather than reading one, and does not update the config.
Changes take effect on the next script run; missing/invalid settings fail rather
than silently falling back to historical defaults.
Controller settings have one shared source: this repository's `motion-settings.json`,
under `motion`. Point, sequence, Python controller/worker, hardware tests, and LiDAR
motion all use its gain, speed, deadband, sampling/timeout,
settling, and heartbeat defaults. Explicit CLI overrides take precedence.
`max_speed` is the default rate for untimed moves; `speed_limit` caps duration-based
moves and explicit speeds, and cannot exceed the tested 3°/s ceiling.
Hardware test procedures retain explicit reduced rates/envelopes and watchdogs;
these do not replace the shared operational defaults.
Live reads and saved references share one coordinate decoder and clock-bracket
validation. Baseline capture uses the same raw coordinate sampling method.

To capture a replacement baseline, first physically restore the agreed zero pose
(telescope forward, upper axis vertical, fixed manual settings unchanged), then:

```bash
.venv/bin/python zero_mount.py baseline-new.json
```

This performs only getters and saves three stationary, tracking-off samples. It
refuses an existing output file and does not home, sync, move, change motor signs,
or replace the default baseline. The script cannot verify physical pose or the
mechanical branch/sign calibration. A power cycle can invalidate the reference;
capturing an arbitrary orientation does not recalibrate the FRD geometry.
Explicitly select the new reference on subsequent commands with
`point.py --baseline baseline-new.json ...`. Capture does not resolve the reported
return timeout or validate the displacement-guard fix; it is not permission to
resume the spiral.

From this project directory:

```bash
.venv/bin/python point.py 0 0
.venv/bin/python point.py 5 2 --speed 1
.venv/bin/python point.py 5 2 --delta --duration 10
.venv/bin/python point.py --help
```

Arguments are fixed-root FRD **azimuth, elevation in degrees**, relative to the
original saved baseline. Positive azimuth turns right; positive elevation raises
the nose. These commands move physical hardware (except `--help`). The CLI uses
the observed signs `pitch_sign=+1`, `yaw_sign=-1`, converts the target with IK,
and calls `control.run_pointing(frame, azimuth=..., elevation=...)`. Both joints
are driven concurrently whenever both have position error. No planner heartbeat is
required for a blocking single target. Default speed cap is 1°/s (maximum 3); timeout is
30 seconds. There is no software excursion limit or stopping margin; supervise
motion visually with an accessible E-stop. Speed, feedback and timeout checks remain.

`--delta` adds the supplied az/el to the current measured root-frame az/el, not
to joint angles. For point moves, `--duration SECONDS` replaces `--speed`: the
common joint-speed cap is the largest IK joint displacement divided by duration,
capped at 3°/s. This is nominal timing, not a timed trajectory: proportional
slowdown, settling, and the cap can extend travel time. The arrival timeout becomes
duration plus `--timeout`. Zero-distance moves only settle; unrepresentably slow
rates are rejected. These options require a stationary live read and cannot use
`--dry-run`. Spiral `--duration` and `--speed` retain their existing meanings.

For sequential absolute waypoints, use `sequence.py --path waypoints.csv`:

```csv
az,el,duration
5,0,5
5,5,5
0,0,10
```

The entire CSV is checked for valid targets, safety boundaries, and positive finite
durations before opening the port. Each waypoint uses the same duration-based
point controller and waits for arrival/settling before advancing. One connection
is held throughout; an error or Ctrl-C ends the sequence, with active motion using
the controller's existing stop-on-exit behavior. Completion prints one JSON record
per waypoint. `--timeout` applies per waypoint in addition to its duration.
`--port`, `--baseline`, and `--polarity` work as in `point.py`.
`sequence.py --path waypoints.csv --dry-run` validates without opening the device;
live-dependent speed feasibility is checked before each move.

Add `--delta` to interpret every row as an az/el displacement from the measured
position at the start of that row (after the previous move settles). The CSV is
checked for finite values and positive durations upfront; each resulting absolute
target is checked against IK and safety limits before its move. Like `point.py`,
delta mode cannot use `--dry-run` because targets depend on live measurements.
Opposite deltas approximately retrace motion; use an absolute waypoint to return
to a fixed saved target without accumulating waypoint settling errors.

Baseline and polarity paths default relative to the script, not the shell directory.
Override with `--baseline`, `--polarity`, and `--port`. Completion prints joint
offsets, model-estimated az/el, and status; errors return a nonzero exit code.
Keep the E-stop available. The single-axis signs were observed physically;
the adapted controller reached one combined (+1°, +1°) target at .1°/s on hardware.
That verifies firmware-feedback convergence, not independent optical accuracy.

### Continuous CSV sweeps

`sweep.py` is a separate experimental alternative; `sequence.py` is unchanged.
Every non-dry-run sweep that opens the mount writes
`output/YYYYMMDDTHHMMSSffffffZ-sweep.jsonl` relative to the astromount repository.
The terminal prints its path. Each run uses a new file; rows are flushed as written.
Records include resolved input waypoints, baseline reference, frame signs, polarity,
settings, initial pointing, resolved timed segments, published az/el targets, and
worker feedback snapshots (measured az/el, joint coordinates, query timestamps,
latest/applied targets, lifecycle and fault). The final event records completion,
interruption, or failure after worker cleanup. A crash/power loss may leave a
partial file without a final event; flushing is not a power-loss durability guarantee.

Telemetry uses monotonic timestamps for alignment and Unix timestamps for wall
time. Snapshots are sampled by the planner at its configured period, not extra
hardware polls; repeated measurement timestamps denote the same measurement and
intermediate worker samples may be skipped. Compare the saved plan at the query
midpoint to measured pointing to estimate trajectory error; do not subtract a
newly published target from older feedback. Applied targets mark worker dispatch,
not firmware acknowledgements. This measures model feedback, not optical truth.

It accepts the same CSV (`az,el,duration`) and CLI options:

```bash
.venv/bin/python sweep.py --path waypoints.csv --dry-run
.venv/bin/python sweep.py --path waypoints.csv
.venv/bin/python sweep.py --path waypoints.csv --delta
```

It starts from the measured pointing and linearly interpolates root-frame az/el
over each row's duration, publishing at the configured `period` through the
existing worker/heartbeat. There is no intermediate arrival wait or command queue;
late ticks skip ahead in time. It holds the final target until settled, or faults
after the final `timeout`. One final JSON measurement is printed. Worker exit
attempts to stop on success, fault, or Ctrl-C.

Equal endpoint elevations give a constant-elevation commanded segment; if the
initial measured elevation differs, the first segment interpolates to it.
For continuous `--delta`, rows accumulate from the initial measured pointing and
previous **planned** endpoint, not from each lagging measured position. Zero
elevation deltas therefore keep the original commanded elevation. This deliberately
differs from stop-and-settle `sequence.py` delta semantics.

Durations schedule moving targets, not guaranteed physical arrival times. Joint
commands remain capped by `speed_limit`; the unchanged proportional controller
can lag, miss intermediate waypoints, and deviate in elevation. No feedforward or
constant-elevation tracking guarantee is added. Absolute dry-run validates CSV/IK
without opening hardware; actual-start validation happens before arming. Delta
dry-run remains unavailable. No automated hardware validation has been performed.

### Hardware motion tests

`motion_tests.py` replaces the five standalone commissioning/probe scripts. It shares
connection setup, preflight, telemetry, stationary verification, and raw-test timers.
`point.py`, `zero_mount.py`, and `measure.py` remain the everyday interfaces.

| Previous script | New command |
| --- | --- |
| `calibrate_jog.py west` | `motion_tests.py jog west` |
| `calibrate_jog.py south --live-rate-test --repeat-direction` | `motion_tests.py jog south --live-rate-test --repeat-direction` |
| `probe_simultaneous.py` | `motion_tests.py simultaneous` |
| `probe_simultaneous.py --rate-updates` | `motion_tests.py simultaneous --rate-updates` |
| `probe_pointing.py` | `motion_tests.py pointing` |
| `probe_spiral.py` | `motion_tests.py spiral` |
| `test_position_live.py out` / `back` | `motion_tests.py position out` / `back` |

All experiments command physical motion. `--help` is offline. Global options go
**before** the subcommand. To deliberately use the new physical baseline:

```bash
.venv/bin/python motion_tests.py --help
.venv/bin/python motion_tests.py --baseline baseline-2026-09-11-1032.json pointing
.venv/bin/python motion_tests.py --baseline baseline-2026-09-11-1032.json position out --record trial.json
.venv/bin/python motion_tests.py --baseline baseline-2026-09-11-1032.json position back --record trial.json
```

All experiments default to the baseline in `config.json`; use `--baseline` to
select a historical reference explicitly. Position tests retain `.1`/`1`
degree choices and separate out/back invocations. Existing outward records cannot
be overwritten; returns require a successful out, unchanged endpoint, and matching
degrees/reference when recorded. Logs now share one JSON telemetry format; saved
historical logs are unchanged. Raw tests retain short watchdogs, bounded rates,
global stop cleanup; all modes verify stationary exit. Local excursion guards are removed.
The raw jog additionally uses the shared overspeed check. No experiment was run
on hardware during this consolidation. The spiral fix still needs physical validation.

### Continuous az/el and spiral experiment

The existing worker also accepts `arm_pointing(frame, azimuth=..., elevation=...)`
and `set_pointing(frame, azimuth=..., elevation=..., issued_at=...)`, where
`frame = Pointing(pitch_sign=1, yaw_sign=-1)` for the observed setup. Arm once;
then publish the planner's latest target at approximately 10 Hz. `issued_at` is
local monotonic generation time (defaults to now). Each update replaces the
mailbox and renews the existing 0.5-second heartbeat. No new controller or queue.
Keep the worker context open for the session; leaving it attempts to stop.
`worker.snapshot.state.pointing(frame)` returns FK-estimated az/el when a state
is available. Blocking calls return the same State type. Neither interface adds
a per-motor thread; the controller sends serial commands, and firmware runs both motors.

```bash
# Offline: no serial access.
.venv/bin/python point.py 0 0 --spiral 5 --dry-run
# PAUSED pending combined-controller hardware validation. This commands motion:
.venv/bin/python point.py 0 0 --spiral 5 --duration 120 --speed 1
```

The spiral is a true 5-degree angular cone about baseline forward, restricted to
at most 5 degrees for this initial experiment. Targets are generated at 10 Hz
from elapsed time, skipping missed ticks. Both motors can run concurrently at
independent capped rates, with local stop/restart for changed rates. Tracking
may lag; this is not a guarantee of a smooth path or a hard measured cone bound.
Individual-axis directions have been observed; combined trajectory tracking has
not yet been validated on hardware. Keep the E-stop ready. The saved baseline
must still be valid, and the approach from the current pose must be unobstructed.
Existing freshness, progress and stop checks remain unchanged; the
continuous-motion timeout covers the requested duration plus two arrival timeouts.

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
angles. It is a separate low-level firmware operation, not used by the joint controller.
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
RA/DEC or encoder readings. `run_pointing` and the worker's pointing methods use
the configured FRD kinematics to convert root az/el to these joint targets;
`state.pointing(frame)` maps measured joints back through FK.

- Shared defaults: gain 0.5/s, speed cap 1°/s, requested sampling 10 Hz, deadband 0.01°.
- No software excursion boundary or stopping margin. Finite joint targets are
  accepted; az/el IK still selects only the front-facing, nonsingular branch.
- Reads combined firmware equatorial coordinates bracketed by sidereal time;
  reconstructs the nearest continuous mechanical branch relative to the baseline.
- Computes a capped P velocity for each joint on every tick. Stops/restarts only
  axes whose commanded velocity changes; unchanged axes continue. No go-to,
  homing, synchronization, or tracking setters. Arrival requires both joints.
- Requires EQ mode, tracking off, and a stationary start. Rejects stale reads,
  unexpected displacement, reversed motion, stalled progress, and reported faults.
  Displacement bounds account for commanded motion anywhere since the previous
  accepted sample, including motion before a stop/cancelled restart. After that
  interval is consumed, idle axes return to the deadband-sized bound. Stops do
  not erase interval history; rejected samples do not advance it. This is a
  conservative speed-cap bound, not a feedforward prediction of exact travel.
  Progress checks are per axis: one moving joint cannot conceal a stalled partner.
- Returns after three stationary samples inside deadband. Timeout (30 s),
  cancellation, exceptions, and Ctrl-C attempt a stop, reconnecting if needed.

Concurrent unequal low rates and axis-local rate changes were measured on AM5N
firmware 1.6.3. The adapted controller also reached root az/el (+1°, +1°) at .1°/s
with measured 10 Hz feedback and both joints inside deadband. Live reversals and
streamed trajectory updates still need physical validation. Independent P loops
do not promise simultaneous arrival or a straight
az/el path. The original 3°/s experiment was single-axis, not combined control.

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
the fixed-base geometry below. The controller itself accepts joint offsets;
the pointing layer converts root-frame az/el into those targets.

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

## Fixed-root FRD pointing kinematics

`astromount_kinematics.Pointing` is a pure, degree-based mapping for the agreed
ideal geometry: root +X front, +Y right, +Z down; lower joint axis lateral, upper
axis vertical at the original baseline; the chosen plate-forward vector is +X.
The original saved baseline remains zero. No new baseline is captured.

The ordered orientation is `Ry(pitch) Rz(yaw)`: the lower joint carries the upper
axis. Azimuth is positive toward right; elevation positive upward. This is
pointing-direction kinematics, not Cartesian position/full-pose IK. Link lengths
and the telescope-to-plate transform are deliberately excluded.

```python
from astromount_kinematics import Pointing

# Explicit NOMINAL sign assumption, not independently verified FRD polarity:
frame = Pointing(pitch_sign=1, yaw_sign=1)
az, el = frame.forward(lower=-0.0083333333, upper=-12.4244444444)
lower, upper = frame.inverse(azimuth=5, elevation=5)
# Inside an already armed worker, publishing this target commands motion:
worker.set_target(ra_degrees=lower, dec_degrees=upper)
```

`pitch_sign=+1` means increasing the measured lower offset raises the nose;
`yaw_sign=+1` means increasing the upper offset turns right at baseline. Use -1
for either reversed relationship. These required parameters are distinct from
the measured west/south command polarity. The physical FRD signs have not been
independently established; do not treat this example as that calibration.

The inverse is analytic and unique on the configured local branch (each joint
limited to less than 90°). Unreachable az/el targets and invalid inputs raise
`ValueError`; no numerical solver, queued trajectory, or firmware go-to is used.
The former ±22.5° geometric cap and controller stopping margin are removed.

Nominal (+1,+1) mapping of the saved manual captures, in degrees:

| Configuration | Lower offset | Upper offset | Root azimuth | Root elevation |
|---|---:|---:|---:|---:|
| Original baseline | 0 | 0 | 0 | 0 |
| First manual capture (clock-bracket midpoint) | -3.452083 | +0.001111 | +0.001113 | -3.452083 |
| Second manual capture | -0.008333 | -12.424444 | -12.424445 | -0.008138 |

These are model predictions for captured endpoints, not measured root-frame
angles or established mechanical travel limits. For the historically tested ±22.5° joint
box, lower-only endpoints are (az 0°, el ±22.5°), upper-only endpoints are
(az ±22.5°, el 0°), and the four corners are (az ±24.148675°, el ±20.704811°).
The boundary is curved, not a rectangular az/el box or an exact circular cone.

Nominal geometry source: [ZWO AM5N structural drawings, printed page 30](https://i.zwoastro.com/wp-content/uploads/2026/03/b063feaca7a20b4c23090773c351e501.pdf#page=31),
together with the installation posture described by the operator. Root alignment
and telescope pointing accuracy are not established by an algebraic round trip.
Tests cover independent rotation composition, all sign combinations, boundaries,
unreachable targets, and recovery of the saved joint offsets. This new layer has
not been tested on physical hardware.

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
degree offsets from the unchanged saved reference. Use `Settings(max_speed=3)`
on the underlying controller for the previously tested single-axis cap; other defaults are
unchanged. `run()` remains available for blocking, single-target applications;
do not call it or access the Mount/Controller while its Worker is running.

- The worker reads feedback, then takes the newest mailbox target. Missed ticks
  are skipped, not replayed. It rechecks target identity, lifecycle, heartbeat,
  and measurement age at dispatch, including between axes and after each local stop.
  An update/stop before that boundary cancels the old decision. After commitment,
  one serial operation can still finish; commands already committed cannot be
  recalled. `applied` identifies the target used at a dispatch boundary, not an
  atomic two-motor update or arrival acknowledgment. The mailbox lock is never
  held across serial I/O.
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

## Verification

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Tests use a pseudo-terminal mount emulator, including failure responses, timeouts,
coordinate validation, and concurrent command framing. Tests never open the
physical mount.
Controller tests additionally exercise the +5/0/-5/0 joint sequence, branch
crossings, bounded speed, invalid targets, stale feedback, reversed motion,
stalls, cancellation, and exception cleanup. No physical hardware was accessed
by the offline test suite. Separate supervised commissioning verified the blocking
joint controller, including a return to baseline with a 3°/s cap. The streaming
worker lifecycle has not yet been physically validated.
Earlier read-only hardware verification identified `AM5N`, firmware `1.6.3`;
the firmware getter is `:GV#` (corrected from `:GVN#`).

Protocol references: [INDI ZWO driver](https://github.com/indilib/indi/blob/master/drivers/telescope/lx200am5.cpp)
and [LX200 command implementation](https://github.com/indilib/indi/blob/master/drivers/telescope/lx200driver.cpp).
