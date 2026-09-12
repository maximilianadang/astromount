# AM5N robot-control investigation — 2026-09-08

## Conclusion

There is no fundamental need for a star field or an additional encoder to build
relative robot control. The controller exposes computed equatorial position,
its own sidereal clock, a pointing-side indicator, and independent directional
axis control. A mechanical-coordinate adapter and fixed mounting transform are
needed.

The photograph shows an approximately horizontal lower/RA axis and a vertical
upper/DEC axis. With the lower axis fixed there, upper-axis rotation changes
ground azimuth. This is a statement about this posture, not every posture.

No motion, tracking changes, homing, sync, mode changes, time/site changes, or
other mount configuration writes were performed in this investigation.
Application code was not changed. The calculations below were checked offline.

## Measured state

Host timestamp: 2026-09-08 21:11:11 UTC. Serial queries through the existing
library took approximately 1–7 ms each in this snapshot (not a worst-case bound).

| Query | Actual response, excluding trailing `#` |
|---|---|
| `:GVP#` model | `AM5N` |
| `:GV#` firmware | `1.6.3` |
| `:GU#` state | `nNG000000000` |
| `:GAT#` tracking | `0` |
| `:GMEQ#` RA/DEC | `09:31:44&+90*00:00` |
| `:GS#` sidereal time | `03:31:44` |
| `:Gm#` pointing side | `N` |
| `:Gt#` configured latitude | `+31*15:41` |
| `:Gg#` configured longitude, west-positive convention | `-120*43:11` |
| `:GC#` date | `01/01/00` |
| `:GL#` local clock | `03:31:09` |
| `:GG#` UTC offset | `+00:00` |
| `:GMZA#` firmware azimuth/altitude | `180*00:00&+31*15:41` |
| `:GTa#` meridian settings | `00+00` |

At 21:12:25 UTC, RA/DEC was `09:32:59&+90*00:00`, sidereal time was
`03:32:59`, status was unchanged, and tracking remained off. Thus the sky hour
angle `LST - RA` was exactly **-6 hours / -90 degrees** at both observations.
RA's 75-second advance followed the firmware clock, not evidence of motor motion.

Additional getters: altitude limits disabled (`:GLC#` = `0`), stored upper/lower
limits 90/0 degrees (`:GLH#`, `:GLL#`). A documented newer homing-history getter
`:Gh#` returned `#` as its first byte, not the specified 0/1. It supplies no usable
homing-history evidence on this firmware. `:Gm# = N` is a zero/indeterminate-side
state, while `:GU#` lacks the `H` home flag; do not equate DEC +90 with verified
physical homing. No unknown register commands were probed.

The date is not current. The configured geographic latitude also cannot stand in
for the physical lower-axis elevation in this robot. The firmware horizon output
is consequently not a measured room direction. Its 180-degree azimuth at DEC +90
also should not be assumed to use our north-zero convention without verification.
These issues invalidate blindly using its horizon numbers as robot coordinates;
they do NOT prevent taking differences using the controller's own clock.

## Mechanical reconstruction

In an ideal equatorial model, let `H = wrap180(15 * (LST_hours - RA_hours))`.
Sky DEC folds at the poles; mechanical DEC does not. The two equivalent
mechanical-angle candidates in the standard convention are:

```
normal:       (H_mech, D_mech) = (H, DEC)
beyond pole:  (H_mech, D_mech) = (wrap180(H - 180), wrap180(180 - DEC))
```

These are model angles, not directly verified AM5N shaft-zero/sign conventions.
Use an established reference, branch information, and temporal continuity to
unwrap the candidate nearest the previous joint state. ZWO's E/W indicator must
be validated against the standard convention before using it as an unconditional
branch selector. At the pole, sky direction alone does not constrain lower-axis
orientation; a retained mechanical reference matters.

An ideal example anchored at mechanical `(H, D) = (-90, 90)` and the first sampled
LST illustrates why adding/subtracting sky DEC alone fails:

| Upper model-angle offset | Mechanical H/D | Sky RA | Sky DEC |
|---|---|---|---|
| -5 degrees | -90 / 85 degrees | 09:31:44 | +85 degrees |
| 0 | -90 / 90 degrees | 09:31:44* | +90 degrees |
| +5 degrees | -90 / 95 degrees | 21:31:44 | +85 degrees |

*At the pole RA is geometrically arbitrary; this is the observed/reference
convention. These are mathematical examples, NOT commands approved for execution.
Physical positive yaw may correspond to the opposite model-angle sign.

Offline checks confirmed that both branch representations produce the same
pointing vector, and nearest-branch selection recovers the fixed lower axis and
upper displacements -5/0/+5. This validates the mathematics, not the firmware's
go-to branch selection or absolute mechanical initialization.

An arbitrary current pose can be the host's local joint zero for a relative
excursion. Automatic homing is not inherently required for that experiment.
However, the adapter must verify that subsequent telemetry follows commanded
displacement and retain branch continuity; rebooting invalidates that assumption.
Absolute room pose additionally needs a measured base/plate reference.

## Rigid-body model

Use the fixed base configuration and the two physical axis lines, referenced at
the chosen joint-zero posture. A compact, general representation is

```
T_ground_plate(q) = exp([S1] q1) exp([S2] q2) M
```

`S1` and `S2` describe the axis directions and pivot positions at zero; `M` is
the top-plate pose at zero. These are constant calibration data. `q1` and `q2`
are signed output-axis displacements from that reference. This model naturally
tilts the upper axis when the lower motor moves. At the pictured reference,
holding q1 fixed makes q2 a rotation about approximately ground vertical.
Axis distances matter for plate translation; directions and zero orientation
are enough for orientation alone. The telescope transform can be appended later.

The firmware adapter supplies q. Forward/inverse kinematics operate on q,
not directly on RA. Time and geographic site do not belong inside this rigid
mechanical model. Firmware sidereal time belongs only in its coordinate adapter.
Unmeasured axis directions, zero offsets, and plate frame orientation must not
be silently filled with exact values inferred from a perspective photo.

## How to execute a constrained excursion later

1. Capture a stationary baseline with tracking off. Collect RA/DEC together
   using `:GMEQ#`, bracket with `:GS#`, and preserve branch/reference information.
   Readbacks are step-derived estimates, not independent angle measurements.
2. Solve the ground-yaw target into joint offsets with the fixed geometry.
   In the pictured posture the lower offset should be zero.
3. For an upper-only path, use DEC directional control (`:Mn#` / `:Ms#`) and
   explicit axis stopping, with feedback, bounded speed, a stopping margin, and
   a timeout. Keep lower-axis motion and tracking disabled. Generic RA/DEC
   go-to leaves path/branch selection to firmware and can enable tracking.
4. Anchor all targets to the SAME baseline: 0 -> +5 -> 0 -> -5 -> 0. Do not
   accumulate measured endpoint errors as new zero positions.
5. Before executing that sequence, physically validate polarity, feedback around
   the pole, stopping latency, and model error with a separately authorized small
   motion. A software threshold at exactly 5 degrees cannot guarantee a physical
   maximum of 5 degrees. A link failure during continuous jogging also leaves
   stopping unconfirmed; a bounded-duration primitive would reduce that risk,
   but its motion behavior on this firmware has not been tested.

The public ZWO protocols inspected (v2.0, v2.1 and the driver supplement) document
no direct signed raw-axis-angle target or explicit go-to branch-selection command.
This is not a claim that no private interface exists. Independent axis motion is
implemented by the published driver. No new hardware is established as necessary;
the remaining limitation under the no-motion constraint is physical validation.

## Historical API issues observed during this investigation

- `identity()` uses `:GVN#`; the correct ZWO firmware query is `:GV#` (verified).
- Joint reconstruction needs sidereal time, branch/reference handling, and
  preferably the combined RA/DEC getter; these are not exposed publicly yet.
- There is no feedback-based joint target executor or arrival verification.
- The guide-duration validator allows 9999 ms; the examined ZWO protocol specifies
  0–3000 ms. Do not rely on the generic LX200 limit for this mount.
- Configuration-setting errors can be `eN#`; `_accept()` currently reads only
  one byte. Full error-frame handling needs correction before robust motion use.

These are identified changes, not changes applied by this diagnostic turn.

## Simultaneous variable-rate investigation (2026-09-10; no device commands)

Spiral testing is paused. Separate axis motion is not proof of independent rates.
ZWO protocol v2.1 documents one `:Rvnnnn.nn#` rate selector, separate `:Mn/s/e/w#`
starts and `:Qn/s/e/w#` stops, but does not specify per-axis rate retention or
unaffected-axis behavior during rate changes. INDIGO implements separate axis
start/stop with a common preset rate; it does not establish unequal concurrent rates.

In [ZWO's manufacturer response](https://bbs.zwoastro.com/d/15709-different-slew-rates),
ASIMount@ZWO states that RA and DEC must use the same rate at the time of that
reply. This is evidence against assuming support, not a version-specific test
of our recorded firmware 1.6.3. ASCOM MoveAxis semantics alone prove nothing about
this firmware implementation.

Our saved live-rate test measured only DEC: changing Rv alone left its active
speed unchanged. That suggests possible rate retention, but never tested starting
the other axis at a different rate, axis-local stops, or cross-axis interference.
Independent simultaneous velocities therefore remain unverified and are not a
documented capability we can enable in the controller yet.

Next discriminating experiment, requiring separate supervised motion approval:
start one axis slowly, start the other at a different slow rate, measure both,
stop only one and verify the other continues unchanged; repeat with axis order
reversed. Only after that passes, test rate increase/decrease and reversals while
the other axis continues. Keep a small measured excursion and global-stop cleanup.
If supported, replace the single selected action with a two-axis velocity vector,
axis-local dispatch and per-axis progress checks, preserving the mailbox/heartbeat,
FK/IK and global fault stop. Do not substitute native goto or upgrade firmware.

### Supervised simultaneous-rate result (2026-09-10)

Executed `probe_simultaneous.py` on AM5N firmware 1.6.3; full samples in
`simultaneous-rates-2026-09-10.jsonl`. No goto, home, sync or tracking changes.
Contrary to the conservative documentation-based assumption above, this firmware
retains different active rates on both axes in this tested sequence:

| Sequence | Lower measured deg/s | Upper measured deg/s |
| --- | ---: | ---: |
| West at .05, alone | +.05044 | 0 |
| Add south at .10 | +.05048 | +.09708 |
| Stop west only | 0 | +.09743 |
| North at .05, alone | 0 | -.04992 |
| Add east at .10 | -.09726 | -.05006 |
| Stop north only | -.09754 | 0 |

Each phase lasted approximately 1.5 seconds; rates use position differences after
the first .35 seconds. Global stop and three stationary samples confirmed after
each trial. Initial baseline-relative joints (+.01667,+.01972); final joints
(-.12917,+.16722) degrees. Not returned to baseline. Maximum observed displacement
from the test origin was less than .31 degrees on either axis, inside the .6-degree
guard. These are firmware-derived measurements, not independent optical metrology.

This validates unequal simultaneous low-speed motion and axis-local stopping in
both start orders, not live rate updates, reversals with another active axis, or
high-speed trajectory tracking. Controller remains unchanged and spiral paused.

### Axis-local rate update result (2026-09-10)

Ran `probe_simultaneous.py --rate-updates`; samples saved in
`simultaneous-rate-updates-2026-09-10.jsonl`. Each trial starts both axes at .05
deg/s, then changes only the first axis to .10 and .025 using its direction-local
Q followed immediately by Rv/M. No global stop between phases; no reversals tested.

| Changing axis / phase | Lower measured deg/s | Upper measured deg/s |
| --- | ---: | ---: |
| Lower / .05 | +.05020 | +.04995 |
| Lower / .10 | +.09673 | +.05004 |
| Lower / .025 | +.02319 | +.04999 |
| Upper / .05 | -.05048 | -.04996 |
| Upper / .10 | -.04678 | -.09719 |
| Upper / .025 | -.04655 | -.02509 |

Supports increasing/decreasing one axis with local stop/restart while the partner
continues near .05 deg/s. Short-window lower-axis estimates are quantized; these
are not precision rate measurements or proof of transient-free motion. Both trials
ended with global stop and three stationary samples. Final baseline-relative joints
(-.09583,+.13417) degrees; not returned to zero. Controller still unchanged; spiral
paused. Continuous 10 Hz two-axis updates and in-motion reversals remain untested.

### Adapted combined pointing controller test (2026-09-10)

Executed `probe_pointing.py`, calling the production `Controller.run_pointing`
with `Pointing(1,-1)`, root az/el (+1,+1), .1 deg/s per-axis cap, 30 s timeout,
and a reduced joint excursion of 2 degrees (1.75-degree stopping boundary).
Log: `combined-pointing-2026-09-10.jsonl`. Start joints (-.09583,+.13417).
IK target joints (+1.0001523,-.9998477); final joints (+.9833333,-.9808333).
FK-estimated final az/el (+.9809778,+.9831892). Final joint errors
(-.016819,+.019014) are inside the configured .02-degree deadband.

154 samples at mean 10.002 Hz; longest read .02043 s. Both motors commanded
concurrently in 143 sampled intervals; measured joint rates during seconds 1–8
were approximately (+.09723,-.09723) deg/s. Completion and post-stop verification
took 15.82 s. Three additional reads agreed on stationary final pose, tracking off.
No return to baseline and no spiral. Near arrival the lower readout alternated
across the deadband and triggered short corrections before settling; this is not
evidence of jitter-free tracking. Firmware-derived FK is not independent optical
verification. This validates one low-speed combined fixed target, not streamed
trajectory replacement, high-speed motion, or reversal while the partner moves.

### First streamed spiral test: stopped on fault (2026-09-10)

User approved the displayed baseline-centered 5-degree cone, two turns outward
and two inward over 120 s. Ran production CLI through `probe_spiral.py` with
1 deg/s per-axis cap and existing worker/heartbeat; full accepted telemetry in
`spiral-2026-09-10.jsonl`. Preflight stationary, tracking off, baseline-relative
joints (+.01667,-.01972). Offline suite: 64 tests passing before execution.

Run aborted around 78 seconds with `Unexpected axis displacement or coordinate
discontinuity`. There were 779 accepted samples at 9.9999 Hz, max read latency
.01839 s and maximum recorded cone radius 4.55397 degrees. Motion showed lag
against the requested spiral. The outward leg and part of the inward leg ran;
the trajectory did not complete and no automatic return/retry was performed.

Worker shutdown attempted global stop. A separate read-only check confirmed five
stationary status samples (`nNG000000870`), tracking off; final joint readout
approximately (-.445833,+3.380278), FK az/el (-3.380380,-.445058) degrees.
Lower readout varied one .004167-degree quantum during that check.

The rejected sample and per-axis command state were not captured by the logging
wrapper (which records only successful reads). Exact cause is unresolved; do not
claim an overspeed, a branch jump, or a false guard trigger from this evidence.
Do not relax the guard or resume the spiral without investigating the failure.

### Displacement interval accounting fixed offline (2026-09-11)

The guard now retains per-axis commanded-motion history since the previous
accepted sample instead of consulting only the latest velocity command. Starts
mark the interval active; local/global stops preserve that history. Successful
reads consume the interval and seed the next from currently commanded motion.
The existing speed-cap allowance is unchanged; wholly idle intervals still use
the deadband. Fault messages now include axis, previous/measured positions, bound,
interval-motion state, current command, elapsed time and firmware status.

Regression tests cover cancellation between local stop/restart on either axis,
start/stop entirely between reads, global stop, return to strict idle checks,
bounded rejection and preservation of history on a rejected read. All 71 offline
tests pass. No hardware commands sent. The original spiral trigger remains an
inference because its rejected sample was not captured; physical validation of
this fix and investigation of the later CLI timeout are still outstanding.

## Sources

- [ZWO protocol v2.1, manufacturer document hosted by INDIGO](https://github.com/indigo-astronomy/indigo/blob/master/indigo_drivers/mount_asi/docs/ZWO_Mount_communication_protocol_v2.1.pdf)
- [ZWO protocol v2.0](https://github.com/indigo-astronomy/indigo/blob/master/indigo_drivers/mount_asi/docs/ZWO_Mount_communication_protocol_v2.0.pdf)
- [INDIGO ZWO driver: firmware getter and separate DEC/RA motion](https://github.com/indigo-astronomy/indigo/blob/master/indigo_drivers/mount_asi/indigo_mount_asi.c)
- [ASCOM explanation of mechanical versus sky coordinates and pointing branches](https://www.ascom-standards.org/newdocs/ptgstate-faq.html)
- [Official AM5N manual](https://i.zwoastro.com/wp-content/uploads/2026/03/b063feaca7a20b4c23090773c351e501.pdf)

## Subsequent offline go-to review

This review used documentation, public driver source and previously saved
measurements only. No serial connection was opened and no mount commands sent.

The INDIGO driver's `asi_slew()` sets target RA with `:Sr...#`, sets target DEC
with `:Sd...#`, then dispatches `:MS#`. It checks acknowledgments but supplies no
mechanical branch, axis lock, excursion bound or trajectory. In particular,
the public driver is not the source of the firmware's motor-path algorithm.
The protocol does not specify a shortest-joint-path guarantee or provide a
documented preview of the selected motor trajectory. The internal branch-selection
implementation for firmware 1.6.3 was not located in the public sources reviewed.
Therefore a local simulation of our desired path cannot validate firmware behavior.

The protocol describes meridian settings in terms of tracking across the meridian.
Disabling automatic meridian flipping must not be treated as a documented promise
that every subsequent go-to preserves the current mechanical branch. It also
documents tracking being enabled after a successful go-to. Keeping an RA sky
coordinate constant is not equivalent to keeping the lower joint stationary as
the clock advances.

The latest user-movement snapshot (`measurement-2026-09-08T213511Z.json`) differs
materially from the original pole-centered baseline: sky DEC is +77:34:32,
hour angle is about -90.0083 degrees, side W, tracking off. In the nearest ideal
mechanical branch, upper-axis offsets -5/0/+5 about THAT saved pose correspond to
DEC +72:34:32 / +77:34:32 / +82:34:32. This entire interval stays at least
7.4244 degrees from the north coordinate pole. The original reference at +90
does cross the pole for a two-sided excursion; the latest reference does not.
The calculation was checked offline. It is conditional on the fixed geometry and
branch reconstruction and does not establish firmware path selection, starting
state now, or direction polarity in the room.

The evidence supports ordinary small destination changes as a plausible use of
the built-in positioning controller. It does not yet establish the requested
strict upper-only, maximum-5-degree travel contract. A separate feedback executor
is a possible alternative, not an established requirement for all position control.

The precise unresolved vendor question is: on AM5N firmware 1.6.3 in equatorial
mode, how does `:MS#` select its mechanical branch after manual jogging, and does
it guarantee preservation of that branch for nearby targets on the same side of
the meridian/pole? Is there a documented axis-angle target, branch lock or trajectory
preview? No question or message was sent to ZWO or anyone else.
