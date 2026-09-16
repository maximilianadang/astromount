from dataclasses import replace
from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from astromount_control import Controller, ControlError, Reference, Settings
from astromount_kinematics import Pointing
from astromount import decode_coordinates, ProtocolError
from test_astromount import simulated


class Clock:
    def __init__(self):
        self.now = 0.0

    def sleep(self, seconds):
        self.now += seconds


class Plant:
    """Ideal step-derived feedback, with independent physical motion integration."""
    def __init__(self, clock):
        self.clock, self.last = clock, 0.0
        self.q, self.velocity = [0.0, 0.0], [0.0, 0.0]
        self._lock = RLock()
        self._serial = SimpleNamespace(is_open=True)
        self.commands = []
        self.polarity, self.frozen, self.flags = 1, False, "nG"

    def integrate(self):
        dt = self.clock.now - self.last
        self.last = self.clock.now
        self.q = [q + dt*v*self.polarity for q, v in zip(self.q, self.velocity)]

    def joint_sample(self):
        self.clock.sleep(0.005)
        self.integrate()
        h, d = (-90, 90) if self.frozen else (-90 + self.q[0], 90 + self.q[1])
        if d > 90:
            h, d = h+180, 180-d
        return (h+180)%360-180, d, self.flags + ("N" if not any(self.velocity) else "")

    def stop(self, direction=None):
        self.integrate()
        if direction is None:
            self.velocity = [0.0, 0.0]
        else:
            self.velocity[0 if direction in ('east', 'west') else 1] = 0.0
        self.commands.append(("stop",) if direction is None else ("stop", direction))

    def jog(self, direction, *, speed_degrees_s):
        self.integrate()
        axis = 0 if direction in ("east", "west") else 1
        assert not self.velocity[axis], "Changing axis must be stopped before restart"
        self.velocity[axis] = speed_degrees_s * (1 if direction in ("west", "north") else -1)
        self.commands.append((direction, speed_degrees_s))


class ControlTests(unittest.TestCase):
    def test_live_and_saved_coordinates_share_decoder(self):
        def queries(before='23:59:59', after='00:00:00', eq='06:00:00&+90*00:00'):
            return {k: {'response': v} for k,v in zip(('sidereal_before','sidereal_after','equatorial'), (before,after,eq))}
        data = queries()
        self.assertEqual(Reference.from_queries(data).angles, decode_coordinates(data))
        self.assertAlmostEqual(decode_coordinates(data)[0], -90-15/7200)
        for invalid in (queries(after='00:00:05'), queries(eq='invalid'), queries(eq='24:00:00&+90*00:00')):
            for decode in (decode_coordinates, Reference.from_queries):
                with self.assertRaises(ProtocolError):
                    decode(invalid)

    def setUp(self):
        self.clock = Clock()
        self.patches = [patch("astromount_control.monotonic", lambda: self.clock.now),
                        patch("astromount_control.sleep", self.clock.sleep)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.plant = Plant(self.clock)
        self.controller = Controller(self.plant, Reference(-90, 90), positive_directions=("west", "north"))

    def test_sequence_crosses_pole_and_returns_to_same_zero(self):
        for dec in (5, 0, -5, 0):
            state = self.controller.run(ra_degrees=0, dec_degrees=dec)
            self.assertLessEqual(abs(state.dec_degrees-dec), self.controller.settings.deadband)
            self.assertEqual(state.ra_degrees, 0)
        self.assertFalse(any(self.plant.velocity))
        self.assertTrue(all(c[1] <= self.controller.settings.max_speed for c in self.plant.commands if c[0] != 'stop'))

    def test_two_axis_target(self):
        state = self.controller.run(ra_degrees=1, dec_degrees=-2)
        for actual, target in zip(state.angles, (1, -2)):
            self.assertLessEqual(abs(actual-target), self.controller.settings.deadband)

    def test_combined_azel_converges_with_overlapping_motion(self):
        frame, velocities = Pointing(1, -1), []
        state = self.controller.run_pointing(frame, azimuth=2, elevation=1,
                                            on_sample=lambda s: velocities.append(tuple(self.plant.velocity)))
        self.assertTrue(any(all(v) for v in velocities))
        for actual, target in zip(state.pointing(frame), (2, 1)):
            self.assertAlmostEqual(actual, target, delta=.04)
        self.assertEqual(self.plant.velocity, [0, 0])

    def test_axis_local_updates_preserve_other_velocity(self):
        c = self.controller
        c.read()
        c._drive((.05, .1))
        for desired, stops in (((.025, .1), ['west']), ((-.025, .1), ['west']),
                               ((-.025, 0), ['north']), ((0, 0), ['east'])):
            self.plant.commands.clear()
            c._drive(desired)
            self.assertEqual(tuple(self.plant.velocity), desired)
            self.assertEqual([x[1] for x in self.plant.commands if x[0] == 'stop'], stops)
        self.plant.commands.clear()
        c._drive((0, 0))
        self.assertEqual(self.plant.commands, [])

    def test_partner_progress_does_not_hide_one_stalled_axis(self):
        for stuck_axis in (0, 1):
            with self.subTest(axis=stuck_axis):
                plant = Plant(self.clock)
                integrate = plant.integrate
                def stalled():
                    old = plant.q[stuck_axis]
                    integrate()
                    plant.q[stuck_axis] = old
                plant.integrate = stalled
                c = Controller(plant, Reference(-90, 90), positive_directions=('west', 'north'),
                               settings=replace(Settings(), progress_timeout=.6))
                with self.assertRaisesRegex(ControlError, 'No measurable progress'):
                    c.run(ra_degrees=1, dec_degrees=1)
                self.assertGreater(abs(plant.q[1-stuck_axis]), .02)
                self.assertFalse(any(plant.velocity))

    def test_cancel_or_stale_state_between_axis_starts_stops_both(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                cancel = Event()
                original = self.plant.jog
                def first_axis(*args, **kwargs):
                    original(*args, **kwargs)
                    self.clock.sleep(.3) if stale else cancel.set()
                self.plant.commands.clear()
                with patch.object(self.plant, 'jog', first_axis):
                    with self.assertRaisesRegex(ControlError, 'stale' if stale else 'Cancelled'):
                        self.controller.run(ra_degrees=1, dec_degrees=1, cancel=cancel)
                self.assertEqual([x[0] for x in self.plant.commands if x[0] != 'stop'], ['west'])
                self.assertFalse(any(self.plant.velocity))

    def test_second_axis_dispatch_failure_globally_stops(self):
        original = self.plant.jog
        def fail_second(direction, **kwargs):
            if direction == 'north':
                raise OSError('Second axis write failed')
            original(direction, **kwargs)
        with patch.object(self.plant, 'jog', fail_second):
            with self.assertRaisesRegex(OSError, 'Second axis'):
                self.controller.run(ra_degrees=1, dec_degrees=1)
        self.assertFalse(any(self.plant.velocity))
        self.assertEqual(self.plant.commands[-1], ('stop',))

    def test_serial_direction_local_stops(self):
        with simulated([(b':Qw#', b''), (b':Qs#', b''), (b':Q#', b'')]) as mount:
            with self.assertRaises(ValueError):
                mount.stop('invalid')
            mount.stop('west')
            mount.stop('south')
            mount.stop()

    def test_saved_measurements_reconstruct_offsets(self):
        root = Path(__file__).resolve().parents[1]
        reference = Reference.from_baseline(root / "baseline-2026-09-08T212838Z.json")
        self.assertEqual(reference.angles, (-90, 90))
        first = reference.offsets(86.55, 89.9988888889)
        second = reference.offsets(-90.0083333333, 77.5755555556, first)
        self.assertAlmostEqual(first[0], -3.45)
        self.assertAlmostEqual(second[0], -0.0083333333)
        self.assertAlmostEqual(second[1], -12.4244444444)

    def test_invalid_targets_send_nothing(self):
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.controller.run(ra_degrees=0, dec_degrees=value)
        self.assertEqual(self.plant.commands, [])

    def test_bad_state_stops_without_jogging(self):
        for flags in ("G", "nZ", "nGs", "nGL"):
            self.plant.flags = flags
            with self.assertRaises(ControlError):
                self.controller.run(ra_degrees=0, dec_degrees=1)
        self.assertTrue(all(c == ("stop",) for c in self.plant.commands))

    def test_cancel_and_stale_callback_stop(self):
        cancel = Event()
        with self.assertRaisesRegex(ControlError, "Cancelled"):
            self.controller.run(ra_degrees=0, dec_degrees=1, cancel=cancel,
                                on_sample=lambda state: cancel.set())
        with self.assertRaisesRegex(ControlError, "stale"):
            self.controller.run(ra_degrees=0, dec_degrees=1,
                                on_sample=lambda state: self.clock.sleep(1))
        self.assertFalse(any(self.plant.velocity))

    def test_wrong_direction_and_frozen_feedback(self):
        self.plant.polarity = -1
        with self.assertRaisesRegex(ControlError, "opposes"):
            self.controller.run(ra_degrees=0, dec_degrees=1)
        self.assertFalse(any(self.plant.velocity))
        self.plant.polarity, self.plant.frozen = 1, True
        with self.assertRaisesRegex(ControlError, "No measurable progress"):
            self.controller.run(ra_degrees=0, dec_degrees=1)
        self.assertFalse(any(self.plant.velocity))

    def test_deadline_and_callback_failure(self):
        self.controller.settings = replace(Settings(), timeout=0.2)
        with self.assertRaisesRegex(ControlError, "timed out"):
            self.controller.run(ra_degrees=0, dec_degrees=5)
        def fail(state):
            raise RuntimeError("callback failed")
        with self.assertRaisesRegex(RuntimeError, "callback failed"):
            self.controller.run(ra_degrees=0, dec_degrees=5, on_sample=fail)
        self.assertFalse(any(self.plant.velocity))

    def test_large_position_allowed_but_discontinuity_rejected(self):
        self.plant.q[1] = 45
        self.assertAlmostEqual(self.controller.read().angles[1], 45)
        self.plant.q = [0, 0]
        def jump(state):
            self.plant.q[0] = 2
        with self.assertRaisesRegex(ControlError, "displacement"):
            self.controller.run(ra_degrees=0, dec_degrees=1, on_sample=jump)
        self.assertFalse(any(self.plant.velocity))

    def test_settings_validation(self):
        self.assertEqual(Settings().deadband, .01)
        for args in ({"kp": 0}, {"max_speed": 7}, {"deadband": 0}, {"settle_samples": True}):
            with self.assertRaises(ValueError):
                Settings(**args)

    def test_cancelled_restart_accounts_for_motion_before_stop(self):
        for axis in (0, 1):
            with self.subTest(axis=axis):
                plant = Plant(self.clock)
                c = Controller(plant, Reference(-90, 90), positive_directions=('west', 'north'),
                               settings=Settings(max_speed=1))
                velocity = [0, 0]
                velocity[axis] = -.7
                c.read()
                c._drive(velocity)
                self.clock.sleep(.1)
                before = c.read()
                self.clock.sleep(.04)
                velocity[axis] = -.65
                permits = iter((True, True, False))
                self.assertFalse(c._drive(velocity, lambda: next(permits)))
                self.assertEqual(c._action[axis], 0)
                after = c.read()
                self.assertGreater(abs(after.angles[axis]-before.angles[axis]), c.settings.deadband)
                # Once the transition interval is consumed, idle checks tighten again.
                plant.q[axis] += .02
                with self.assertRaisesRegex(ControlError, 'interval_motion=False'):
                    c.read()

    def test_start_stop_between_samples_and_global_stop_preserve_interval(self):
        for global_stop in (False, True):
            with self.subTest(global_stop=global_stop):
                plant = Plant(self.clock)
                c = Controller(plant, Reference(-90, 90), positive_directions=('west', 'north'),
                               settings=Settings(max_speed=1))
                c.read()
                c._drive((.7, -.7))
                self.clock.sleep(.04)
                c._stop() if global_stop else c._drive((0, 0))
                state = c.read()
                self.assertAlmostEqual(state.angles[0], .028)
                self.assertAlmostEqual(state.angles[1], -.028)
                self.assertEqual(c._interval_motion, [False, False])

    def test_interval_allowance_is_bounded_and_rejected_sample_is_not_consumed(self):
        c = self.controller
        c.read()
        c._drive((.1, 0))
        self.clock.sleep(.1)
        c._drive((0, 0))
        self.plant.q[0] += 1
        with self.assertRaisesRegex(ControlError, 'axis=0.*measured=.*bound=.*interval_motion=True'):
            c.read()
        self.assertTrue(c._interval_motion[0])
        self.assertEqual(c._previous.angles, (0, 0))

    def test_serial_velocity_and_combined_read(self):
        with simulated([(b":Rv23.93#", b""), (b":Mn#", b""),
                        (b":GS#", b"23:59:59#"), (b":GMEQ#", b"06:00:00&+90*00:00#"),
                        (b":GS#", b"00:00:00#"), (b":GU#", b"nNG#")]) as mount:
            mount.jog("north", speed_degrees_s=0.1)
            h, d, flags = mount.joint_sample()
            self.assertAlmostEqual(h, -90-15/7200)
            self.assertEqual((d, flags), (90, "nNG"))

    def test_setting_error_frame_is_consumed_and_closes(self):
        with simulated([(b":Td#", b"e3#")]) as mount:
            with self.assertRaisesRegex(RuntimeError, "e3"):
                mount.set_tracking(False)
            self.assertFalse(mount._serial.is_open)


if __name__ == "__main__":
    unittest.main()
