from dataclasses import replace
from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from astromount_control import Controller, ControlError, Reference, Settings
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

    def stop(self):
        self.integrate()
        self.velocity = [0.0, 0.0]
        self.commands.append(("stop",))

    def jog(self, direction, *, speed_degrees_s):
        self.integrate()
        axis = 0 if direction in ("east", "west") else 1
        assert not any(self.velocity), "Shared speed changed during active movement"
        self.velocity[axis] = speed_degrees_s * (1 if direction in ("west", "north") else -1)
        self.commands.append((direction, speed_degrees_s))


class ControlTests(unittest.TestCase):
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
            self.assertLessEqual(abs(state.dec_degrees-dec), 0.02)
            self.assertEqual(state.ra_degrees, 0)
        self.assertFalse(any(self.plant.velocity))
        self.assertTrue(all(c[1] <= 0.1 for c in self.plant.commands if len(c) == 2))

    def test_two_axis_target(self):
        state = self.controller.run(ra_degrees=1, dec_degrees=-2)
        for actual, target in zip(state.angles, (1, -2)):
            self.assertLessEqual(abs(actual-target), 0.02)

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
        for value in (22.5, -22.5, 22.24, float("nan"), float("inf")):
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

    def test_excursion_and_discontinuity(self):
        self.plant.q[1] = 22.3
        with self.assertRaisesRegex(ControlError, "boundary"):
            self.controller.run(ra_degrees=0, dec_degrees=0)
        self.plant.q = [0, 0]
        def jump(state):
            self.plant.q[0] = 2
        with self.assertRaisesRegex(ControlError, "displacement"):
            self.controller.run(ra_degrees=0, dec_degrees=1, on_sample=jump)
        self.assertFalse(any(self.plant.velocity))

    def test_settings_validation(self):
        for args in ({"kp": 0}, {"max_speed": 7}, {"max_speed": 1}, {"margin": 23}, {"deadband": 0.5}, {"settle_samples": True}):
            with self.assertRaises(ValueError):
                Settings(**args)

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
