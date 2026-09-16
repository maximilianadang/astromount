"""Real-thread tests using an in-memory plant; never opens a serial device."""
from dataclasses import replace
from threading import Event, get_ident
from time import monotonic, sleep
import unittest

from astromount_control import Controller, ControlError, Lifecycle, Reference, Settings, Worker
from test_control import Plant
from astromount_kinematics import Pointing


class RealClock:
    @property
    def now(self):
        return monotonic()

    sleep = staticmethod(sleep)


class ThreadPlant(Plant):
    def __init__(self):
        super().__init__(RealClock())
        self.last = monotonic()
        self.owners = set()
        self.gate = None
        self.entered = Event()
        self.reads = []

    def joint_sample(self):
        self.owners.add(get_ident())
        if self.gate is not None:
            self.entered.set()
            self.gate.wait(1)
        self.reads.append(monotonic())
        return super().joint_sample()

    def jog(self, *args, **kwargs):
        self.owners.add(get_ident())
        super().jog(*args, **kwargs)

    def stop(self, direction=None):
        self.owners.add(get_ident())
        super().stop(direction)


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.plant = ThreadPlant()
        self.control = Controller(self.plant, Reference(-90, 90), positive_directions=('west', 'north'),
                                  settings=replace(Settings(), period=.01))
        self.worker = Worker(self.control, heartbeat=.15).start()
        self.addCleanup(self.worker.close)

    def wait_for(self, predicate, timeout=1):
        end = monotonic()+timeout
        while not predicate():
            if monotonic() > end:
                self.fail(f'Timed out: {self.worker.snapshot}')
            sleep(.002)

    def test_start_disarmed_and_single_serial_owner(self):
        self.wait_for(lambda: self.worker.snapshot.state is not None)
        self.assertFalse(self.plant.commands)
        with self.assertRaises(ControlError):
            self.worker.set_target(ra_degrees=0, dec_degrees=1)
        self.worker.arm(ra_degrees=0, dec_degrees=.1)
        self.wait_for(lambda: any(self.plant.velocity))
        self.worker.close()
        self.assertFalse(any(self.plant.velocity))
        self.assertEqual(len(self.plant.owners), 1)
        self.assertNotIn(get_ident(), self.plant.owners)

    def test_pointing_starts_both_axes_and_heartbeat_stops_both(self):
        self.worker.arm_pointing(Pointing(1, -1), azimuth=1, elevation=1)
        self.wait_for(lambda: all(self.plant.velocity))
        self.assertGreater(self.plant.velocity[0], 0)
        self.assertLess(self.plant.velocity[1], 0)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertIn('Heartbeat', self.worker.snapshot.fault)
        self.assertFalse(any(self.plant.velocity))
        self.assertEqual(len(self.plant.owners), 1)

    def test_replacement_between_axis_starts_cancels_second_old_command(self):
        entered, release = Event(), Event()
        original = self.plant.jog
        def jog(direction, **kwargs):
            original(direction, **kwargs)
            if direction == 'west':
                entered.set()
                if not release.wait(1):
                    raise RuntimeError('Test gate timed out')
        self.plant.jog = jog
        self.addCleanup(release.set)
        self.worker.arm(ra_degrees=1, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        seq = self.worker.set_target(ra_degrees=-1, dec_degrees=-1)
        release.set()
        self.wait_for(lambda: all(v < 0 for v in self.plant.velocity))
        self.assertNotIn('north', [c[0] for c in self.plant.commands])
        self.assertEqual(self.worker.snapshot.applied.sequence, seq)

    def test_heartbeat_latches_and_requires_explicit_arm(self):
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: self.worker.snapshot.fault is not None)
        self.assertIn('Heartbeat', self.worker.snapshot.fault)
        self.assertFalse(any(self.plant.velocity))
        with self.assertRaises(ControlError):
            self.worker.set_target(ra_degrees=0, dec_degrees=1)
        self.worker.arm(ra_degrees=0, dec_degrees=-1)
        self.wait_for(lambda: self.plant.velocity[1] < 0)

    def test_latest_target_after_read_and_no_mailbox_io_block(self):
        self.wait_for(lambda: self.worker.snapshot.state is not None)
        self.plant.gate = Event()
        self.assertTrue(self.plant.entered.wait(.5))
        try:
            self.worker.arm(ra_degrees=0, dec_degrees=1)
            for i in range(100):
                sequence = self.worker.set_target(ra_degrees=0, dec_degrees=-1)
        finally:
            self.plant.gate.set()
        self.wait_for(lambda: self.worker.snapshot.target is not None)
        self.assertEqual(self.worker.snapshot.target.sequence, sequence)
        self.assertTrue(all(c[0] in ('stop', 'south') for c in self.plant.commands))

    def test_invalid_and_out_of_order_targets_do_not_refresh(self):
        stamp = monotonic()
        self.worker.arm(ra_degrees=0, dec_degrees=1, issued_at=stamp)
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.worker.set_target(ra_degrees=0, dec_degrees=value)
        for when in (stamp, stamp-1, monotonic()+1):
            with self.assertRaises(ValueError):
                self.worker.set_target(ra_degrees=0, dec_degrees=1, issued_at=when)
        self.wait_for(lambda: self.worker.snapshot.fault is not None)

    def test_stop_clears_target_and_shutdown_rejects_publications(self):
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: any(self.plant.velocity))
        self.worker.stop()
        self.wait_for(lambda: not self.worker.snapshot.armed and not any(self.plant.velocity))
        with self.assertRaises(ControlError):
            self.worker.set_target(ra_degrees=0, dec_degrees=1)
        self.worker.close()
        with self.assertRaises(RuntimeError):
            self.worker.arm(ra_degrees=0, dec_degrees=0)

    def test_arrival_keeps_heartbeat_and_accepts_new_target(self):
        self.worker.arm(ra_degrees=0, dec_degrees=0)
        self.wait_for(lambda: self.worker.snapshot.arrived)
        self.worker.set_target(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: any(self.plant.velocity))
        self.assertFalse(self.worker.snapshot.arrived)

    def test_target_refresh_does_not_reset_progress_watchdog(self):
        self.control.settings = replace(self.control.settings, progress_timeout=.05)
        self.plant.frozen = True
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        end = monotonic()+.5
        while self.worker.snapshot.fault is None and monotonic() < end:
            try:
                self.worker.set_target(ra_degrees=0, dec_degrees=1)
            except ControlError:
                break
            sleep(.01)
        self.wait_for(lambda: self.worker.snapshot.fault is not None)
        self.assertIn('progress', self.worker.snapshot.fault)

    def test_expired_target_cannot_be_revived_during_blocked_read(self):
        self.wait_for(lambda: self.worker.snapshot.state is not None)
        self.plant.gate = Event()
        self.assertTrue(self.plant.entered.wait(.5))
        try:
            self.worker.arm(ra_degrees=0, dec_degrees=1)
            sleep(.16)
            with self.assertRaisesRegex(ControlError, 'Heartbeat expired'):
                self.worker.set_target(ra_degrees=0, dec_degrees=1)
        finally:
            self.plant.gate.set()
        self.wait_for(lambda: self.worker.snapshot.fault is not None)
        self.assertFalse(any(c[0] != 'stop' for c in self.plant.commands))

    def test_updates_do_not_extend_continuous_motion_timeout(self):
        self.worker.close()
        self.control.settings = replace(self.control.settings, timeout=.08)
        self.worker = Worker(self.control, heartbeat=.15).start()
        self.addCleanup(self.worker.close)
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        end = monotonic()+.5
        while self.worker.snapshot.fault is None and monotonic() < end:
            try:
                self.worker.set_target(ra_degrees=0, dec_degrees=1.1)
            except ControlError:
                break
            sleep(.01)
        self.wait_for(lambda: self.worker.snapshot.fault is not None)
        self.assertIn('timed out', self.worker.snapshot.fault)

    def pause(self, obj, method):
        """Deterministic interleaving: block method until test releases it."""
        entered, release = Event(), Event()
        original = getattr(obj, method)
        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(1):
                raise RuntimeError('Test gate timed out')
            return original(*args, **kwargs)
        setattr(obj, method, blocked)
        self.addCleanup(setattr, obj, method, original)
        self.addCleanup(release.set)
        return entered, release

    def test_explicit_lifecycle_and_idempotent_close(self):
        unused = Worker(self.control)
        self.assertEqual(unused.snapshot.mode, Lifecycle.NEW)
        with self.assertRaises(ControlError):
            unused.arm(ra_degrees=0, dec_degrees=0)
        unused.stop()
        unused.close()
        self.assertEqual(unused.snapshot.mode, Lifecycle.CLOSED)
        unused.close()
        with self.assertRaises(RuntimeError):
            unused.start()
        self.wait_for(lambda: self.worker.snapshot.state is not None)
        entered, release = self.pause(self.plant, 'joint_sample')
        self.assertTrue(entered.wait(.5))
        self.worker.arm(ra_degrees=0, dec_degrees=0)
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.ARMING)
        with self.assertRaises(ControlError):
            self.worker.arm(ra_degrees=0, dec_degrees=0)
        release.set()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.ARMED)
        self.worker.close()
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.CLOSED)
        self.worker.stop()
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.CLOSED)

    def test_stop_between_measurement_and_dispatch_prevents_jog(self):
        entered, release = self.pause(self.control, '_step')
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        self.worker.stop()
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.STOPPING)
        with self.assertRaises(ControlError):
            self.worker.arm(ra_degrees=0, dec_degrees=0)
        release.set()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.DISARMED)
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands))
        self.assertIsNone(self.worker.snapshot.target)
        self.assertIsNone(self.worker.snapshot.applied)

    def test_stop_during_speed_change_stop_prevents_restart(self):
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: any(self.plant.velocity))
        entered, release = self.pause(self.plant, 'stop')
        before = len(self.plant.commands)
        self.worker.set_target(ra_degrees=0, dec_degrees=-1)
        self.assertTrue(entered.wait(.5))
        self.worker.stop()
        release.set()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.DISARMED)
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands[before:]))

    def test_shutdown_between_measurement_and_dispatch_prevents_jog(self):
        entered, release = self.pause(self.control, '_step')
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        with self.assertRaisesRegex(ControlError, 'did not stop'):
            self.worker.close(timeout=.001)
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.CLOSING)
        with self.assertRaises(ControlError):
            self.worker.set_target(ra_degrees=0, dec_degrees=1)
        release.set()
        self.worker.close()
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands))
        self.assertEqual(self.worker.snapshot.mode, Lifecycle.CLOSED)

    def test_replacement_cancels_old_decision_and_telemetry_identifies_new_one(self):
        entered, release = self.pause(self.control, '_step')
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        seq = self.worker.set_target(ra_degrees=0, dec_degrees=-1)
        self.assertIsNone(self.worker.snapshot.applied)
        self.assertEqual(self.worker.snapshot.target.sequence, seq)
        release.set()
        self.wait_for(lambda: self.plant.velocity[1] < 0)
        snap = self.worker.snapshot
        self.assertEqual(snap.applied.sequence, seq)
        self.assertTrue(all(c[0] in ('stop', 'south') for c in self.plant.commands))

    def test_expiry_during_stop_io_prevents_restart(self):
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: any(self.plant.velocity))
        entered, release = self.pause(self.plant, 'stop')
        before = len(self.plant.commands)
        self.worker.set_target(ra_degrees=0, dec_degrees=-1)
        self.assertTrue(entered.wait(.5))
        sleep(.16)
        release.set()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertIn('Heartbeat', self.worker.snapshot.fault)
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands[before:]))

    def test_fault_monitors_without_restarting_and_stop_does_not_clear_fault(self):
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        fault, stamp = self.worker.snapshot.fault, self.worker.snapshot.state.finished_at
        self.wait_for(lambda: self.worker.snapshot.state.finished_at > stamp)
        self.worker.stop()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertEqual(self.worker.snapshot.fault, fault)
        self.worker.arm(ra_degrees=0, dec_degrees=0)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.ARMED)
        self.assertIsNone(self.worker.snapshot.fault)

    def test_stationary_preflight_rejects_moving_mount(self):
        self.wait_for(lambda: self.worker.snapshot.state is not None)
        self.plant.velocity[1] = .001
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertIn('stationary', self.worker.snapshot.fault)
        self.assertFalse(any(self.plant.velocity))

    def test_reversals_do_not_reset_progress_timeout(self):
        self.worker.close()
        self.control.settings = replace(self.control.settings, progress_timeout=.08)
        self.plant.frozen = True
        self.worker = Worker(self.control, heartbeat=.15).start()
        self.addCleanup(self.worker.close)
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        end, sign = monotonic()+.5, 1
        while self.worker.snapshot.mode != Lifecycle.FAULT and monotonic() < end:
            sign *= -1
            try:
                self.worker.set_target(ra_degrees=0, dec_degrees=sign)
            except ControlError:
                break
            sleep(.015)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertIn('progress', self.worker.snapshot.fault)

    def test_unchanged_heartbeat_preserves_arrival(self):
        self.worker.arm(ra_degrees=0, dec_degrees=0)
        self.wait_for(lambda: self.worker.snapshot.arrived)
        self.worker.set_target(ra_degrees=0, dec_degrees=0)
        self.assertTrue(self.worker.snapshot.arrived)

    def test_overrun_skips_ticks_instead_of_catching_up(self):
        entered, release = self.pause(self.control, '_step')
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        sleep(.04)
        released = monotonic()
        release.set()
        self.wait_for(lambda: any(t > released for t in self.plant.reads))
        first = next(t for t in self.plant.reads if t > released)
        self.assertGreaterEqual(first - released, self.control.settings.period * .9)

    def test_stale_state_at_dispatch_faults_without_jogging(self):
        self.worker.close()
        self.worker = Worker(self.control, heartbeat=.5).start()
        self.addCleanup(self.worker.close)
        entered, release = self.pause(self.control, '_step')
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.assertTrue(entered.wait(.5))
        sleep(.26)
        release.set()
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertIn('stale', self.worker.snapshot.fault)
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands))

    def test_close_reports_stop_failure_without_hiding_closed_state(self):
        self.worker.close()
        worker = Worker(self.control).start()
        self.wait_for(lambda: worker.snapshot.state is not None)
        original = self.plant.stop
        def fail():
            raise OSError('Simulated lost link')
        self.plant.stop = fail
        try:
            with self.assertRaisesRegex(ControlError, 'Stop transmission failed'):
                worker.close()
            self.assertEqual(worker.snapshot.mode, Lifecycle.CLOSED)
            self.assertFalse(worker.snapshot.armed)
            self.assertIsNone(worker.snapshot.target)
        finally:
            self.plant.stop = original

    def test_fault_rearm_must_pass_preflight(self):
        self.plant.flags = 'G'  # Tracking enabled, invalid for this controller.
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.worker.arm(ra_degrees=0, dec_degrees=1)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.FAULT)
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands))
        self.plant.flags = 'nG'
        self.worker.arm(ra_degrees=0, dec_degrees=0)
        self.wait_for(lambda: self.worker.snapshot.mode == Lifecycle.ARMED)


if __name__ == '__main__':
    unittest.main()
