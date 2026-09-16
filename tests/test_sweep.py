from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch

import point
from astromount_control import ControlError, Settings, State
from astromount_config import FRAME
from astromount_trajectory import sweep


class SweepTests(TestCase):
    def run_sweep(self, points, *, delta=False, delay=0, fault=None, arrive=True):
        clock, targets = [0.], []
        state = State(*FRAME.inverse(0, 10), 0, 0, 'nNG')
        settings = Settings(timeout=.5)
        control = Mock(settings=settings)
        control.read.return_value = state
        snapshot = SimpleNamespace(armed=True, fault=None, arrived=False, state=state)
        worker = Mock(snapshot=snapshot)
        def publish(frame, *, azimuth, elevation, issued_at=None):
            targets.append((clock[0], azimuth, elevation))
            snapshot.state = State(*frame.inverse(azimuth, elevation), 0, clock[0], 'nNG')
            snapshot.arrived = False
        def sleep(seconds):
            clock[0] += max(seconds, .1) + (delay if len(targets) == 2 else 0)
            snapshot.arrived, snapshot.fault = arrive, fault
        worker.set_pointing.side_effect = publish
        with patch('astromount_trajectory.Worker') as factory, \
             patch('astromount_trajectory.monotonic', side_effect=lambda: clock[0]), \
             patch('astromount_trajectory.sleep', side_effect=sleep):
            factory.return_value.__enter__.return_value = worker
            def create(controller):
                self.assertEqual(controller.settings.max_speed, settings.speed_limit)
                self.assertAlmostEqual(controller.settings.timeout, sum(p[2] for p in points) + settings.timeout)
                return factory.return_value
            factory.side_effect = create
            try:
                final = sweep(control, FRAME, points, delta=delta)
            finally:
                self.assertIs(control.settings, settings)
                factory.return_value.__exit__.assert_called_once()
        worker.arm_pointing.assert_called_once()
        return targets, final

    def test_fixed_elevation_and_both_joint_targets(self):
        targets, final = self.run_sweep([(10,10,1), (-10,10,1)])
        self.assertTrue(all(abs(el-10) < 1e-10 for _, az, el in targets))
        self.assertAlmostEqual(final.pointing(FRAME)[0], -10)
        joints = [FRAME.inverse(az, el) for _, az, el in targets]
        self.assertGreater(max(q[0] for q in joints)-min(q[0] for q in joints), .1)
        self.assertGreater(max(q[1] for q in joints)-min(q[1] for q in joints), 15)
        self.assertLess(targets[-1][0], 2.2)

    def test_delta_accumulates_planned_endpoints(self):
        targets, final = self.run_sweep([(10,0,1), (-20,0,1)], delta=True)
        self.assertAlmostEqual(final.pointing(FRAME)[0], -10)
        self.assertTrue(all(abs(el-10) < 1e-10 for _, az, el in targets))

    def test_delayed_ticks_skip_instead_of_queue(self):
        targets, final = self.run_sweep([(10,10,1), (20,10,1)], delay=1.1)
        after = targets[2]
        self.assertGreater(after[0], 1)
        self.assertAlmostEqual(after[1], 10*after[0])
        self.assertLess(len(targets), 15)

    def test_fault_and_final_timeout_stop_worker(self):
        with self.assertRaisesRegex(ControlError, 'Heartbeat expired'):
            self.run_sweep([(10,10,1)], fault='Heartbeat expired')
        with self.assertRaisesRegex(ControlError, 'arrival timed out'):
            self.run_sweep([(10,10,.2)], arrive=False)

    def test_interrupt_stops_worker(self):
        control = Mock(settings=Settings())
        control.read.return_value = State(0,0,0,0,'nNG')
        with patch('astromount_trajectory.Worker') as factory:
            worker = factory.return_value.__enter__.return_value
            worker.snapshot = SimpleNamespace(fault=None, armed=True, arrived=False)
            worker.set_pointing.side_effect = KeyboardInterrupt
            with self.assertRaises(KeyboardInterrupt): sweep(control, FRAME, [(10,0,1)])
            factory.return_value.__exit__.assert_called_once()

    def test_invalid_later_segment_never_starts_worker(self):
        control = Mock(settings=Settings())
        control.read.return_value = State(0,0,0,0,'nNG')
        with patch('astromount_trajectory.Worker') as worker, self.assertRaises(ValueError):
            sweep(control, FRAME, [(10,0,1), (180,0,1)])
        worker.assert_not_called()

    def test_cli_reuses_inputs_and_preserves_sequence_mode(self):
        with patch('point.read_waypoints', return_value=[(10,0,1), (0,0,1)]), \
             patch('point.sweep_log') as logging, \
             patch('point.Mount') as mount, patch('point.Controller'), \
             patch('point.sweep', return_value=State(0,0,0,1,'nNG')) as run, \
             redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            self.assertEqual(point.main(['--path', 'unused.csv', '--dry-run'], sequence=True, continuous=True), 0)
            mount.assert_not_called()
            run.assert_not_called()
            logging.assert_not_called()
            self.assertEqual(point.main(['--path', 'unused.csv', '--delta'], sequence=True, continuous=True), 0)
            self.assertEqual(run.call_args.args[2], [(10,0,1), (0,0,1)])
            self.assertTrue(run.call_args.kwargs['delta'])
            self.assertIs(run.call_args.kwargs['log'], logging.return_value.__enter__.return_value)
