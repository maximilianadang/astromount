import math
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch

from astromount_control import ControlError, Worker
from astromount_kinematics import Pointing
from astromount_trajectory import spiral, trace


class TrajectoryTests(TestCase):
    frame = Pointing(1, -1)

    def test_cone_radius_endpoints_and_ik(self):
        for i in range(1001):
            az, el = spiral(i/1000)
            joints = self.frame.inverse(az, el)
            x, _, _ = self.frame.direction(*joints)
            radius = math.degrees(math.acos(min(1, max(-1, x))))
            self.assertAlmostEqual(radius, 5*(1-abs(2*i/1000-1)), places=7)
            self.assertLessEqual(max(map(abs, joints)), 5 + 1e-10)
        self.assertEqual(spiral(0), (0, 0))
        self.assertEqual(spiral(1), (0, 0))
        for kwargs in ({'radius': 0}, {'radius': 6}, {'turns': 0}, {'radius': float('nan')}):
            with self.assertRaises(ValueError):
                spiral(.5, **kwargs)

    def test_pointing_uses_existing_mailbox(self):
        worker = Worker(Mock())
        with patch.object(worker, '_publish') as publish:
            worker.arm_pointing(self.frame, azimuth=5, elevation=0, issued_at=10)
            lower, upper, stamp, arm = publish.call_args.args
            self.assertAlmostEqual(lower, 0)
            self.assertAlmostEqual(upper, -5)
            self.assertEqual((stamp, arm), (10, True))
            worker.set_pointing(self.frame, azimuth=0, elevation=5, issued_at=11)
            self.assertEqual(publish.call_args.args, (5, 0, 11, False))
            publish.reset_mock()
            with self.assertRaises(ValueError):
                worker.set_pointing(self.frame, azimuth=180, elevation=0)
            publish.assert_not_called()

    def test_trace_skips_delays_refreshes_and_returns_to_zero(self):
        clock, targets = [0.0], []
        snapshot = SimpleNamespace(fault=None, armed=True, arrived=False, state='final')
        worker = Mock(snapshot=snapshot)
        def publish(frame, *, azimuth, elevation, issued_at=None):
            targets.append((clock[0], azimuth, elevation))
            snapshot.arrived = False
        def sleep(delay):
            clock[0] += max(.1, delay) + (.3 if len(targets) == 3 else 0)
            snapshot.arrived = True
        worker.set_pointing.side_effect = publish
        with patch('astromount_trajectory.monotonic', side_effect=lambda: clock[0]), \
             patch('astromount_trajectory.sleep', side_effect=sleep):
            self.assertEqual(trace(worker, self.frame, duration=1), 'final')
        worker.arm_pointing.assert_called_once_with(self.frame, azimuth=0, elevation=0)
        self.assertEqual(targets[-1][1:], (0, 0))
        self.assertTrue(any(az or el for _, az, el in targets))
        self.assertLess(len(targets), 16)  # No catch-up burst after the delayed tick.

    def test_fault_and_invalid_duration(self):
        worker = Mock(snapshot=SimpleNamespace(fault='Heartbeat expired', armed=False))
        with self.assertRaisesRegex(ControlError, 'Heartbeat expired'):
            trace(worker, self.frame)
        worker.set_pointing.assert_not_called()
        worker.reset_mock()
        with self.assertRaises(ValueError):
            trace(worker, self.frame, duration=float('nan'))
        worker.arm_pointing.assert_not_called()
