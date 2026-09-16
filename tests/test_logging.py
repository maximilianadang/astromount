from contextlib import redirect_stderr
from dataclasses import replace
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import Mock, patch

from astromount_logging import sweep_log
from astromount_control import Settings, State, Snapshot, Lifecycle, Target
from astromount_config import FRAME
from astromount_trajectory import sweep


class LoggingTests(TestCase):
    def test_unique_flushed_files_and_outcomes(self):
        with TemporaryDirectory() as directory, patch('astromount_logging.ROOT', Path(directory)), redirect_stderr(StringIO()):
            for error, status in [(None, 'completed'), (KeyboardInterrupt(), 'interrupted'), (RuntimeError('fault'), 'failed')]:
                try:
                    with sweep_log(dict(test=True)) as log:
                        log('target', commanded_azel=[15, 0])
                        files = sorted((Path(directory)/'output').glob('*.jsonl'))
                        rows = [json.loads(line) for line in files[-1].read_text().splitlines()]
                        self.assertEqual(rows[-1]['event'], 'target')  # Visible before close.
                        if error: raise error
                except BaseException as caught:
                    self.assertIs(caught, error)
                rows = [json.loads(line) for line in files[-1].read_text().splitlines()]
                self.assertEqual(rows[-1]['status'], status)
                self.assertIn('unix_s', rows[0])
            self.assertEqual(len(files), 3)

    def test_timestamped_plan_targets_and_measurements(self):
        clock, events = [0.], []
        state = State(0,0,0,.01,'nNG')
        control = Mock(settings=Settings())
        control.read.return_value = state
        worker = Mock(heartbeat=.5)
        worker.snapshot = Snapshot(mode=Lifecycle.ARMED, state=state, arrived=True)
        def publish(frame, *, azimuth, elevation, issued_at):
            q = frame.inverse(azimuth, elevation)
            target = Target(q, issued_at, len(events))
            worker.snapshot = replace(worker.snapshot, state=State(*q, issued_at, issued_at+.01,'nNG'), target=target, applied=target)
            return target.sequence
        worker.set_pointing.side_effect = publish
        def emit(event, **values):
            events.append(json.loads(json.dumps(dict(event=event, **values))))
        with patch('astromount_trajectory.Worker') as factory, \
             patch('astromount_trajectory.monotonic', side_effect=lambda: clock[0]), \
             patch('astromount_trajectory.sleep', side_effect=lambda seconds: clock.__setitem__(0, clock[0]+.1)):
            factory.return_value.__enter__.return_value = worker
            sweep(control, FRAME, [(1,0,.2)], log=emit)
        self.assertEqual(events[0]['event'], 'initial')
        plan = next(e for e in events if e['event'] == 'plan')
        self.assertEqual(plan['began_monotonic_s'], 0)
        self.assertEqual(plan['settings']['max_speed'], 3)
        targets = [e for e in events if e['event'] == 'target']
        self.assertAlmostEqual(targets[-1]['commanded_azel'][0], 1)
        feedback = events[-1]
        self.assertEqual(feedback['event'], 'feedback')
        self.assertAlmostEqual(feedback['measured_azel'][0], 1)
        self.assertIn('started_at', feedback['snapshot']['state'])
        self.assertIn('issued_at', feedback['snapshot']['applied'])

    def test_logging_failure_exits_worker(self):
        control = Mock(settings=Settings())
        control.read.return_value = State(0,0,0,0,'nNG')
        def fail(event, **values):
            if event == 'plan': raise OSError('disk full')
        with patch('astromount_trajectory.Worker') as factory, self.assertRaises(OSError):
            sweep(control, FRAME, [(1,0,1)], log=fail)
        factory.return_value.__exit__.assert_called_once()
