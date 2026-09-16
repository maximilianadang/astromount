from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from pathlib import Path
from unittest import TestCase
from unittest.mock import patch

import point
from astromount_control import Controller, State, Settings, duration_settings


class SequenceTests(TestCase):
    def invoke(self, text, *options):
        original = Path.open
        with patch('pathlib.Path.open', lambda path, *a, **kw: StringIO(text) if path == Path('waypoints.csv') else original(path, *a, **kw)):
            return point.main(['--path', 'waypoints.csv', *options], sequence=True)

    def test_invalid_file_never_opens_mount(self):
        for text in ('', 'az,el,duration\n', 'az,el\n1,2\n',
                     'az,el,duration\n0,0,1\n180,0,1\n',
                     'az,el,duration\nnan,0,1\n', 'az,el,duration\n0,0,0\n',
                     'az,el,duration\n0,0,inf\n', 'az,el,duration\n0,0\n'):
            with self.subTest(text=text), patch('point.Mount') as mount, redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit): self.invoke(text)
                mount.assert_not_called()

    def test_dry_run(self):
        with patch('point.Mount') as mount, redirect_stdout(StringIO()):
            self.assertEqual(self.invoke('az,el,duration\n45,30,10\n0,0,5\n', '--dry-run'), 0)
            mount.assert_not_called()

    def test_sequential_targets_and_per_move_duration(self):
        states = [State(0,0,0,1,'nNG'), State(0,-5,2,3,'nNG')]
        calls = []
        def reached(control, frame, *, azimuth, elevation):
            calls.append((azimuth, elevation, control.settings.max_speed, control.settings.timeout))
            return states[len(calls)-1]
        with patch('point.Mount') as mount, patch.object(Controller, 'read', side_effect=states), \
             patch.object(Controller, 'run_pointing', reached), redirect_stdout(StringIO()) as output:
            self.assertEqual(self.invoke('az,el,duration\n5,0,10\n0,0,1\n'), 0)
            self.assertEqual(calls, [(5,0,.5,40), (0,0,3,31)])
            self.assertEqual(len(output.getvalue().splitlines()), 2)
            mount.assert_called_once()

    def test_failure_or_interrupt_does_not_advance(self):
        for error, code in [(RuntimeError('Timed out'), 1), (KeyboardInterrupt(), 130)]:
            with self.subTest(error=error), patch('point.Mount'), \
                 patch.object(Controller, 'read', return_value=State(0,0,0,1,'nNG')), \
                 patch.object(Controller, 'run_pointing', side_effect=error) as run, \
                 redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                if code == 1:
                    with self.assertRaises(SystemExit): self.invoke('az,el,duration\n5,0,10\n0,0,10\n')
                else:
                    self.assertEqual(self.invoke('az,el,duration\n5,0,10\n0,0,10\n'), code)
                run.assert_called_once()

    def test_duration_settings_validate_and_preserve_defaults(self):
        settings = Settings()
        state = State(0, 0, 0, 1, 'nNG')
        for duration in (0, -1, float('nan'), float('inf'), 1e12):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                duration_settings(settings, state, (5, 0), duration)
        timed = duration_settings(settings, state, (5, 0), 10)
        self.assertEqual(timed.max_speed, .5)
        self.assertEqual(timed.timeout, settings.timeout + 10)
        self.assertEqual(settings, Settings())

    def test_deltas_use_each_measured_position(self):
        states = [State(0,-2,0,1,'nNG'), State(0,-6.99,2,3,'nNG')]
        with patch('point.Mount'), patch.object(Controller, 'read', side_effect=states), \
             patch.object(Controller, 'run_pointing', return_value=states[-1]) as run, redirect_stdout(StringIO()):
            self.assertEqual(self.invoke('az,el,duration\n5,0,10\n-5,0,10\n', '--delta'), 0)
            self.assertAlmostEqual(run.call_args_list[0].kwargs['azimuth'], 7)
            self.assertAlmostEqual(run.call_args_list[1].kwargs['azimuth'], 1.99)

    def test_delta_invalid_input_and_dry_run_never_open_port(self):
        for text, options in [('az,el,duration\nnan,0,1\n', []),
                              ('az,el,duration\n0,inf,1\n', []),
                              ('az,el,duration\n0,0,-1\n', []),
                              ('az,el,duration\n5,0,1\n', ['--dry-run'])]:
            with self.subTest(text=text, options=options), patch('point.Mount') as mount, redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit): self.invoke(text, '--delta', *options)
                mount.assert_not_called()

    def test_delta_ik_branch_checked_before_motion(self):
        with patch('point.Mount') as mount, patch.object(Controller, 'read', return_value=State(0,-20,0,1,'nNG')), \
             patch.object(Controller, '_step') as step, redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit): self.invoke('az,el,duration\n100,0,10\n', '--delta')
            step.assert_not_called()
            mount.return_value.__enter__.return_value.jog.assert_not_called()
