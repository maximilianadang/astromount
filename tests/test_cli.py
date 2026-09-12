from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from unittest import TestCase
from unittest.mock import patch

import point
from astromount_control import State
from astromount_config import BASELINE, PORT, FRAME
from pathlib import Path


class CLITests(TestCase):
    def test_shared_defaults_and_explicit_baseline_override(self):
        with patch('point.Mount') as mount, patch('point.Reference.from_baseline') as load, redirect_stdout(StringIO()):
            point.main(['0', '0', '--dry-run'])
            load.assert_called_with(BASELINE)
            point.main(['0', '0', '--baseline', 'custom.json', '--dry-run'])
            load.assert_called_with(Path('custom.json'))
            mount.assert_not_called()
        self.assertEqual(point.PORT, PORT)
        self.assertIs(point.FRAME, FRAME)

    def test_spiral_dry_run_and_invalid_arguments_never_open_port(self):
        with patch('point.Mount') as mount, redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            self.assertEqual(point.main(['0', '0', '--spiral', '5', '--dry-run']), 0)
            for args in (['1','0','--spiral','5'], ['0','0','--spiral','6'],
                         ['0','0','--spiral','5','--duration','nan']):
                with self.assertRaises(SystemExit):
                    point.main(args)
            mount.assert_not_called()

    def test_help_and_invalid_inputs_never_open_port(self):
        with patch('point.Mount') as mount, redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            for args, code in [(['--help'], 0), (['0','0','--speed','4'], 1),
                               (['180','0'], 1), (['nan','0'], 1), (['0','0','--timeout','-1'], 1)]:
                with self.assertRaises(SystemExit) as error:
                    point.main(args)
                self.assertEqual(error.exception.code, code)
            mount.assert_not_called()

    def test_maps_signs_and_calls_existing_controller(self):
        with patch('point.Mount') as mount, patch('point.Controller') as controller, redirect_stdout(StringIO()):
            controller.return_value.run_pointing.return_value = State(0,-5,0,1,'nNG')
            self.assertEqual(point.main(['5','0']), 0)
            call = controller.return_value.run_pointing.call_args
            self.assertEqual(call.kwargs, {'azimuth': 5, 'elevation': 0})
            self.assertEqual((call.args[0].pitch_sign, call.args[0].yaw_sign), (1, -1))
            self.assertEqual(controller.call_args.kwargs['settings'].max_speed, 1)
            self.assertEqual(controller.call_args.kwargs['positive_directions'], ['west','south'])
            mount.return_value.__exit__.assert_called_once()

    def test_failure_returns_nonzero(self):
        with patch('point.Mount'), patch('point.Controller') as controller, redirect_stderr(StringIO()):
            controller.return_value.run_pointing.side_effect = RuntimeError('Feedback failed')
            with self.assertRaises(SystemExit) as error:
                point.main(['0','0'])
            self.assertEqual(error.exception.code, 1)

    def test_delta_and_duration(self):
        current = State(4, -3, 0, 1, 'nNG')
        for options, delta, duration in [(['--delta'], True, None),
                                       (['--duration', '10'], False, 10),
                                       (['--delta', '--duration', '2'], True, 2),
                                       (['--duration', '.01'], False, .01)]:
            with self.subTest(options=options), patch('point.Mount'), patch('point.Controller') as factory, redirect_stdout(StringIO()):
                control = factory.return_value
                control.read.return_value = control.run_pointing.return_value = current
                self.assertEqual(point.main(['5', '2', *options]), 0)
                az, el = current.pointing(FRAME) if delta else (0, 0)
                control.run_pointing.assert_called_once_with(FRAME, azimuth=az+5, elevation=el+2)
                if duration:
                    distance = max(abs(a-b) for a, b in zip(FRAME.inverse(az+5, el+2), current.angles))
                    self.assertAlmostEqual(control.settings.max_speed, min(3, distance/duration))
                    self.assertEqual(control.settings.timeout, duration+30)

    def test_zero_delta_duration_keeps_valid_speed(self):
        with patch('point.Mount'), patch('point.Controller') as factory, redirect_stdout(StringIO()):
            factory.return_value.read.return_value = factory.return_value.run_pointing.return_value = State(0,0,0,1,'nNG')
            self.assertEqual(point.main(['0', '0', '--delta', '--duration', '5']), 0)
            self.assertEqual(factory.return_value.settings.max_speed, 1)

    def test_new_invalid_options_do_not_open_port(self):
        with patch('point.Mount') as mount, redirect_stderr(StringIO()):
            for options in (['--duration', '0'], ['--duration', 'nan'], ['--duration', 'inf'],
                            ['--speed', '1', '--duration', '2'], ['--delta', '--spiral', '5'],
                            ['--delta', '--dry-run'], ['--duration', '2', '--dry-run']):
                with self.subTest(options=options), self.assertRaises(SystemExit):
                    point.main(['0', '0', *options])
            mount.assert_not_called()

    def test_bad_live_target_or_state_never_runs(self):
        for current, args in [(State(0,-20,0,1,'nNG'), ['5','0','--delta']),
                              (State(0,0,0,1,'nG'), ['5','0','--delta']),
                              (State(0,0,0,1,'nNG'), ['5','0','--duration','1000000'])]:
            with self.subTest(args=args, current=current), patch('point.Mount'), patch('point.Controller') as factory, redirect_stderr(StringIO()):
                factory.return_value.read.return_value = current
                with self.assertRaises(SystemExit): point.main(args)
                factory.return_value.run_pointing.assert_not_called()
