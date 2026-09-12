from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import Mock, patch

import commission
from test_control import Clock, Plant


class CommissionTests(TestCase):
    def setUp(self):
        self.clock, self.plant = Clock(), None
        self.plant = Plant(self.clock)
        self.plant.identity = lambda: {'model': 'AM5N', 'firmware': 'simulated'}
        self.plant.tracking = lambda: False
        original = self.plant.jog
        def jog(direction, **kwargs):
            original(direction, **kwargs)
            if direction in ('north', 'south'): self.plant.velocity[1] *= -1
        self.plant.jog = jog
        for module in ('commission', 'astromount_control'):
            for name, replacement in (('monotonic', lambda: self.clock.now), ('sleep', self.clock.sleep)):
                p = patch(f'{module}.{name}', replacement)
                p.start(); self.addCleanup(p.stop)
        self.timer_patch = patch('commission.Timer')
        self.timer = self.timer_patch.start()
        self.addCleanup(self.timer_patch.stop)
        self.mount_patch = patch('commission.Mount')
        self.mount = self.mount_patch.start()
        self.addCleanup(self.mount_patch.stop)
        self.mount.return_value.__enter__.return_value = self.plant

    def run_cli(self, args):
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            return commission.main(args)

    def test_help_and_invalid_commands_do_not_open_device(self):
        for args in (['--help'], ['jog','--help'], ['simultaneous','--help'], ['pointing','--help'],
                     ['spiral','--help'], ['position','--help'], ['jog','up'], ['position','out','--degrees','5']):
            with self.assertRaises(SystemExit): self.run_cli(args)
        self.mount.assert_not_called()

    def test_single_jog_stops_and_preserves_other_axis(self):
        self.assertEqual(self.run_cli(['jog','west']), 0)
        self.assertGreater(self.plant.q[0], .08)
        self.assertLessEqual(self.plant.q[0], .15)
        self.assertEqual(self.plant.q[1], 0)
        self.assertFalse(any(self.plant.velocity))
        self.timer.assert_called_once()
        self.assertEqual(self.timer.call_args.args[0], 2)
        self.timer.return_value.cancel.assert_called_once()
        self.timer.return_value.join.assert_called_once()

    def test_live_rate_updates_have_no_intermediate_stop(self):
        self.plant._command = Mock()
        self.assertEqual(self.run_cli(['jog','south','--live-rate-test','--repeat-direction']), 0)
        self.assertEqual([c.args[0] for c in self.plant._command.call_args_list],
                         [':Rv23.93#', ':Ms#', ':Rv5.98#', ':Ms#'])
        self.assertEqual(self.plant.commands, [('south', .05), ('stop',)])
        self.assertEqual(self.timer.call_args.args[0], 5)

    def test_both_simultaneous_sequences_and_local_stop_updates(self):
        self.assertEqual(self.run_cli(['simultaneous']), 0)
        self.assertEqual(self.plant.commands,
                         [('west',.05), ('south',.1), ('stop','west'), ('stop',),
                          ('north',.05), ('east',.1), ('stop','north'), ('stop',)])
        self.plant.commands.clear()
        self.assertEqual(self.run_cli(['simultaneous','--rate-updates']), 0)
        self.assertEqual(self.plant.commands,
                         [('west',.05), ('south',.05), ('stop','west'), ('west',.1),
                          ('stop','west'), ('west',.025), ('stop',),
                          ('north',.05), ('east',.05), ('stop','north'), ('north',.1),
                          ('stop','north'), ('north',.025), ('stop',)])
        self.assertFalse(any(self.plant.velocity))

    def test_pointing_uses_production_controller_and_converges(self):
        self.assertEqual(self.run_cli(['pointing']), 0)
        azel = commission.FRAME.forward(*self.plant.q)
        for value in azel: self.assertAlmostEqual(value, 1, delta=.02)
        self.assertFalse(any(self.plant.velocity))

    def test_spiral_uses_worker_and_active_baseline(self):
        with patch('commission.Worker') as worker, patch('commission.trace') as trace, \
             patch('commission.Reference.from_baseline', return_value=commission.Reference(-90,90)) as load:
            self.assertEqual(self.run_cli(['spiral']), 0)
            load.assert_called_once_with(commission.BASELINE)
            trace.assert_called_once_with(worker.return_value.__enter__.return_value, commission.FRAME)
            worker.return_value.__exit__.assert_called_once()

    def test_out_back_records_preserved_and_replays_rejected(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'position.json'
            args = ['--record', str(path)]
            self.assertEqual(self.run_cli(['position','out', *args]), 0)
            data = json.loads(path.read_text())
            self.assertTrue(data['out']['success'])
            before = path.read_bytes()
            self.mount.reset_mock()
            with self.assertRaises(SystemExit): self.run_cli(['position','out', *args])
            self.mount.assert_not_called()
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(self.run_cli(['position','back', *args]), 0)
            self.assertTrue(json.loads(path.read_text())['back']['success'])
            self.assertLessEqual(abs(self.plant.q[1]), .01)
            with self.assertRaises(SystemExit): self.run_cli(['position','back', *args])

    def test_record_mismatch_and_changed_endpoint_do_not_move(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'position.json'
            self.run_cli(['position','out','--record',str(path)])
            self.plant.commands.clear()
            with self.assertRaises(SystemExit):
                self.run_cli(['position','back','--degrees','1','--record',str(path)])
            self.plant.q[1] += .1
            with self.assertRaises(SystemExit): self.run_cli(['position','back','--record',str(path)])
            self.assertEqual(self.plant.commands, [])

    def test_watchdog_expired_before_dispatch_never_jogs(self):
        self.timer.side_effect = lambda seconds, stop: Mock(start=stop)
        with self.assertRaises(SystemExit): self.run_cli(['jog','west'])
        self.assertTrue(all(c[0] == 'stop' for c in self.plant.commands))

    def test_guard_failure_stops_and_closed_loop_write_failure_stops(self):
        original = self.plant.joint_sample
        def runaway():
            if any(self.plant.velocity): self.plant.q[0] += 1
            return original()
        self.plant.joint_sample = runaway
        with self.assertRaises(SystemExit): self.run_cli(['jog','west'])
        self.assertFalse(any(self.plant.velocity))
        self.plant.q, self.plant.joint_sample = [0,0], original
        original_jog = self.plant.jog
        def fail(direction, **kwargs):
            original_jog(direction, **kwargs)
            raise OSError('Simulated dispatch failure')
        self.plant.jog = fail
        with self.assertRaises(SystemExit): self.run_cli(['pointing'])
        self.assertFalse(any(self.plant.velocity))
