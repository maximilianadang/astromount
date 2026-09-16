import ast
import importlib.util
from contextlib import redirect_stdout
from io import StringIO
import runpy
from unittest import TestCase
from unittest.mock import patch

from astromount_config import BASELINE, ROOT
from astromount_control import Reference


class MeasureTests(TestCase):
    def test_import_performs_no_io(self):
        with patch('astromount.Mount') as mount, patch.object(Reference, 'from_baseline') as baseline, \
             redirect_stdout(StringIO()) as output:
            spec = importlib.util.spec_from_file_location('measure', ROOT / 'measure.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.assertTrue(callable(module.main))
            mount.assert_not_called()
            baseline.assert_not_called()
            self.assertEqual(output.getvalue(), '')

    def test_baseline_azel_uses_only_getters(self):
        with patch('astromount.Mount') as cls, redirect_stdout(StringIO()) as output:
            mount = cls.return_value.__enter__.return_value
            mount.joint_sample.return_value = (*Reference.from_baseline(BASELINE).angles, 'nNG')
            runpy.run_path(str(ROOT / 'measure.py'), run_name='__main__')
            self.assertEqual(ast.literal_eval(output.getvalue().splitlines()[2]),
                             {'status': 'nNG', 'estimated_azel_degrees': (0, 0)})
            self.assertEqual([c[0] for c in mount.method_calls],
                             ['identity', 'position', 'joint_sample', 'tracking'])
