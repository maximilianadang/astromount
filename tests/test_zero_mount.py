from contextlib import redirect_stdout, redirect_stderr
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import Mock, patch
from threading import RLock

from astromount import Mount
from test_astromount import simulated
from astromount_control import Reference
import zero_mount


class ZeroMountTests(TestCase):
    def test_capture_wire_sequence_is_read_only(self):
        commands = [(b':GVP#', b'AM5N#'), (b':GV#', b'1.6.3#')]
        commands += [(b':GU#', b'nNG#'), (b':GS#', b'03:49:15#'),
                     (b':GMEQ#', b'09:49:15&+90*00:00#'), (b':GS#', b'03:49:15#'),
                     (b':GU#', b'nNG#')]*3
        with simulated(commands) as mount, patch('zero_mount.sleep'):
            data = zero_mount.capture(mount)
        self.assertEqual(Reference.from_queries(data['queries']).angles, (-90, 90))

    def mount(self, status='nNG', positions=None):
        m = Mock()
        m.identity.return_value = {'model': 'AM5N', 'firmware': '1.6.3'}
        positions = iter(positions or ['09:49:15&+90*00:00']*3)
        def query(command):
            self.assertIn(command, (':GU#', ':GS#', ':GMEQ#'))
            return next(positions) if command == ':GMEQ#' else ('03:49:15' if command == ':GS#' else status)
        m._command.side_effect = query
        m._lock = RLock()
        m.status.side_effect = lambda: query(':GU#')
        m.coordinate_queries.side_effect = lambda: Mount.coordinate_queries(m)
        return m

    def test_capture_load_roundtrip_and_no_overwrite(self):
        with TemporaryDirectory() as directory, patch('zero_mount.Mount') as cls, \
             patch('zero_mount.sleep'), redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            cls.return_value.__enter__.return_value = self.mount()
            path = Path(directory)/'baseline.json'
            self.assertEqual(zero_mount.main([str(path)]), 0)
            self.assertEqual(Reference.from_baseline(path).angles, (-90, 90))
            original = path.read_bytes()
            cls.reset_mock()
            with self.assertRaises(SystemExit):
                zero_mount.main([str(path)])
            cls.assert_not_called()
            self.assertEqual(original, path.read_bytes())

    def test_unsafe_status_motion_and_malformed_samples(self):
        with patch('zero_mount.sleep'):
            for status in ('nG', 'NG', 'nNZ', 'nNGs', 'nNGL'):
                with self.assertRaises(RuntimeError):
                    zero_mount.capture(self.mount(status))
            for readings in (['09:49:15&+90*00:00', '09:49:15&+89*59:00'], ['invalid']):
                with self.assertRaises((RuntimeError, ValueError)):
                    zero_mount.capture(self.mount(positions=readings))

    def test_slow_sample_rejected(self):
        with patch('zero_mount.monotonic', side_effect=[0, .3]):
            with self.assertRaisesRegex(RuntimeError, 'too long'):
                zero_mount.capture(self.mount())

    def test_failed_capture_leaves_no_file(self):
        with TemporaryDirectory() as directory, patch('zero_mount.Mount') as cls, redirect_stderr(StringIO()):
            cls.return_value.__enter__.return_value = self.mount('NG')
            path = Path(directory)/'baseline.json'
            with self.assertRaises(SystemExit):
                zero_mount.main([str(path)])
            self.assertFalse(path.exists())
