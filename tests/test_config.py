import json
import argparse
from dataclasses import asdict, replace
import runpy
from unittest import TestCase
from unittest.mock import patch

from astromount_config import ROOT
from astromount_config import motion_defaults, MOTION_SETTINGS
from astromount_control import Settings, State, duration_settings
from astromount_sequence import add_arguments, controller_values


class ConfigTests(TestCase):
    def test_controller_and_cli_share_every_default(self):
        self.assertEqual(MOTION_SETTINGS, ROOT / 'motion-settings.json')
        defaults = motion_defaults()
        parser = argparse.ArgumentParser()
        add_arguments(parser)
        values = controller_values(parser.parse_args(['--path', 'unused.csv']))
        self.assertEqual(values, asdict(Settings()))
        for key, value in values.items(): self.assertEqual(value, defaults[key])

    def test_changed_motion_defaults_and_explicit_override(self):
        defaults = dict(motion_defaults(), deadband=.02, timeout=17, max_speed=.2, speed_limit=.5)
        parser = argparse.ArgumentParser()
        with patch('astromount_config.motion_defaults', return_value=defaults): add_arguments(parser)
        values = controller_values(parser.parse_args(['--path', 'unused.csv', '--timeout', '19']))
        self.assertEqual(values['deadband'], .02)
        self.assertEqual(values['timeout'], 19)
        settings = Settings(**values)
        timed = duration_settings(settings, State(0,0,0,1,'nNG'), (5,0), 1)
        self.assertEqual(timed.max_speed, .5)
        with self.assertRaises(ValueError): replace(settings, max_speed=.6)

    def test_missing_motion_field_has_no_fallback(self):
        defaults = motion_defaults()
        del defaults['deadband']
        with patch('astromount_config.motion_defaults', return_value=defaults), self.assertRaises(KeyError):
            add_arguments(argparse.ArgumentParser())

    def load(self, data):
        with patch('pathlib.Path.read_text', return_value=json.dumps(data)), patch('astromount.Mount') as mount:
            config = runpy.run_path(str(ROOT / 'astromount_config.py'))
            mount.assert_not_called()
            return config

    def test_values_and_relative_absolute_paths(self):
        config = self.load(dict(port='/dev/example', baseline='new.json', polarity='/tmp/signs.json', pitch_sign=-1, yaw_sign=1))
        self.assertEqual(config['PORT'], '/dev/example')
        self.assertEqual(config['BASELINE'], ROOT / 'new.json')
        self.assertEqual(str(config['POLARITY']), '/tmp/signs.json')
        self.assertEqual((config['FRAME'].pitch_sign, config['FRAME'].yaw_sign), (-1, 1))

    def test_invalid_or_missing_values_fail(self):
        data = dict(port='/dev/example', baseline='new.json', polarity='signs.json', pitch_sign=1, yaw_sign=-1)
        for key, value in [('port', ''), ('baseline', None), ('polarity', ' '), ('pitch_sign', 0), ('yaw_sign', 2)]:
            with self.subTest(key=key), self.assertRaises(ValueError): self.load({**data, key: value})
        with self.assertRaises(KeyError): self.load({})

    def test_missing_or_malformed_file_fails(self):
        with patch('pathlib.Path.read_text', side_effect=FileNotFoundError), self.assertRaises(FileNotFoundError):
            runpy.run_path(str(ROOT / 'astromount_config.py'))
        with patch('pathlib.Path.read_text', return_value='{'), self.assertRaises(json.JSONDecodeError):
            runpy.run_path(str(ROOT / 'astromount_config.py'))
