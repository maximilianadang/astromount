"""Shared config.json defaults; relative paths are rooted here. No hardware I/O."""
import json
from pathlib import Path
from astromount_kinematics import Pointing

ROOT = Path(__file__).resolve().parent
_config = json.loads((ROOT / 'config.json').read_text())
if any(not isinstance(_config[key], str) or not _config[key].strip() for key in ('port', 'baseline', 'polarity')):
    raise ValueError('Config port, baseline, and polarity must be nonempty strings')
PORT = _config['port']
BASELINE = ROOT / _config['baseline']
POLARITY = ROOT / _config['polarity']
FRAME = Pointing(pitch_sign=_config['pitch_sign'], yaw_sign=_config['yaw_sign'])

MOTION_SETTINGS = ROOT / 'motion-settings.json'


def motion_defaults():
    """Shared motion policy; missing files/keys fail rather than use hidden defaults."""
    return json.loads(MOTION_SETTINGS.read_text())['motion']
