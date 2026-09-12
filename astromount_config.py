"""Local installation defaults. Importing this module performs no hardware I/O."""
from pathlib import Path
from astromount_kinematics import Pointing

ROOT = Path(__file__).resolve().parent
PORT = '/dev/serial/by-id/usb-ZWO_Systems_ZWO_Device_123456-if00'
BASELINE = ROOT / 'baseline-2026-09-11-1032.json'
POLARITY = ROOT / 'polarity-2026-09-08.json'
FRAME = Pointing(pitch_sign=1, yaw_sign=-1)
