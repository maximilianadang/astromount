"""Shared CSV waypoint input; no hardware dependencies."""
import csv
import math
from pathlib import Path


def read_waypoints(path):
    with path.open(newline='') as source:
        rows = csv.reader(source, strict=True)
        if next(rows, None) != ['az', 'el', 'duration']: raise ValueError('CSV header must be az,el,duration')
        points = [tuple(map(float, row)) for row in rows]
    if not points: raise ValueError('CSV must contain at least one waypoint')
    for az, el, duration in points:
        if not all(map(math.isfinite, (az, el, duration))) or duration <= 0:
            raise ValueError('CSV requires finite az/el and positive finite duration')
    return points


def add_arguments(parser, *, sequence=True, path_required=True, defaults=None, delta_help=None):
    """Pointing CLI options shared by sequence.py and acquisition wrappers."""
    from astromount_config import PORT, BASELINE, POLARITY, motion_defaults
    defaults = motion_defaults() if defaults is None else defaults
    if sequence:
        parser.add_argument('--path', type=Path, required=path_required, help='CSV with header az,el,duration; absolute unless --delta')
        parser.set_defaults(azimuth=0, elevation=0, speed=None, spiral=None, duration=None)
    parser.add_argument('--delta', action='store_true', help=delta_help or 'Add each az/el to the current measured root-frame pointing')
    for name in MOTION_FIELDS:
        parser.add_argument('--' + name.replace('_', '-'), type=int if name == 'settle_samples' else float,
                            default=defaults[name], help=f'{name} (motion-settings.json default: {defaults[name]})')
    parser.add_argument('--dry-run', action='store_true', help='Validate inputs without opening the device')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--polarity', type=Path, default=POLARITY)


MOTION_FIELDS = ('kp', 'max_speed', 'speed_limit', 'deadband',
                 'period', 'max_sample_age', 'timeout', 'progress_timeout', 'settle_samples')


def controller_values(args):
    return {key: getattr(args, key) for key in MOTION_FIELDS}


def run_waypoint(control, frame, settings, az, el, duration=None, *, delta=False, cancel=None):
    """Resolve each relative target from a fresh reading, then use shared control."""
    from astromount_control import require_status, duration_settings
    control.settings = settings
    if delta or duration is not None:
        current = control.read()
        require_status(current.status, stationary=True)
        if delta:
            azimuth, elevation = current.pointing(frame)
            az, el = az + azimuth, el + elevation
        if duration is not None:
            control.settings = duration_settings(settings, current, frame.inverse(az, el), duration)
    settings.validate_target(*frame.inverse(az, el))
    return control.run_pointing(frame, azimuth=az, elevation=el, **({'cancel': cancel} if cancel is not None else {}))
