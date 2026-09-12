"""Command fixed-root FRD azimuth/elevation using the configured baseline."""

import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path

from astromount import Mount
from astromount_control import Controller, Reference, Settings, Worker, require_status, duration_settings
from astromount_config import PORT, BASELINE, POLARITY, FRAME
from astromount_trajectory import spiral, trace


def main(argv=None, *, sequence=False):
    parser = argparse.ArgumentParser(description=__doc__ + ' Running this commands physical motion.')
    if sequence:
        parser.add_argument('--path', type=Path, required=True, help='CSV with header az,el,duration; absolute root-frame waypoints')
        parser.set_defaults(azimuth=0, elevation=0, speed=None, delta=False, spiral=None, duration=None)
    else:
        parser.add_argument('azimuth', type=float, help='Degrees; positive turns right')
        parser.add_argument('elevation', type=float, help='Degrees; positive raises the nose')
        parser.add_argument('--speed', type=float, help='Speed cap in deg/s, at most 3 (default: 1)')
        parser.add_argument('--delta', action='store_true', help='Add az/el to the current measured pointing')
        parser.add_argument('--spiral', type=float, metavar='RADIUS', help='Baseline-centered spiral, radius >0 and <=5 degrees; requires 0 0')
        parser.add_argument('--duration', type=float, help='Nominal point-move seconds instead of --speed; spiral duration (default: 120)')
        parser.add_argument('--turns', type=float, default=2, help='Spiral turns per outward/inward leg (default: 2)')
    parser.add_argument('--timeout', type=float, default=30, help='Arrival timeout in seconds (default: 30)')
    parser.add_argument('--dry-run', action='store_true', help='Validate inputs without opening the device')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--baseline', type=Path, default=BASELINE)
    parser.add_argument('--polarity', type=Path, default=POLARITY)
    args = parser.parse_args(argv)
    try:
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError('Duration must be finite and positive')
        if args.spiral is None and args.duration is not None and args.speed is not None:
            raise ValueError('Use either --duration or --speed for point moves')
        if args.delta and args.spiral is not None:
            raise ValueError('--delta cannot be combined with --spiral')
        if args.dry_run and args.spiral is None and (args.delta or args.duration is not None):
            raise ValueError('--delta and point-move --duration require a live read; unavailable with --dry-run')
        if args.speed is None: args.speed = 1
        if not 0 < args.speed <= 3:
            raise ValueError('Speed must be greater than zero and at most 3 deg/s')
        settings = Settings(max_speed=args.speed, margin=1.25, timeout=args.timeout)
        frame = FRAME
        if not all(map(math.isfinite, (args.azimuth, args.elevation))):
            raise ValueError('Azimuth and elevation must be finite')
        if not args.delta: frame.inverse(args.azimuth, args.elevation)
        if args.spiral is not None:
            if args.duration is None: args.duration = 120
            if (args.azimuth, args.elevation) != (0, 0) or not math.isfinite(args.duration) or args.duration <= 0:
                raise ValueError('Spiral requires center 0 0 and finite positive duration')
            # The generator bounds angular radius to 5°, hence both joint offsets to 5°.
            spiral(0, args.spiral, args.turns)
            settings = replace(settings, timeout=args.duration + 2*args.timeout)
        reference = Reference.from_baseline(args.baseline)
        directions = json.loads(args.polarity.read_text())['positive_directions']
        control = Controller(None, reference, positive_directions=directions, settings=settings)
        waypoints = [(args.azimuth, args.elevation, args.duration)]
        if sequence:
            with args.path.open(newline='') as source:
                rows = csv.reader(source, strict=True)
                if next(rows, None) != ['az', 'el', 'duration']:
                    raise ValueError('CSV header must be az,el,duration')
                waypoints = [tuple(map(float, row)) for row in rows]
            if not waypoints: raise ValueError('CSV must contain at least one waypoint')
            for az, el, duration in waypoints:
                if not math.isfinite(duration) or duration <= 0:
                    raise ValueError('Duration must be finite and positive')
                control._target(*frame.inverse(az, el))
        if args.dry_run:
            print('Inputs and IK validated; no device opened. Physical tracking accuracy is not validated.')
            return 0
        with Mount(args.port, timeout=.3) as mount:
            control.mount = mount
            for args.azimuth, args.elevation, args.duration in waypoints:
                if args.spiral is not None:
                    with Worker(control) as worker:
                        final = trace(worker, frame, duration=args.duration, radius=args.spiral,
                                      turns=args.turns, settle_timeout=args.timeout)
                if args.spiral is None and (args.delta or args.duration is not None):
                    current = control.read()
                    require_status(current.status, stationary=True)
                    if args.delta:
                        azimuth, elevation = current.pointing(frame)
                        args.azimuth += azimuth
                        args.elevation += elevation
                    target = frame.inverse(args.azimuth, args.elevation)
                    if args.duration is not None:
                        control.settings = duration_settings(settings, current, target, args.duration)
                if args.spiral is None:
                    final = control.run_pointing(frame, azimuth=args.azimuth, elevation=args.elevation)
                print(json.dumps({'joint_offsets_degrees': final.angles,
                                  'estimated_azel_degrees': final.pointing(frame), 'status': final.status}))
        return 0
    except KeyboardInterrupt:
        print('Interrupted; any active controller run attempted to stop.')
        return 130
    except (OSError, RuntimeError, ValueError, KeyError, csv.Error) as exc:
        parser.exit(1, f'Error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
