"""Command fixed-root FRD azimuth/elevation using the configured baseline."""

import argparse
import csv
from astromount_sequence import read_waypoints, add_arguments, run_waypoint, controller_values
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

from astromount import Mount
from astromount_control import Controller, Reference, Settings, Worker, require_status, duration_settings
from astromount_config import PORT, BASELINE, POLARITY, FRAME
from astromount_trajectory import spiral, trace, sweep
from astromount_logging import sweep_log


def main(argv=None, *, sequence=False, continuous=False):
    description = 'Continuous CSV az/el sweep; settles only at the final target.' if continuous else __doc__
    parser = argparse.ArgumentParser(description=description + ' Running this commands physical motion.')
    if not sequence:
        parser.add_argument('azimuth', type=float, help='Degrees; positive turns right')
        parser.add_argument('elevation', type=float, help='Degrees; positive raises the nose')
        parser.add_argument('--speed', type=float, help='Override configured default speed in deg/s; cannot exceed speed-limit')
        parser.add_argument('--spiral', type=float, metavar='RADIUS', help='Baseline-centered spiral, radius >0 and <=5 degrees; requires 0 0')
        parser.add_argument('--duration', type=float, help='Nominal point-move seconds instead of --speed; spiral duration (default: 120)')
        parser.add_argument('--turns', type=float, default=2, help='Spiral turns per outward/inward leg (default: 2)')
    add_arguments(parser, sequence=sequence,
                  delta_help='Accumulate deltas from initial measured pointing and planned endpoints' if continuous else None)
    args = parser.parse_args(argv)
    try:
        if args.spiral is not None:
            if args.delta or (args.azimuth, args.elevation) != (0, 0):
                raise ValueError('Spiral requires absolute center 0 0; --delta is unavailable')
            if args.duration is None: args.duration = 120
            spiral(0, args.spiral, args.turns)
        else:
            if args.duration is not None and args.speed is not None:
                raise ValueError('Use either --duration or --speed for point moves')
            if args.dry_run and (args.delta or args.duration is not None):
                raise ValueError('--delta and point-move --duration require a live read; unavailable with --dry-run')
        values = controller_values(args)
        if args.speed is not None: values['max_speed'] = args.speed
        settings = Settings(**values)
        frame = FRAME
        if args.spiral is not None:
            settings = replace(settings, timeout=args.duration + 2*args.timeout)
        reference = Reference.from_baseline(args.baseline)
        directions = json.loads(args.polarity.read_text())['positive_directions']
        waypoints = [(args.azimuth, args.elevation, args.duration)]
        if sequence: waypoints = read_waypoints(args.path)
        for az, el, duration in waypoints:
            if duration is not None and (not math.isfinite(duration) or duration <= 0):
                raise ValueError('Duration must be finite and positive')
            if not all(map(math.isfinite, (az, el))):
                raise ValueError('Azimuth and elevation must be finite')
            if not args.delta: settings.validate_target(*frame.inverse(az, el))
        if args.dry_run:
            print('Inputs and IK validated; no device opened. Physical tracking accuracy is not validated.')
            return 0
        with Mount(args.port, timeout=.3) as mount:
            control = Controller(mount, reference, positive_directions=directions, settings=settings)
            for az, el, duration in (waypoints[:1] if continuous else waypoints):
                if continuous:
                    with sweep_log(dict(waypoints=waypoints, delta=args.delta, port=args.port,
                                        baseline=str(args.baseline), reference=asdict(reference),
                                        positive_directions=directions, frame=asdict(frame), settings=asdict(settings))) as log:
                        final = sweep(control, frame, waypoints, delta=args.delta, log=log)
                elif args.spiral is not None:
                    with Worker(control) as worker:
                        final = trace(worker, frame, duration=duration, radius=args.spiral,
                                      turns=args.turns, settle_timeout=args.timeout)
                else:
                    final = run_waypoint(control, frame, settings, az, el, duration, delta=args.delta)
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
