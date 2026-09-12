"""Save the current physical zero pose as a new baseline; never commands motion.

First restore the agreed pose: telescope forward, upper motor axis vertical,
fixed manual adjustments unchanged. This captures coordinates, not physical
homing or a new kinematic calibration. Existing motor/FRD signs are not measured.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from time import monotonic, sleep

from astromount import Mount
from astromount_control import Reference, require_status
from astromount_config import PORT


def capture(mount):
    data = {'captured_at_utc': datetime.now(timezone.utc).isoformat(),
            'identity': mount.identity(), 'interpretation': __doc__, 'samples': []}
    origin = None
    for _ in range(3):
        start = monotonic()
        with mount._lock:
            queries = {'status_before': {'command': ':GU#', 'response': mount.status()},
                       **mount.coordinate_queries(), 'status_after': {'command': ':GU#', 'response': mount.status()}}
        if monotonic() - start > .25:
            raise RuntimeError('Position sample took too long; no baseline saved')
        for key in ('status_before', 'status_after'):
            require_status(queries[key]['response'], stationary=True)
        reference = Reference.from_queries(queries)
        origin = origin or reference
        if any(abs(v) > .01 for v in origin.offsets(*reference.angles)):
            raise RuntimeError('Position changed during capture; no baseline saved')
        data['samples'].append(queries)
        sleep(.2)
    data['queries'] = data['samples'][-1]
    data['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New baseline JSON file (must not already exist)')
    parser.add_argument('--port', default=PORT)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f'Refusing to overwrite {args.output}')
        with Mount(args.port, timeout=.3) as mount:
            data = capture(mount)
        data['port'] = args.port
        with args.output.open('x') as file:
            file.write(json.dumps(data, indent=2) + '\n')
        print(f'Saved {args.output.resolve()}. No motion or firmware coordinate changes sent.')
        print('Select this file with point.py --baseline PATH; the old default is unchanged.')
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f'Error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
