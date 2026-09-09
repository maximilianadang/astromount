"""Explicit small hardware commissioning test; out and back require separate runs."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import monotonic, sleep

from astromount import Mount
from astromount_control import Controller, Reference, Settings, ControlError


def main(stage, degrees=.1):
    if degrees not in (.1, 1.0):
        raise ValueError('Commissioning displacement must be 0.1 or 1 degree')
    record = Path('position-test-2026-09-08.json' if degrees == .1 else 'position-test-1deg-2026-09-08.json')
    if stage == 'out' and record.exists():
        raise RuntimeError('Test record already exists; do not overwrite its reference')
    data = json.loads(record.read_text()) if stage == 'back' else {}
    if stage == 'back' and (not data.get('out', {}).get('success') or 'back' in data):
        raise RuntimeError('Require one successful outward test and no previous return')
    reference = Reference.from_baseline('baseline-2026-09-08T212838Z.json')
    polarity = json.loads(Path('polarity-2026-09-08.json').read_text())['positive_directions']
    settings = Settings(max_speed=3, margin=1.25, timeout=10 if degrees == .1 else 15)
    envelope = .15 if degrees == .1 else 1.25
    with Mount('/dev/serial/by-id/usb-ZWO_Systems_ZWO_Device_123456-if00', timeout=.3) as mount:
        control = Controller(mount, reference, positive_directions=polarity, settings=settings)
        state = control.read()
        if 'N' not in state.status or mount.tracking():
            raise ControlError('Require stationary mount and tracking off')
        origin = tuple(data['origin']) if data else state.angles
        if stage == 'back' and any(abs(a-b) > .02 for a,b in zip(state.angles, data['out']['final']['angles'])):
            raise ControlError('Mount changed since outward test')
        target = (origin[0], origin[1] + (degrees if stage == 'out' else 0))
        data.setdefault('origin', origin)
        data.setdefault('settings', asdict(settings))
        data.setdefault('local_envelope_degrees', envelope)
        result = data[stage] = {'target': target, 'samples': [], 'success': False}
        started = monotonic()

        def observe(s):
            delta = tuple(a-b for a,b in zip(s.angles, origin))
            if abs(delta[0]) > .02 or abs(delta[1]) > envelope:
                raise ControlError('Outside small-test envelope')
            row = {'elapsed': monotonic()-started, 'angles': s.angles, 'status': s.status}
            result['samples'].append(row)
            print(json.dumps(row), flush=True)

        try:
            final = control.run(ra_degrees=target[0], dec_degrees=target[1], on_sample=observe)
            for _ in range(3):
                sleep(.1)
                final = control.read()
                observe(final)
                if 'N' not in final.status or any(abs(a-b) > settings.deadband for a,b in zip(final.angles,target)):
                    raise ControlError('Arrival or stop not confirmed')
            result['final'] = {'angles': final.angles, 'status': final.status, 'tracking': mount.tracking()}
            result['success'] = not result['final']['tracking']
            print(json.dumps({'stage': stage, 'success': result['success'], 'final': result['final']}), flush=True)
        except BaseException as exc:
            control._stop()
            result['error'] = repr(exc)
            raise
        finally:
            record.write_text(json.dumps(data, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('out', 'back'))
    parser.add_argument('--degrees', type=float, choices=(.1, 1.0), default=.1)
    args = parser.parse_args()
    main(args.stage, args.degrees)
