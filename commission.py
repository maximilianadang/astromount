"""Supervised mount experiments. All subcommands except --help command motion."""
import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import math
from pathlib import Path
from threading import Event, Timer
from time import monotonic, sleep

from astromount import Mount
from astromount_config import PORT, BASELINE, POLARITY, FRAME, ROOT
from astromount_control import Controller, ControlError, Reference, Settings, State, Worker, require_status
from astromount_trajectory import trace


def emit(value): print(json.dumps(value), flush=True)


class Recorder(Controller):
    """Shared preflight/telemetry; raw experiments bypass the position controller."""
    def __init__(self, *args, raw=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.raw, self.rows, self.origin, self.limits = raw, [], None, (math.inf, math.inf)
        self.started = monotonic()

    def read(self):
        old = self._previous
        if self.raw:
            start = monotonic()
            h, d, status = self.mount.joint_sample()
            state = State(*self.reference.offsets(h, d, old.angles if old else (0, 0)), start, monotonic(), status)
            require_status(status)
            if state.finished_at-start > .25: raise ControlError('Stale position')
            if any(abs(q) >= self.settings.excursion-self.settings.margin for q in state.angles):
                raise ControlError('Outside commissioning envelope')
            if old and any(abs(a-b) > .15*(state.finished_at-old.finished_at+.25)+.02
                           for a,b in zip(state.angles, old.angles)):
                raise ControlError('Overspeed/discontinuity guard')
            self._previous = state
        else:
            state = super().read()
        if self.origin and any(abs(a-b) >= limit for a,b,limit in zip(state.angles, self.origin, self.limits)):
            raise ControlError('Outside small-test envelope')
        row = dict(asdict(state), angles=state.angles, estimated_azel=state.pointing(FRAME),
                   elapsed=state.finished_at-self.started)
        if not self.rows or int(row['elapsed']/10) != int(self.rows[-1]['elapsed']/10): emit({'progress': row})
        self.rows.append(row)
        return state

    def preflight(self):
        state = self.read()
        require_status(state.status, stationary=True)
        if self.mount.tracking(): raise ControlError('Require tracking off')
        self.origin = state.angles
        emit({'origin': self.origin, 'identity': self.mount.identity()})
        return state

    def verify_stop(self, target=None):
        previous = None
        for _ in range(3):
            sleep(.15)
            state = self.read()
            require_status(state.status, stationary=True)
            if previous and any(abs(a-b) > .02 for a,b in zip(state.angles, previous)):
                raise ControlError('Stop not confirmed: USE E-STOP')
            if target and any(abs(a-b) > self.settings.deadband for a,b in zip(state.angles, target)):
                raise ControlError('Arrival not confirmed')
            previous = state.angles
        if self.mount.tracking(): raise ControlError('Tracking unexpectedly enabled')
        emit({'stopped': state.angles, 'estimated_azel': state.pointing(FRAME)})
        return state


@contextmanager
def timed_motion(control, seconds):
    expired = Event()
    def stop():
        expired.set()
        control._stop()
    timer = Timer(seconds, stop)
    try:
        timer.start()
        yield expired
    finally:
        timer.cancel()
        try: stop()
        finally: timer.join()


def raw_trial(c, first, second=None, *, speed=.05, live=False, repeat=False, updates=False):
    """One phase engine for single-axis jogs and both simultaneous-rate experiments."""
    axis = 0 if first in ('east', 'west') else 1
    c.limits = (.6, .6) if second else tuple((.4 if live else .15) if i == axis else .02 for i in range(2))
    rates, phases = (speed, .1, .025), 3 if live or second else 1
    with timed_motion(c, 6 if second else 5 if live else 2) as expired:
        for phase in range(phases):
            with c.mount._lock:
                if expired.is_set(): raise ControlError('Watchdog expired')
                if phase == 0:
                    c.mount.jog(first, speed_degrees_s=speed)
                    if updates: c.mount.jog(second, speed_degrees_s=.05)
                elif live:
                    rate = math.floor(rates[phase]/(360/86164.0905)*100)/100
                    c.mount._command(f':Rv{rate:.2f}#', 'none')
                    if repeat: c.mount._command(f':M{first[0]}#', 'none')
                elif updates:
                    c.mount.stop(first)
                    c.mount.jog(first, speed_degrees_s=rates[phase])
                elif phase == 1: c.mount.jog(second, speed_degrees_s=.1)
                else: c.mount.stop(first)
            start, window = monotonic(), []
            while monotonic()-start < (1.5 if phases == 3 else 2) and not expired.is_set():
                state = c.read()
                if state.finished_at-start > .35: window.append(state)
                if phases == 1 and abs(state.angles[axis]-c.origin[axis]) >= .09: break
                sleep(.08 if second else .05)
            if len(window) < 2: raise ControlError('Insufficient samples for rate verification')
            a,b = window[0],window[-1]
            emit({'first': first, 'second': second, 'phase': phase,
                  'measured_deg_s': [(y-x)/(b.finished_at-a.finished_at) for x,y in zip(a.angles,b.angles)]})
    c.verify_stop()


def position_trial(c, args, data):
    origin = tuple(data.get('origin', c.origin))
    envelope = .15 if args.degrees == .1 else 1.25
    if args.stage == 'back' and any(abs(a-b) > .02 for a,b in zip(c.origin, data['out']['final']['angles'])):
        raise ControlError('Mount changed since outward test')
    c.origin, c.limits = origin, (.02, envelope)
    target = (origin[0], origin[1]+(args.degrees if args.stage == 'out' else 0))
    data.update(origin=origin, degrees=args.degrees, reference=c.reference.angles,
                settings=asdict(c.settings), local_envelope_degrees=envelope)
    result = data[args.stage] = {'target': target, 'samples': c.rows, 'success': False}
    try:
        c.run(ra_degrees=target[0], dec_degrees=target[1])
        final = c.verify_stop(target)
        result.update(success=True, final={'angles': final.angles, 'status': final.status, 'tracking': False})
    except BaseException as exc:
        result['error'] = repr(exc)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--baseline', type=Path, help='Override baseline; historical tests default to September 8, spiral to active config')
    commands = parser.add_subparsers(dest='command', required=True)
    jog = commands.add_parser('jog', help='Bounded single-axis jog or live-rate test')
    jog.add_argument('direction', choices=('east', 'west', 'north', 'south'))
    jog.add_argument('--speed', type=float, choices=(.025, .05), default=.05)
    jog.add_argument('--live-rate-test', action='store_true')
    jog.add_argument('--repeat-direction', action='store_true')
    both = commands.add_parser('simultaneous', help='Unequal concurrent rates and axis-local stopping')
    both.add_argument('--rate-updates', action='store_true')
    commands.add_parser('pointing', help='Combined +1/+1 az/el at .1 deg/s; no automatic return')
    commands.add_parser('spiral', help='Logged production spiral: 5 degrees, 120 seconds, 1 deg/s cap')
    position = commands.add_parser('position', help='Separate out/back joint-position commissioning runs')
    position.add_argument('stage', choices=('out', 'back'))
    position.add_argument('--degrees', type=float, choices=(.1, 1.), default=.1)
    position.add_argument('--record', type=Path, help='Out/back record; refuses to overwrite a prior outward run')
    args = parser.parse_args(argv)
    c, record, data = None, None, {}
    try:
        baseline = args.baseline or (BASELINE if args.command == 'spiral' else ROOT/'baseline-2026-09-08T212838Z.json')
        reference = Reference.from_baseline(baseline)
        directions = json.loads(POLARITY.read_text())['positive_directions']
        settings = Settings(max_speed=1, margin=1.25, timeout=180) if args.command == 'spiral' else Settings()
        if args.command == 'pointing': settings = Settings(max_speed=.1, excursion=2, timeout=30)
        if args.command in ('jog', 'simultaneous'):
            settings = Settings(excursion=22.25 if args.command == 'jog' else 21.25)
        if args.command == 'position':
            settings = Settings(max_speed=3, margin=1.25, timeout=10 if args.degrees == .1 else 15)
            path = args.record or Path('position-test-2026-09-08.json' if args.degrees == .1 else 'position-test-1deg-2026-09-08.json')
            if args.stage == 'back':
                data = json.loads(path.read_text())
                if not data.get('out', {}).get('success') or 'back' in data:
                    raise ControlError('Require one successful outward test and no previous return')
                if data.get('degrees', args.degrees) != args.degrees or tuple(data.get('reference', reference.angles)) != reference.angles:
                    raise ControlError('Record degrees/reference do not match this invocation')
            record = path.open('x' if args.stage == 'out' else 'r+')
        emit({'experiment': args.command, 'baseline': str(baseline), 'settings': asdict(settings)})
        with Mount(args.port, timeout=.3) as mount:
            c = Recorder(mount, reference, positive_directions=directions, settings=settings,
                         raw=args.command in ('jog', 'simultaneous'))
            c.preflight()
            if args.command == 'jog':
                raw_trial(c, args.direction, speed=args.speed, live=args.live_rate_test, repeat=args.repeat_direction)
            elif args.command == 'simultaneous':
                for first, second in (('west', 'south'), ('north', 'east')):
                    raw_trial(c, first, second, updates=args.rate_updates)
            elif args.command == 'pointing':
                if any(abs(q) > .25 for q in c.origin): raise ControlError('Require starting near baseline')
                c.run_pointing(FRAME, azimuth=1, elevation=1)
                c.verify_stop()
            elif args.command == 'spiral':
                with Worker(c) as worker: trace(worker, FRAME)
                c.verify_stop()
            else: position_trial(c, args, data)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        parser.exit(1, f'Error: {exc}\n')
    finally:
        if record:
            try:
                record.seek(0)
                record.write(json.dumps(data, indent=2)+'\n')
                record.truncate()
            finally: record.close()
        if c:
            for row in c.rows: emit(row)


if __name__ == '__main__': raise SystemExit(main())
