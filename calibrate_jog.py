"""Explicit one-axis commissioning jog; each invocation moves physical hardware."""

import argparse
import json
import math
from threading import Event, Timer
from time import monotonic, sleep

from astromount import Mount
from astromount_control import Reference


def jog(direction, live_rate_test=False, repeat_direction=False, speed=0.05):
    if speed not in (0.025, 0.05):
        raise ValueError('Commissioning start speed must be 0.025 or 0.05 deg/s')
    reference = Reference.from_baseline('baseline-2026-09-08T212838Z.json')
    axis = 0 if direction in ('east', 'west') else 1
    port = '/dev/serial/by-id/usb-ZWO_Systems_ZWO_Device_123456-if00'
    with Mount(port, timeout=0.3) as mount:
        def sample(previous=(0, 0)):
            started = monotonic()
            h, d, status = mount.joint_sample()
            q = reference.offsets(h, d, previous)
            if monotonic() - started > 0.25:
                raise RuntimeError('Stale position')
            if 'n' not in status or 'G' not in status or any(c in status for c in 'ZLSsTt'):
                raise RuntimeError(f'Unsafe status: {status}')
            if any(abs(v) >= 22 for v in q):
                raise RuntimeError('Outside commissioning envelope')
            return q, status

        origin, status = sample()
        if 'N' not in status or mount.tracking():
            raise RuntimeError('Require stationary mount with tracking off')
        print(json.dumps({'direction': direction, 'origin': origin}), flush=True)
        expired = Event()

        def stop():
            expired.set()
            try:
                if mount._serial.is_open:
                    mount.stop()
                else:
                    with Mount(port, timeout=0.3) as recovery:
                        recovery.stop()
            except Exception:
                print('STOP FAILED: USE E-STOP', flush=True)
                raise

        timer = Timer(5.0 if live_rate_test else 2.0, stop)
        previous = origin
        samples = []
        phase = 0
        rates = (speed, 0.1, 0.025)
        start = monotonic()
        try:
            # Serialize timer arming and dispatch, so stop cannot precede start.
            with mount._lock:
                timer.start()
                mount.jog(direction, speed_degrees_s=speed)
            while not expired.is_set():
                elapsed = monotonic() - start
                if live_rate_test:
                    if elapsed >= 4.5:
                        break
                    next_phase = int(elapsed / 1.5)
                    if next_phase != phase:
                        # No stop; repeat M only when explicitly requested.
                        rate = math.floor(rates[next_phase] / (360 / 86164.0905) * 100) / 100
                        with mount._lock:
                            if expired.is_set():
                                break
                            mount._command(f':Rv{rate:.2f}#', 'none')
                            if repeat_direction:
                                mount._command(f':M{direction[0]}#', 'none')
                        phase = next_phase
                        print(json.dumps({'speed_update': rates[phase], 'repeat_direction': repeat_direction, 'elapsed': elapsed}), flush=True)
                q, status = sample(previous)
                delta = tuple(v - zero for v, zero in zip(q, origin))
                row = {'elapsed': monotonic()-start, 'phase': phase, 'offsets': q, 'delta': delta, 'status': status}
                samples.append(row)
                print(json.dumps(row), flush=True)
                if abs(delta[1-axis]) > 0.02 or abs(delta[axis]) > (0.4 if live_rate_test else 0.15):
                    raise RuntimeError('Unexpected displacement')
                if not live_rate_test and abs(delta[axis]) >= 0.09:
                    break
                previous = q
                sleep(0.05)
        finally:
            timer.cancel()
            stop()
            timer.join()
        stopped_previous = None
        for _ in range(3):
            sleep(0.1)
            q, status = sample(previous)
            if 'N' not in status or (stopped_previous is not None and
                                    any(abs(a-b) > 0.02 for a,b in zip(q, stopped_previous))):
                raise RuntimeError('Stop not confirmed: USE E-STOP')
            stopped_previous = q
            previous = q
            print(json.dumps({'stopped': q, 'delta': tuple(a-b for a,b in zip(q,origin)), 'status': status}), flush=True)
        if samples:
            for p in range(3 if live_rate_test else 1):
                window = [s for s in samples if s['phase'] == p and s['elapsed'] > 1.5*p + 0.3]
                if len(window) < 2:
                    raise RuntimeError('Insufficient samples for speed verification')
                first, last = window[0], window[-1]
                speed = (last['offsets'][axis]-first['offsets'][axis])/(last['elapsed']-first['elapsed'])
                print(json.dumps({'phase': p, 'commanded_speed': rates[p], 'measured_signed_speed': speed}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=('east', 'west', 'north', 'south'))
    parser.add_argument('--live-rate-test', action='store_true', help='Test speed-only updates during a <=5 second jog')
    parser.add_argument('--repeat-direction', action='store_true', help='Reissue direction after rate update, without stopping')
    parser.add_argument('--speed', type=float, choices=(0.025, 0.05), default=0.05, help='Initial speed in deg/s')
    args = parser.parse_args()
    jog(args.direction, args.live_rate_test, args.repeat_direction, args.speed)
