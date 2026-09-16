"""Flushed, per-run JSONL telemetry; no hardware I/O."""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import sys
from time import monotonic, time

from astromount_config import ROOT


@contextmanager
def sweep_log(metadata, path=None):
    if path is None:
        directory = ROOT / 'output'
        directory.mkdir(exist_ok=True)
        path = directory / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '-sweep.jsonl')
    with path.open('x') as file:
        def emit(event, **values):
            print(json.dumps(dict(event=event, monotonic_s=monotonic(), unix_s=time(), **values),
                             allow_nan=False), file=file, flush=True)
        emit('start', **metadata)
        print(f'Sweep log: {path}', file=sys.stderr, flush=True)
        try:
            yield emit
        except BaseException as exc:
            emit('end', status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed', error=repr(exc))
            raise
        else:
            emit('end', status='completed')
