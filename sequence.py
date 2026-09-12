"""Run CSV waypoints sequentially through the shared pointing CLI."""

from point import main

if __name__ == '__main__':
    raise SystemExit(main(sequence=True))
