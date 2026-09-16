"""Continuous CSV az/el sweep; sequence.py inputs, no intermediate settling."""
from point import main

if __name__ == '__main__':
    raise SystemExit(main(sequence=True, continuous=True))
