"""ZWO AM5/AM5N control. Coordinates are firmware estimates, not encoder readings."""

from dataclasses import dataclass
import math
import re
from threading import RLock
from time import monotonic

import serial


class ProtocolError(RuntimeError):
    """Malformed, incomplete, or rejected mount response."""


@dataclass(frozen=True)
class Position:
    ra_hours: float
    dec_degrees: float
    started_at: float
    finished_at: float


def _angle(text: str, *, ra: bool) -> float:
    pattern = r"(\d{2}):(\d{2})(?::(\d{2}(?:\.\d+)?))?" if ra else r"([+-]\d{2})\*(\d{2})(?::(\d{2}(?:\.\d+)?))?"
    match = re.fullmatch(pattern, text)
    if not match:
        raise ProtocolError(f"Invalid coordinate: {text!r}")
    major, minutes, seconds = match.groups()
    minutes, seconds = int(minutes), float(seconds or 0)
    value = abs(int(major)) + minutes / 60 + seconds / 3600
    if minutes >= 60 or seconds >= 60 or (value >= 24 if ra else value > 90):
        raise ProtocolError(f"Coordinate out of range: {text!r}")
    return -value if major.startswith("-") else value


def _sexagesimal(value: float, *, ra: bool) -> str:
    if not math.isfinite(value) or not (0 <= value < 24 if ra else -90 <= value <= 90):
        raise ValueError("RA must be in [0, 24) hours; DEC in [-90, 90] degrees")
    total = round(abs(value) * 3600)
    if ra:
        total %= 24 * 3600
    major, rest = divmod(total, 3600)
    minutes, seconds = divmod(rest, 60)
    sign = "" if ra else ("-" if math.copysign(1, value) < 0 else "+")
    separator = ":" if ra else "*"
    return f"{sign}{major:02d}{separator}{minutes:02d}:{seconds:02d}"


def decode_coordinates(queries):
    """Decode a saved/live clock bracket into hour angle and declination (degrees)."""
    before = _angle(queries['sidereal_before']['response'], ra=True)
    after = _angle(queries['sidereal_after']['response'], ra=True)
    coordinates = queries['equatorial']['response'].split('&')
    if len(coordinates) != 2:
        raise ProtocolError('Invalid combined coordinate response')
    ra, dec = _angle(coordinates[0], ra=True), _angle(coordinates[1], ra=False)
    elapsed = (after - before) % 24
    if elapsed > 2 / 3600:
        raise ProtocolError('Firmware clock changed during position read')
    return (15 * (before + elapsed / 2 - ra) + 180) % 360 - 180, dec


class Mount:
    """Own one connection. Opening and closing never send mount commands.

    Calls are serialized, including multi-command operations. Motion calls return
    after dispatch/acknowledgment, not after physical completion. No retries.
    """

    def __init__(self, port: str, *, baudrate: int = 9600, timeout: float = 2):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self._lock = RLock()
        self._timeout = timeout
        self._serial = serial.Serial(
            port, baudrate, timeout=timeout, write_timeout=timeout, exclusive=True
        )

    def __enter__(self): return self

    def __exit__(self, *exc): self.close()

    def close(self) -> None:
        """Release the port; does not stop movement or change tracking."""
        with self._lock:
            self._serial.close()

    def _command(self, command: str, reply: str = "terminated") -> str:
        """Reply is hash-terminated, one byte, a slew result, or absent."""
        with self._lock:
            try:
                self._serial.reset_input_buffer()
                data = command.encode("ascii")
                if self._serial.write(data) != len(data):
                    raise ProtocolError(f"Incomplete write: {command}")
                if reply == "none":
                    return ""
                deadline = monotonic() + self._timeout

                def read(terminated: bool) -> str:
                    result = bytearray()
                    while monotonic() < deadline and len(result) < 128:
                        self._serial.timeout = max(0, deadline - monotonic())
                        byte = self._serial.read(1)
                        if not byte:
                            break
                        if terminated and byte == b"#":
                            return result.decode("ascii")
                        result.extend(byte)
                        if not terminated:
                            return result.decode("ascii")
                    raise ProtocolError(f"Incomplete/oversized reply to {command}: {bytes(result)!r}")

                response = read(reply == "terminated")
                if reply == "byte" and response == "e":
                    raise ProtocolError(f"Command rejected: {command} returned e{read(True)}")
                if reply == "slew" and response != "0":
                    # Failed go-to replies include a hash-terminated explanation.
                    raise ProtocolError(f"Go-to rejected ({response}): {read(True)}")
                return response
            except (serial.SerialException, OSError, UnicodeError, ProtocolError):
                # A late reply cannot safely be associated with the next command.
                self._serial.close()
                raise

    def _accept(self, command: str) -> None:
        response = self._command(command, "byte")
        if response != "1":
            raise ProtocolError(f"Command rejected: {command} returned {response!r}")

    def identity(self) -> dict[str, str | None]:
        """Model and firmware; an empty firmware response is represented by None."""
        with self._lock:
            model = self._command(":GVP#")
            if not model:
                raise ProtocolError("Empty model response")
            return {"model": model, "firmware": self._command(":GV#") or None}

    def position(self) -> Position:
        """Sequential RA/DEC samples, bracketed by monotonic timestamps."""
        with self._lock:
            started = monotonic()
            ra = _angle(self._command(":GR#"), ra=True)
            dec = _angle(self._command(":GD#"), ra=False)
            return Position(ra, dec, started, monotonic())

    def status(self) -> str:
        """Raw firmware status flags: N = not slewing, H = home, G/Z = EQ/alt-az."""
        return self._command(":GU#")

    def coordinate_queries(self):
        """Read one atomic clock bracket, retaining raw responses for baseline capture."""
        with self._lock:
            return {name: {'command': command, 'response': self._command(command)} for name, command in
                    (('sidereal_before', ':GS#'), ('equatorial', ':GMEQ#'), ('sidereal_after', ':GS#'))}

    def joint_sample(self) -> tuple[float, float, str]:
        """Read hour angle (degrees), DEC and status; caller reconstructs joint offsets."""
        with self._lock:
            return *decode_coordinates(self.coordinate_queries()), self.status()

    def tracking(self) -> bool:
        response = self._command(":GAT#")
        if response not in ("0", "1"):
            raise ProtocolError(f"Invalid tracking response: {response!r}")
        return response == "1"

    def set_tracking(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be bool")
        self._accept(":Te#" if enabled else ":Td#")

    def goto(self, *, ra_hours: float, dec_degrees: float) -> None:
        """Start an equatorial go-to. Requires configured time/site/alignment.

        Firmware may enable tracking on arrival. This is not a joint-angle API.
        """
        ra = _sexagesimal(ra_hours, ra=True)
        dec = _sexagesimal(dec_degrees, ra=False)
        with self._lock:
            self._accept(f":Sr{ra}#")
            self._accept(f":Sd{dec}#")
            self._command(":MS#", "slew")

    @staticmethod
    def _direction(direction: str) -> str:
        if direction not in ("north", "south", "east", "west"):
            raise ValueError("direction must be north, south, east, or west")
        return direction[0]

    def move(self, direction: str, *, rate: int) -> None:
        """Start continuous motion at firmware speed index 0..9; call stop().

        Indices: 0.25, 0.5, 1, 2, 4, 8, 20, 60, 720, 1440 times sidereal.
        The rate selector is shared; direction commands start individual axes.
        """
        direction = self._direction(direction)
        if type(rate) is not int or not 0 <= rate <= 9:
            raise ValueError("rate must be an integer from 0 to 9")
        with self._lock:
            self._command(f":R{rate}#", "none")
            self._command(f":M{direction}#", "none")

    def guide(self, direction: str, *, duration_ms: int) -> None:
        """Dispatch a timed correction at the mount's configured guide rate.

        Returns immediately; the caller schedules pulses and avoids overlap.
        """
        direction = self._direction(direction)
        if type(duration_ms) is not int or not 1 <= duration_ms <= 3000:
            raise ValueError("duration_ms must be an integer from 1 to 3000")
        self._command(f":Mg{direction}{duration_ms:04d}#", "none")

    def jog(self, direction: str, *, speed_degrees_s: float) -> None:
        """Start directional motion with a positive speed, limited to 6 deg/s.

        Stop the affected axis before changing its rate/direction. On tested AM5N
        1.6.3 firmware the other axis retains its active rate. No acknowledgment
        or automatic stopping is provided by firmware.
        """
        direction = self._direction(direction)
        if not math.isfinite(speed_degrees_s) or not 0 < speed_degrees_s <= 6:
            raise ValueError("speed_degrees_s must be finite and in (0, 6]")
        rate = math.floor(speed_degrees_s / (360 / 86164.0905) * 100) / 100
        if rate == 0:
            raise ValueError("Speed is below the firmware's resolution")
        with self._lock:
            self._command(f":Rv{rate:.2f}#", "none")
            self._command(f":M{direction}#", "none")

    def stop(self, direction: str | None = None) -> None:
        """Stop all slewing, or only the given direction. Does not disable tracking."""
        suffix = '' if direction is None else self._direction(direction)
        self._command(f":Q{suffix}#", "none")

    def home(self) -> None:
        """Start physical movement to the home sensor reference."""
        self._command(":hC#", "none")
