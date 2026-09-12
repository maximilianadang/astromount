"""Exercise real serial framing against a Linux pseudo-terminal, never hardware."""

from contextlib import contextmanager
import os
import pty
import select
from threading import Thread
import unittest

from astromount import Mount, ProtocolError


@contextmanager
def simulated(exchanges):
    master, slave = pty.openpty()
    failures = []

    def serve():
        try:
            for expected, response in exchanges:
                received = bytearray()
                while not received.endswith(b"#"):
                    if not select.select([master], [], [], 2)[0]:
                        raise AssertionError(f"Missing command: {expected}")
                    received.extend(os.read(master, 1))
                assert received == expected, (received, expected)
                if response:
                    os.write(master, response)
        except Exception as exc:
            failures.append(exc)

    with Mount(os.ttyname(slave), timeout=0.1) as mount:
        thread = Thread(target=serve)
        thread.start()
        try:
            yield mount
        finally:
            thread.join(3)
            os.close(master)
            os.close(slave)
            if thread.is_alive():
                raise AssertionError("Simulator did not finish")
            if failures:
                raise failures[0]


class MountTests(unittest.TestCase):
    def test_read_only_queries_and_empty_firmware(self):
        with simulated([
            (b":GVP#", b"AM5N#"), (b":GV#", b"#"),
            (b":GR#", b"06:18:35#"), (b":GD#", b"-00*30:00#"),
            (b":GU#", b"NHPG#"), (b":GAT#", b"0#"),
        ]) as mount:
            self.assertEqual(mount.identity(), {"model": "AM5N", "firmware": None})
            position = mount.position()
            self.assertAlmostEqual(position.ra_hours, 6 + 18 / 60 + 35 / 3600)
            self.assertEqual(position.dec_degrees, -0.5)
            self.assertLessEqual(position.started_at, position.finished_at)
            self.assertEqual(mount.status(), "NHPG")
            self.assertFalse(mount.tracking())

    def test_motion_wire_format_and_reply_boundaries(self):
        with simulated([
            (b":Sr00:00:00#", b"1"), (b":Sd-00*30:00#", b"1"),
            (b":MS#", b"0"), (b":R0#", b""), (b":Me#", b""),
            (b":Mgn0100#", b""), (b":Q#", b""), (b":Td#", b"1"),
            (b":hC#", b""), (b":GVP#", b"AM5N#"), (b":GV#", b"#"),
        ]) as mount:
            mount.goto(ra_hours=23.999999, dec_degrees=-0.5)
            mount.move("east", rate=0)
            mount.guide("north", duration_ms=100)
            mount.stop()
            mount.set_tracking(False)
            mount.home()
            self.assertEqual(mount.identity()["model"], "AM5N")

    def test_invalid_inputs_send_nothing(self):
        with simulated([]) as mount:
            for ra, dec in [(24, 0), (0, 91), (float("nan"), 0), (0, float("inf"))]:
                with self.assertRaises(ValueError):
                    mount.goto(ra_hours=ra, dec_degrees=dec)
            for duration in [0, -1, 10000, True, 1.5]:
                with self.assertRaises(ValueError):
                    mount.guide("north", duration_ms=duration)
            with self.assertRaises(ValueError):
                mount.move("east", rate=10)
            with self.assertRaises(ValueError):
                mount.move("up", rate=0)
            with self.assertRaises(ValueError):
                mount.set_tracking("false")

    def test_failed_target_never_starts_slew(self):
        with simulated([(b":Sr01:00:00#", b"0")]) as mount:
            with self.assertRaisesRegex(ProtocolError, "rejected"):
                mount.goto(ra_hours=1, dec_degrees=2)

    def test_slew_error_explanation(self):
        with simulated([
            (b":Sr01:00:00#", b"1"), (b":Sd+02*00:00#", b"1"),
            (b":MS#", b"1Below horizon#"),
        ]) as mount:
            with self.assertRaisesRegex(ProtocolError, "Below horizon"):
                mount.goto(ra_hours=1, dec_degrees=2)
            self.assertFalse(mount._serial.is_open)

    def test_bad_frames_close_connection_without_retry(self):
        for response in [b"", b"partial", b"x" * 128, b"\xff#"]:
            with self.subTest(response=response), simulated([(b":GU#", response)]) as mount:
                with self.assertRaises((ProtocolError, UnicodeError)):
                    mount.status()
                self.assertFalse(mount._serial.is_open)

    def test_invalid_coordinate(self):
        for response in [b"garbage#", b"24:00:00#", b"12:60:00#"]:
            with self.subTest(response=response), simulated([(b":GR#", response)]) as mount:
                with self.assertRaises(ProtocolError):
                    mount.position()

    def test_concurrent_gotos_remain_atomic(self):
        with simulated([
            (b":Sr01:00:00#", b"1"), (b":Sd+02*00:00#", b"1"), (b":MS#", b"0"),
            (b":Sr01:00:00#", b"1"), (b":Sd+02*00:00#", b"1"), (b":MS#", b"0"),
        ]) as mount:
            failures = []

            def point():
                try:
                    mount.goto(ra_hours=1, dec_degrees=2)
                except Exception as exc:
                    failures.append(exc)

            threads = [Thread(target=point) for _ in range(2)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
