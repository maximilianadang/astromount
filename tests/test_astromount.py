"""Exercise real serial framing against a Linux pseudo-terminal, never hardware."""

from contextlib import contextmanager
import os
import pty
import random
import select
from threading import Thread
import unittest

from astromount import Mount, PointingFrame, ProtocolError


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
    def test_horizontal_target_reuses_goto(self):
        with simulated([
            (b":GU#", b"NG#"), (b":Sr04:00:00#", b"1"),
            (b":Sd+00*00:00#", b"1"), (b":MS#", b"0"),
        ]) as mount:
            mount.goto_horizontal(PointingFrame(0, 0), azimuth_degrees=90, altitude_degrees=30)

    def test_relative_yaw_both_directions(self):
        for offset, declination in [(5, b"-05"), (-5, b"+05")]:
            with self.subTest(offset=offset), simulated([
                (b":GR#", b"06:00:00#"), (b":GD#", b"+00*00:00#"),
                (b":GU#", b"NG#"), (b":Sr06:00:00#", b"1"),
                (b":Sd" + declination + b"*00:00#", b"1"), (b":MS#", b"0"),
            ]) as mount:
                mount.yaw(offset, frame=PointingFrame(0, 0))

    def test_horizontal_target_rejects_mode_or_active_slew(self):
        for status in [b"NZ#", b"G#", b"NGZ#", b"#"]:
            with self.subTest(status=status), simulated([(b":GU#", status)]) as mount:
                with self.assertRaises(ProtocolError):
                    mount.goto_horizontal(PointingFrame(0, 0), azimuth_degrees=90, altitude_degrees=30)

    def test_invalid_horizontal_target_and_zero_yaw_send_nothing(self):
        with simulated([]) as mount:
            frame = PointingFrame(0, 0)
            for az, alt in [(float("nan"), 0), (0, 91), (0, float("inf"))]:
                with self.assertRaises(ValueError):
                    mount.goto_horizontal(frame, azimuth_degrees=az, altitude_degrees=alt)
            with self.assertRaises(ValueError):
                mount.yaw(float("nan"), frame=frame)
            mount.yaw(0, frame=frame)
            mount.yaw(360, frame=frame)

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


class PointingFrameTests(unittest.TestCase):
    def test_known_directions_and_hemispheres(self):
        for latitude, ra, dec, azimuth, altitude in [
            (0, 6, 0, 90, 0), (0, 18, 0, 270, 0),
            (45, 0, 0, 180, 45), (-45, 0, 0, 0, 45),
            (45, 6, 90, 0, 45),
        ]:
            with self.subTest(latitude=latitude, ra=ra, dec=dec):
                az, alt = PointingFrame(latitude, 0).horizontal(ra_hours=ra, dec_degrees=dec)
                self.assertAlmostEqual((az - azimuth + 180) % 360 - 180, 0)
                self.assertAlmostEqual(alt, altitude)

    def test_round_trip_across_sky(self):
        rng = random.Random(5)
        for _ in range(200):
            frame = PointingFrame(rng.uniform(-89, 89), rng.uniform(0, 24))
            ra, dec = rng.uniform(0, 24), rng.uniform(-89, 89)
            az, alt = frame.horizontal(ra_hours=ra, dec_degrees=dec)
            result_ra, result_dec = frame.equatorial(azimuth_degrees=az, altitude_degrees=alt)
            self.assertAlmostEqual((result_ra - ra + 12) % 24 - 12, 0)
            self.assertAlmostEqual(result_dec, dec)

    def test_azimuth_wrap_and_sidereal_time(self):
        frame = PointingFrame(40, 3)
        first = frame.equatorial(azimuth_degrees=-5, altitude_degrees=20)
        self.assertEqual(first, frame.equatorial(azimuth_degrees=355, altitude_degrees=20))
        later = PointingFrame(40, 4).equatorial(azimuth_degrees=355, altitude_degrees=20)
        self.assertAlmostEqual((later[0] - first[0]) % 24, 1)
        self.assertEqual(later[1], first[1])

    def test_invalid_frames_coordinates_and_singularities(self):
        for latitude, sidereal in [(91, 0), (float("nan"), 0), (0, 24), (0, float("inf"))]:
            with self.assertRaises(ValueError):
                PointingFrame(latitude, sidereal)
        frame = PointingFrame(45, 0)
        for ra, dec in [(24, 0), (0, 91), (0, float("nan")), (0, 45)]:
            with self.assertRaises(ValueError):
                frame.horizontal(ra_hours=ra, dec_degrees=dec)
        with self.assertRaises(ValueError):
            frame.equatorial(azimuth_degrees=0, altitude_degrees=45)


if __name__ == "__main__":
    unittest.main()
