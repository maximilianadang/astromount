import math
import random
import unittest

from astromount_kinematics import Pointing


class KinematicsTests(unittest.TestCase):
    def test_baseline_and_individual_axes(self):
        frame = Pointing(1, 1)
        self.assertEqual(frame.direction(0, 0), (1, 0, 0))
        for q in (-22.5, -5, 0, 5, 22.5):
            for actual, expected in zip(frame.forward(q, 0), (0, q)):
                self.assertAlmostEqual(actual, expected)
            for actual, expected in zip(frame.forward(0, q), (q, 0)):
                self.assertAlmostEqual(actual, expected)

    def test_composition_rotates_upper_axis_with_lower(self):
        frame = Pointing(1, 1)
        # Independent Rodrigues rotations: first Z yaw, then Y pitch.
        def rotate(v, axis, degrees):
            c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
            dot = sum(a*b for a, b in zip(axis, v))
            cross = (axis[1]*v[2]-axis[2]*v[1], axis[2]*v[0]-axis[0]*v[2], axis[0]*v[1]-axis[1]*v[0])
            return tuple(c*x+s*y+(1-c)*dot*a for x,y,a in zip(v,cross,axis))
        expected = rotate(rotate((1,0,0), (0,0,1), 20), (0,1,0), 15)
        for actual, value in zip(frame.direction(15,20), expected):
            self.assertAlmostEqual(actual, value)
        az, el = frame.forward(15,20)
        self.assertGreater(az, 20)
        self.assertLess(el, 15)

    def test_roundtrips_all_signs_and_boundaries(self):
        rng = random.Random(7)
        points = [(a,b) for a in (-75,0,75) for b in (-75,0,75)]
        points += [(rng.uniform(-22.5,22.5), rng.uniform(-22.5,22.5)) for _ in range(500)]
        for ps in (-1,1):
            for ys in (-1,1):
                frame = Pointing(ps,ys)
                for q in points:
                    for actual, expected in zip(frame.inverse(*frame.forward(*q)), q):
                        self.assertAlmostEqual(actual, expected, places=9)

    def test_prior_captured_joint_positions_roundtrip(self):
        frame = Pointing(1,1)
        for q in ((-3.4520833333,.0011111111),(-.0083333333,-12.4244444444),(-.0083333333,-.0194444444)):
            for actual, expected in zip(frame.inverse(*frame.forward(*q)),q):
                self.assertAlmostEqual(actual, expected, places=9)

    def test_invalid_and_unreachable_inputs(self):
        for args in ((0,1), (1,True)):
            with self.assertRaises(ValueError):
                Pointing(*args)
        frame = Pointing(1,1)
        for q in ((0,float('nan')), (float('inf'),0)):
            with self.assertRaises(ValueError):
                frame.forward(*q)
        for target in ((180,0), (0,90), (0,91), (float('inf'),0)):
            with self.assertRaises(ValueError):
                frame.inverse(*target)
        for a,b in zip(frame.inverse(355,0), frame.inverse(-5,0)):
            self.assertAlmostEqual(a,b)
