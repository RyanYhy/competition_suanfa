import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from smart_community_sim.drive_metrics import compare_motion, stationary_check


class DriveMetricsTests(unittest.TestCase):
    def test_encoder_turn_without_body_turn_fails(self):
        report=compare_motion((1,2,0),(1,2,0),(0,0,0),(0,0,1))
        self.assertFalse(report['odometry_consistent'])
        self.assertAlmostEqual(report['yaw_error_rad'],1)

    def test_wrap_and_different_absolute_frame_origins(self):
        report=compare_motion((1,2,math.pi-.1),(1,2,-math.pi+.1),
                              (0,0,0),(0,0,.2))
        self.assertTrue(report['odometry_consistent'])
        self.assertAlmostEqual(report['yaw_error_rad'],0)

    def test_arc_motion_is_not_an_in_place_turn(self):
        report=compare_motion((0,0,0),(.2,0,.5),(0,0,0),(0,0,.5))
        self.assertFalse(report['odometry_consistent'])
        self.assertAlmostEqual(report['displacement_error_m'],.2)

    def test_wheel_spin_translation_is_detected(self):
        report=compare_motion((0,0,0),(0,0,0),(0,0,0),(.2,0,0))
        self.assertFalse(report['odometry_consistent'])

    def test_invalid_measurement_cannot_pass(self):
        for bad in (math.nan,math.inf):
            self.assertFalse(compare_motion((0,0,0),(0,0,bad),(0,0,0),(0,0,0))['odometry_consistent'])

    def test_stationary_encoder_drift_fails_even_when_body_is_stopped(self):
        report=stationary_check((0,0,0),(.003,0,.001),(0,0,0),(-.025,0,.03))
        self.assertFalse(report['pass'])

    def test_small_motion_error_does_not_imply_stationary(self):
        report=stationary_check((0,0,0),(.03,0,0),(0,0,0),(.03,0,0))
        self.assertTrue(report['odometry_consistent'])
        self.assertFalse(report['pass'])

    def test_stationary_bounded_noise_passes(self):
        self.assertTrue(stationary_check((0,0,0),(.002,0,.001),(0,0,0),(.003,0,.002))['pass'])
