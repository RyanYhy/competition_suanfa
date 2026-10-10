#!/usr/bin/env python3
"""Exercise launch-time races without starting a ROS master or Gazebo."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location(
    'verify_keyboard_drive',
    Path(__file__).resolve().parents[1] / 'scripts/verify_keyboard_drive.py')
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


class StartupTests(unittest.TestCase):
    def run_wait(self, states, clocks, wall_times):
        get = Mock(side_effect=states)
        with patch.object(verify.rospy, 'is_shutdown', return_value=False), \
             patch.object(verify.rospy.Time, 'now', side_effect=[
                 SimpleNamespace(to_sec=lambda value=t: value) for t in clocks]), \
             patch.object(verify.time, 'monotonic', side_effect=wall_times), \
             patch.object(verify.time, 'sleep'):
            verify.wait_for_simulation(get)
        return get

    def test_waits_for_robot_then_clock(self):
        get = self.run_wait(
            [SimpleNamespace(success=False), SimpleNamespace(success=True),
             SimpleNamespace(success=True)], [0, 0, .1], [0, 1, 2, 3])
        self.assertEqual(get.call_count, 3)

    def test_recovers_from_startup_service_error(self):
        get = self.run_wait(
            [verify.rospy.ServiceException('starting'), SimpleNamespace(success=True)],
            [0, .1], [0, 1, 2])
        self.assertEqual(get.call_count, 2)

    def test_paused_simulation_times_out_with_specific_reason(self):
        with self.assertRaisesRegex(RuntimeError, '仿真时钟推进'):
            self.run_wait([SimpleNamespace(success=True)], [0, 0], [0, 1, 61])


if __name__ == '__main__':
    unittest.main()
