#!/usr/bin/env python3
"""Offline checks of traffic gating and scene-specific route clearance."""
import importlib.util
import math
from pathlib import Path
import tempfile
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


patrol = load_script('patrol')
maps = load_script('generate_navigation_maps')


class TrafficTests(unittest.TestCase):
    def test_latched_green_is_not_permission(self):
        gate = patrol.TrafficGate()
        gate.update('green', 20)
        gate.update('green', 20.5)
        self.assertFalse(gate.ready(20.5))

    def test_only_early_green_with_fresh_heartbeat_is_permission(self):
        gate = patrol.TrafficGate()
        gate.update('red', 9.5)
        gate.update('green', 10)
        self.assertTrue(gate.ready(10.5))
        gate.update('green', 12.5)
        self.assertFalse(gate.ready(12.5))
        gate.update('yellow', 25)
        self.assertFalse(gate.ready(25))

    def test_stale_transition_and_clock_reset_do_not_authorize(self):
        gate = patrol.TrafficGate()
        gate.update('red', 9)
        gate.update('green', 24)
        self.assertFalse(gate.ready(24))
        gate.update('red', 30)
        gate.update('green', 30.5)
        self.assertTrue(gate.ready(30.5))
        self.assertFalse(gate.ready(33))
        gate.update('green', 0)
        self.assertFalse(gate.ready(0))


class MapAndRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = np.flipud(np.array(Image.open(ROOT/'maps/field_reference.pgm')))
        cls.keepout = np.flipud(np.array(Image.open(ROOT/'maps/competition_keepout.pgm')))
        cls.obstacles = (cls.reference < 250) | (cls.keepout < 250)
        cls.x, cls.y = np.meshgrid(-2.3+(np.arange(230)+.5)*.02,
                                  -2.3+(np.arange(230)+.5)*.02)
        cls.route = yaml.safe_load((ROOT/'config/navigation/patrol_route.yaml').read_text())['waypoints']

    def test_all_stops_fit_padded_footprint(self):
        for point in self.route:
            with self.subTest(point=point['name']):
                c, s = math.cos(point['yaw']), math.sin(point['yaw'])
                dx, dy = self.x-point['x'], self.y-point['y']
                # Configured half dimensions .20/.18, plus half a grid cell.
                inside = (abs(c*dx+s*dy) <= .21) & (abs(-s*dx+c*dy) <= .19)
                self.assertFalse(np.any(inside & self.obstacles))

    def test_five_mm_perimeter_walls_survive_twenty_mm_grid(self):
        # No cell centre lies in these thin walls. Lidar sees them nonetheless,
        # so each perimeter side must remain continuously occupied in the map.
        for coordinate, along in ((self.x, self.y), (self.y, self.x)):
            for side in (-1, 1):
                wall = (abs(coordinate-side*2.11) < .001) & (abs(along) < 2.08)
                self.assertTrue(np.any(wall))
                self.assertTrue(np.all(self.reference[wall] == 0))
                # The north signal has a real post close to the perimeter;
                # inspect perimeter stretches away from those interior posts.
                road_edge = (abs(coordinate-side*2.09) < .001) & (abs(along) < 2.08) & (abs(along) > .8)
                self.assertTrue(np.all(self.reference[road_edge] == 254))

    def test_route_segments_and_turns_have_clearance(self):
        previous = {'x': 1.65, 'y': 1.75}
        # Circumscribed radius covers every heading, plus half a cell diagonal.
        radius = math.hypot(.20, .18) + math.sqrt(2)*.01
        ox, oy = self.x[self.obstacles], self.y[self.obstacles]
        for point in self.route:
            with self.subTest(segment=point['name']):
                steps = max(2, math.ceil(math.hypot(point['x']-previous['x'], point['y']-previous['y'])/.01)+1)
                for t in np.linspace(0, 1, steps):
                    x = previous['x']*(1-t)+point['x']*t
                    y = previous['y']*(1-t)+point['y']*t
                    if point['name'] == 'start_centerline':
                        # Spawn is offset from the road centre. First translate
                        # sideways with the initial west-facing heading retained.
                        self.assertFalse(np.any((abs(ox-x)<=.21)&(abs(oy-y)<=.19)))
                        continue
                    self.assertGreater(float(np.min((ox-x)**2+(oy-y)**2)), radius**2)
            previous = point

    def test_waits_remain_before_stop_lines_with_goal_tolerance(self):
        for point in self.route:
            if point.get('signal') == 'north_signal':
                self.assertGreater(point['x']-.20-.05, .30+.0125)
            elif point.get('signal') == 'south_signal':
                self.assertGreater(point['y']-.20-.05, -.55+.0125)

    def test_alignment_preserves_grid_and_outside_is_forbidden(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            output = folder/'aligned.yaml'
            maps.align_keepout(ROOT/'maps/field_reference.yaml', output)
            aligned = np.flipud(np.array(Image.open(output.with_suffix('.pgm'))))
            np.testing.assert_array_equal(aligned, self.keepout)
            target = dict(image='large.pgm', origin=[-3., -3., 0.], resolution=.02)
            Image.new('L', (300, 300), 254).save(str(folder/'large.pgm'))
            (folder/'large.yaml').write_text(yaml.safe_dump(target))
            maps.align_keepout(folder/'large.yaml', output)
            result = yaml.safe_load(output.read_text())
            self.assertEqual(result['origin'], target['origin'])
            image = np.array(Image.open(output.with_suffix('.pgm')))
            self.assertEqual(image.shape, (300, 300))
            self.assertEqual(image[0, 0], 0)
            self.assertEqual(image[0, -1], 0)


if __name__ == '__main__':
    unittest.main()
