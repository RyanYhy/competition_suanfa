#!/usr/bin/env python3
"""Regression checks based on the newly supplied organizer documents."""
import hashlib
import json
import math
import sys
import unittest
from pathlib import Path
import xml.etree.ElementTree as E
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/"src"))
from smart_community_sim.traffic_guard import blocked_signal
from smart_community_sim.vision_core import TemplateRecognizer, signal_state
from patrol import TrafficGate


class OfficialGeometryTests(unittest.TestCase):
    def test_official_files_are_not_replaced_by_mock_art(self):
        records = json.loads((ROOT/'config/recognition_assets.json').read_text())
        self.assertEqual(len(records), 21)
        for record in records:
            with self.subTest(asset=record['id']):
                bundled = (ROOT/record['texture']).read_bytes()
                self.assertEqual(record['sha256'], hashlib.sha256(bundled).hexdigest())
                original=ROOT.parent/'doc/复赛资料'/record['source']
                if original.is_file():
                    self.assertEqual(hashlib.sha256(original.read_bytes()).digest(), hashlib.sha256(bundled).digest())

    def test_physical_dimensions_and_removed_final_round_objects(self):
        world = E.parse(str(ROOT/'worlds/smart_community.world')).getroot().find('world')
        names = [model.get('name') for model in world.findall('model')]
        self.assertEqual(sum(n.startswith('person_A') for n in names), 9)
        self.assertEqual(sum(n.startswith('person_B') for n in names), 9)
        self.assertFalse(any(n.startswith(('building_', 'ebike_', 'bin_')) or n == 'station' for n in names))
        for name in ('north_signal', 'south_signal'):
            link = world.find("model[@name='"+name+"']/link")
            self.assertEqual(link.findtext("collision[@name='housing']/geometry/box/size"), '0.05 0.59 0.14')
            self.assertEqual(float(link.findtext("collision[@name='housing']/pose").split()[2]), .41)
            self.assertEqual(link.findtext("collision[@name='left_post']/geometry/box/size"), '0.025 0.025 0.48')
        for i in range(1, 4):
            link = world.find("model[@name='car_"+str(i)+"']/link")
            self.assertEqual(link.findtext("collision[@name='backing']/geometry/box/size"), '0.0005 0.345 0.25')
            link = world.find("model[@name='license_plate_"+str(i)+"']/link")
            self.assertEqual(link.findtext("collision[@name='standee']/geometry/box/size"), '0.0001 0.095 0.03')
            self.assertEqual(link.findtext("visual[@name='number_plate']/geometry/mesh/scale"), '1 0.095 0.03')

    def test_single_official_lap(self):
        points = yaml.safe_load((ROOT/'config/navigation/patrol_route.yaml').read_text())['waypoints']
        self.assertEqual([p['signal'] for p in points if 'signal' in p], ['north_signal', 'south_signal'])
        # The west road must turn into the middle road, without the old southwest detour.
        self.assertFalse(any(p['x'] < -.4 and p['y'] < .59 for p in points))
        north = next(i for i,p in enumerate(points) if p.get('signal') == 'north_signal')
        south = next(i for i,p in enumerate(points) if p.get('signal') == 'south_signal')
        cars = [i for i,p in enumerate(points) if p['name'].startswith('observe_plate_')]
        self.assertLess(north, south); self.assertLess(south, min(cars))
        for a,b in zip(points, points[1:]):
            self.assertTrue(abs(a['x']-b['x']) < 1e-7 or abs(a['y']-b['y']) < 1e-7)


class BrakingTests(unittest.TestCase):
    def check(self,x,y,yaw,vx,vy,mx=0,my=0,green=False):
        return blocked_signal(x,y,yaw,vx,vy,mx,my,lambda _:green)

    def test_red_or_missing_light_stops_front_before_line(self):
        self.assertEqual(self.check(.58,1.8,math.pi,.18,0), 'north_signal')
        self.assertEqual(self.check(-.1,-.27,-math.pi/2,.18,0), 'south_signal')
        self.assertIsNone(self.check(.68,1.8,math.pi,.18,0))
        self.assertIsNone(self.check(.58,1.8,math.pi,.18,0,green=True))

    def test_measured_motion_and_holonomic_direction_are_considered(self):
        self.assertEqual(self.check(.56,1.8,0,0,0,mx=-.18), 'north_signal')
        self.assertEqual(self.check(.56,1.8,math.pi/2,0,.18), 'north_signal')
        self.assertIsNone(self.check(.56,.6,0,-.18,0))
        self.assertIsNone(self.check(.1,1.8,math.pi,.18,0))

    def test_unknown_revokes_green_without_losing_recent_red_phase(self):
        gate=TrafficGate();gate.update('red',9);gate.update('unknown',9.3)
        self.assertFalse(gate.green(9.3))
        gate.update('green',9.6);self.assertTrue(gate.ready(9.6))
        gate.update('unknown',10);self.assertFalse(gate.green(10))
        first=TrafficGate();first.update('unknown',1);first.update('green',1.3)
        self.assertFalse(first.ready(1.3))


class CameraRecognitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = TemplateRecognizer(ROOT)

    def test_resident_visitor_and_plate_from_rescaled_pixels(self):
        for name in ('resident_1','resident_5','visitor_F1','visitor_F2','plate_1','plate_2','plate_3'):
            with self.subTest(identity=name):
                source=cv2.imread(str(ROOT/'materials/textures'/(name+'.png')),cv2.IMREAD_UNCHANGED)
                if name.startswith('plate'):
                    source=cv2.resize(source,(190,60))
                else:
                    source=cv2.resize(source,None,fx=.7,fy=.7)
                alpha=source[:,:,3:4].astype(float)/255
                rgb=(source[:,:,:3]*alpha+40*(1-alpha)).astype(np.uint8)
                frame=np.full((480,640,3),40,np.uint8)
                h,w=rgb.shape[:2];frame[90:90+h,230:230+w]=rgb
                result=self.detector.detect(frame)
                self.assertIn(name, [d['id'] for d in result])
                self.assertEqual(len(result),1)

    def test_blank_frame_and_unknown_signal_do_not_invent_results(self):
        self.assertEqual(self.detector.detect(np.zeros((480,640,3),np.uint8)),[])
        self.assertEqual(signal_state([]),'unknown')
        self.assertEqual(signal_state([dict(category='signal',state='off',color='green')]),'unknown')
        self.assertEqual(signal_state([dict(category='signal',state='on',color=c) for c in ('red','green')]),'unknown')

    def test_six_lamp_templates_distinguish_lit_from_dark(self):
        for color in ('red','yellow','green'):
            for state in ('on','off'):
                with self.subTest(color=color,state=state):
                    name='signal_'+color+'_'+state
                    source=cv2.imread(str(ROOT/'materials/textures'/(name+'.png')))
                    source=cv2.resize(source,(100,100))
                    frame=np.full((480,640,3),50,np.uint8);frame[100:200,240:340]=source
                    detected=self.detector.detect(frame,signals=True)
                    self.assertEqual(signal_state(detected),color if state=='on' else 'unknown')


if __name__ == '__main__':unittest.main()
