"""Regression checks for failures observed in actual camera experiments."""
import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from smart_community_sim.observation_tracker import PersonTracker
from smart_community_sim.yolo_ocr import lamp_color,normalize_plate,valid_plate


class ObservationRegression(unittest.TestCase):
    def test_same_person_seen_from_two_viewpoints_is_counted_once(self):
        tracker=PersonTracker()
        detection={'confidence':.8,'role':'resident'}
        for location in ([0.,0.],[.01,-.01],[.015,.01],[.025,0.]):
            tracker.update('A',[(detection,location)])
        self.assertEqual(tracker.counts()['A']['total'],1)
        self.assertEqual(tracker.counts()['B']['total'],0)

    def test_nearby_figures_stay_distinct_and_single_frame_is_unconfirmed(self):
        tracker=PersonTracker()
        observations=[({'confidence':.7,'role':'resident'},[0.,0.]),
                      ({'confidence':.6,'role':'visitor'},[.10,0.])]
        tracker.update('B',observations)
        self.assertEqual(tracker.counts()['B']['total'],0)
        for _ in range(2):tracker.update('B',observations)
        self.assertEqual(tracker.counts()['B']['total'],2)
        self.assertEqual(tracker.counts()['B']['visitors'],1)

    def test_off_yellow_lens_does_not_override_luminous_red(self):
        image=np.zeros((60,180,3),np.uint8)
        image[:,:100]=(0,170,170)  # Colored but unlit yellow lens.
        image[:,120:150]=(0,0,255)
        self.assertEqual(lamp_color(image,[0,0,180,60])[0],'red')
        image[:,120:150]=0
        self.assertEqual(lamp_color(image,[0,0,180,60])[0],'unknown')

    def test_two_luminous_colors_are_unknown(self):
        image=np.zeros((60,180,3),np.uint8)
        image[:,:60]=(0,0,255);image[:,120:]=(0,255,0)
        self.assertEqual(lamp_color(image,[0,0,180,60])[0],'unknown')

    def test_plate_format_does_not_invent_missing_characters(self):
        self.assertEqual(normalize_plate('粤B · 12345'),'粤B12345')
        self.assertTrue(valid_plate('粤B12345'))
        self.assertFalse(valid_plate('粤B1234'))
        self.assertFalse(valid_plate('UNKNOWN'))


if __name__=='__main__':unittest.main()
