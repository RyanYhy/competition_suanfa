#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import tempfile
import json
import importlib.util
import numpy as np
from PIL import Image
import unittest
import xml.etree.ElementTree as E
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from smart_community_sim.timing import fresh_sample, limit_velocity
from smart_community_sim.benchmark_metrics import summarize, goal_accuracy


class TimingTests(unittest.TestCase):
    def test_slow_simulation_is_not_a_lost_command(self):
        self.assertTrue(fresh_sample(1.10,1.,11.,10.))
    def test_real_loss_pause_and_clock_reset_are_rejected(self):
        self.assertFalse(fresh_sample(1.5,1.,10.1,10.))
        self.assertFalse(fresh_sample(1.,1.,13.,10.))
        self.assertFalse(fresh_sample(.5,1.,10.1,10.))
        self.assertFalse(fresh_sample(1.,None,10.,None))
        self.assertFalse(fresh_sample(float('nan'),1.,10.,9.))
    def test_both_planners_share_a_reachable_actuator_envelope(self):
        for vx in (-.3,0.,.3):
            for vy in (-.3,0.,.3):
                for wz in (-1.,0.,1.):
                    x,y,a=limit_velocity(vx,vy,wz,.24,.6)
                    self.assertLessEqual(math.hypot(x,y),.24+1e-9)
                    self.assertLessEqual(abs(a),.6+1e-9)
                    self.assertLessEqual(max(abs(x-y-.27*a),abs(x+y+.27*a),abs(x+y-.27*a),abs(x-y+.27*a)),.432+1e-9)
        self.assertEqual(limit_velocity(float('nan'),0.,0.),(0.,0.,0.))


class MetricsTests(unittest.TestCase):
    def test_position_rmse_angle_wrap_and_real_time_factor(self):
        rows=[dict(est_x=.03,est_y=.04,est_yaw=-math.pi+.01,true_x=0.,true_y=0.,true_yaw=math.pi-.01,
                   odom_x=.3,odom_y=.4,phase='navigating',sim_time=t,wall_time=2*t,true_speed=.12,cmd_speed=0.,cmd_yaw=0.) for t in (0.,1.,2.)]
        report=summarize(rows)
        self.assertAlmostEqual(report['position_error_m']['rmse'],.05)
        self.assertAlmostEqual(report['odom_error_m']['rmse'],.5)
        self.assertAlmostEqual(report['heading_error_deg']['rmse'],math.degrees(.02))
        self.assertEqual(report['real_time_factor'],.5)
        self.assertEqual(report['moving_zero_command_fraction'],1.)
    def test_no_measurements_are_not_a_success(self):
        self.assertFalse(summarize([])['valid'])
    def test_incomplete_or_nonfinite_goal_measurements_cannot_pass(self):
        goals=[dict(name='start',position_error_m=.01,heading_error_deg=1.)]
        self.assertFalse(goal_accuracy(goals,['start','finish'])['accuracy_pass'])
        self.assertEqual(goal_accuracy(goals,['start','finish'])['missing_goals'],['finish'])
        goals.append(dict(name='finish',position_error_m=.03,heading_error_deg=3.))
        self.assertTrue(goal_accuracy(goals,['start','finish'])['accuracy_pass'])
        goals[-1]['position_error_m']=float('nan')
        self.assertFalse(goal_accuracy(goals,['start','finish'])['accuracy_pass'])


class PlannerConfigurationTests(unittest.TestCase):
    def test_dwa_samples_zero_turn_and_does_not_latch_bad_xy(self):
        profile=yaml.safe_load((ROOT/'config/navigation/profiles/balanced.yaml').read_text())['DWAPlannerROS']
        self.assertEqual(profile['vtheta_samples']%2,1)
        self.assertFalse(profile['latch_xy_goal_tolerance'])
        self.assertEqual(profile['forward_point_distance'],0.)
        self.assertLessEqual(profile['yaw_goal_tolerance'],.06)
    def test_teb_is_holonomic_and_keeps_full_footprint(self):
        profile=yaml.safe_load((ROOT/'config/navigation/teb.yaml').read_text())['TebLocalPlannerROS']
        self.assertGreater(profile['max_vel_y'],0)
        self.assertEqual(profile['weight_kinematics_nh'],0)
        self.assertEqual(profile['footprint_model']['vertices'],[[.20,.18],[.20,-.18],[-.20,-.18],[-.20,.18]])
    def test_cartographer_does_not_duplicate_encoder_tf(self):
        lua=(ROOT/'config/cartographer/community_2d.lua').read_text()
        self.assertIn('published_frame = "odom"',lua)
        self.assertIn('provide_odom_frame = false',lua)
        launch=E.parse(ROOT/'launch/navigation.launch').getroot()
        amcl=launch.find("node[@name='amcl']")
        self.assertIn("== 'amcl'",amcl.get('if'))
        self.assertIn('pure_localization_trimmer',(ROOT/'config/cartographer/community_localization.lua').read_text())

class MapAlignmentTests(unittest.TestCase):
    def test_cartographer_export_rotates_into_competition_frame(self):
        spec=importlib.util.spec_from_file_location('align_cartographer_map',ROOT/'scripts/align_cartographer_map.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory);pixels=np.array([[0,254],[205,100]],dtype=np.uint8)
            Image.fromarray(pixels).save(directory/'native.pgm')
            (directory/'native.yaml').write_text(yaml.safe_dump(dict(image='native.pgm',resolution=1.,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
            (directory/'state.json').write_text(json.dumps(dict(anchor=[0.,0.,math.pi/2])))
            metadata=module.align(directory/'native.yaml',directory/'state.json',directory/'aligned.yaml')
            self.assertEqual(metadata['origin'],[-2.,0.,0.])
            np.testing.assert_array_equal(np.array(Image.open(directory/'aligned.pgm')),np.rot90(pixels))
            with self.assertRaises(ValueError):module.align(directory/'native.yaml',directory/'state.json',directory/'aligned.yaml')


if __name__=='__main__':unittest.main()
