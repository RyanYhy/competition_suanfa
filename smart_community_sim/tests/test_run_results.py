"""Owned result cleanup and evidence separation; temporary files only."""
import sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from smart_community_sim.run_results import RunResults
from smart_community_sim.terminal_log import LOG_NAME


class LatestResultsTests(unittest.TestCase):
    def test_next_run_clears_old_images_counts_and_completed_status(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'results';store=RunResults(root,{'A':9,'B':9},[{'name':'finish'}])
            store.begin({'backend':'test'});first=store.run_id
            (root/'yolo_images/old.png').write_bytes(b'old')
            (root/'navigation').mkdir();(root/'navigation/report.json').write_text('{"passed":true}')
            (root/'vision_evaluation.json').write_text('{"passed":true}')
            (root/LOG_NAME).write_text('old completed task')
            old_counts={d:dict(total=9,residents=8,visitors=1,tracks=[]) for d in ('A','B')}
            old_counts['plates']=[dict(observation='observe_plate_1',text='粤B12345')]
            store.update_counts(old_counts,force=True);store.goal('finish');store.status('complete','old complete')
            store.begin({'backend':'test'})
            self.assertNotEqual(first,store.run_id);self.assertEqual(store.state,'ready')
            self.assertEqual(store.counts['A']['total'],0);self.assertEqual(store.goals,[])
            self.assertEqual(store.counts['plates'],[])
            self.assertFalse((root/'yolo_images/old.png').exists())
            self.assertFalse((root/'navigation').exists())
            self.assertFalse((root/'vision_evaluation.json').exists())
            self.assertEqual((root/LOG_NAME).read_text(),'')

    def test_default_mode_only_saves_three_requested_items(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'results';store=RunResults(root,{'A':9,'B':9},[]);store.begin({})
            frame=np.full((20,20,3),120,np.uint8)
            record=dict(stamp=1.,frame=1,observation='observe_A_north',detections=[{'box':[0,0,10,10]}])
            store.save_image('observe_A_north',frame,frame,record,(1,),crop=frame)
            store.frame(record);store.event(dict(sequence=1),frame)
            counts={d:dict(total=1,residents=1,visitors=0,tracks=[{'id':'internal'}]) for d in ('A','B')}
            counts['plates']=[];store.finalize(counts)
            self.assertEqual({p.name for p in root.iterdir()},{'yolo_images','识别总览.jpg',LOG_NAME})
            self.assertTrue(store.marker(root).is_file())
            self.assertEqual(store.counts['A']['total'],1)
            self.assertNotIn('detections',store.images['observe_A_north'])

    def test_explicit_debug_saves_evidence_elsewhere(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'results';debug=RunResults(root,{'A':9,'B':9},[],save_debug=True);debug.begin({'test':True})
            frame=np.full((20,20,3),120,np.uint8)
            record=dict(stamp=1.,frame=1,observation='observe_plate_1',detections=[])
            debug.save_image('observe_plate_1',frame,frame,record,(1,),crop=frame)
            debug.frame(record);debug.event(dict(sequence=1),frame)
            for name in ('raw_images/observe_plate_1.png','plate_crops/observe_plate_1.png',
                         'records/configuration.json','records/frames.jsonl','records/events/0001.png'):
                self.assertTrue((debug.debug_root/name).exists(),name)
            self.assertEqual({p.name for p in root.iterdir()},{'yolo_images','识别总览.jpg',LOG_NAME})
            debug.begin({})
            self.assertFalse((debug.debug_root/'records/frames.jsonl').exists())
            with self.assertRaises(ValueError):RunResults(root,{},[],save_debug=True,debug_root=root/'extra')

    def test_failed_run_keeps_partial_progress_and_builds_actual_overview(self):
        with tempfile.TemporaryDirectory() as tmp:
            route=[{'name':'observe_A_north'},{'name':'finish'}]
            root=Path(tmp)/'results';store=RunResults(root,{'A':9,'B':9},route);store.begin({})
            blank=(root/'识别总览.jpg').read_bytes()
            frame=np.full((20,20,3),120,np.uint8)
            store.save_image('observe_A_north',frame,frame,dict(stamp=1.,frame=1,observation='observe_A_north',detections=[]),(1,))
            store.goal('observe_A_north');store.status('stopped','车牌覆盖不足')
            self.assertEqual(store.state,'stopped');self.assertEqual(store.goals,['observe_A_north'])
            self.assertIn('车牌覆盖不足',store.detail)
            self.assertNotEqual(blank,(root/'识别总览.jpg').read_bytes())

    def test_late_blank_frame_cannot_replace_best_verification_picture(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=RunResults(Path(tmp)/'results',{'A':9,'B':9},[]);store.begin({})
            record=dict(stamp=1.,frame=1,observation='observe_A_north',detections=[])
            frame=np.full((20,20,3),120,np.uint8)
            self.assertTrue(store.save_image('observe_A_north',frame,frame,record,(3,0,2.)))
            self.assertFalse(store.save_image('observe_A_north',frame*0,frame*0,dict(record,stamp=2.),(0,0,0)))
            self.assertEqual(store.images['observe_A_north']['stamp'],1.)

    def test_unowned_directory_and_symlinks_are_not_deleted(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'results';(root/'yolo_images').mkdir(parents=True);(root/'yolo_images/keep.txt').write_text('user data')
            store=RunResults(root,{'A':9,'B':9},[])
            with self.assertRaises(ValueError):store.begin({})
            self.assertTrue((root/'yolo_images/keep.txt').exists())
            alias=Path(tmp)/'alias';alias.symlink_to(root,target_is_directory=True)
            with self.assertRaises(ValueError):RunResults(alias,{},[])
