"""Report role changes honestly and retain the final footer across late callbacks."""
import contextlib,io,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from smart_community_sim.observation_report import ObservationReports
from smart_community_sim.observation_tracker import PersonTracker
from smart_community_sim.task_progress import observation_lines,mission_summary
from smart_community_sim.terminal_log import TerminalLog,LOG_NAME


class ObservationReportTests(unittest.TestCase):
    def test_voted_role_uses_its_own_score_and_counts_one_instance(self):
        reports=ObservationReports();tracker=PersonTracker()
        for role,score in [('resident',.9),('visitor',.55),('visitor',.54),('visitor',.53)]:
            update=tracker.update('A',[(dict(role=role,confidence=score),[0.,1.])])
            reports.update(dict(observation='observe_A_south',task='person',detections=[d for d,t in update]))
        result=reports.report('observe_A_south',tracker.counts())
        self.assertEqual(len(result['people']),1);self.assertEqual(result['people'][0]['role'],'visitor')
        self.assertAlmostEqual(result['people'][0]['confidence'],.55)
        self.assertEqual(result['people'][0]['frames'],4)
        self.assertIn('55.0%',observation_lines(result)[1]);self.assertNotIn('90.0%',observation_lines(result)[1])

    def test_unconfirmed_person_and_unaccepted_plate_are_not_reported_as_success(self):
        reports=ObservationReports()
        reports.update(dict(observation='observe_A_north',task='person',detections=[dict(track_id='A_person_1',role='resident',confidence=.8)]))
        result=reports.report('observe_A_north',{'A':dict(tracks=[])})
        self.assertEqual(result['people'],[]);self.assertEqual(result['pending'],1)
        target=dict(confidence=.9,ocr_candidates=[dict(text='粤B12345',ocr_confidence=.7)])
        reports.update(dict(observation='observe_plate_1',task='plate',detections=[target]),targets=[target])
        result=reports.report('observe_plate_1',dict(plates=[]))
        self.assertIsNone(result['plate']);self.assertIn('未确认',observation_lines(result)[0])
        self.assertIn('候选',observation_lines(result)[1])

    def test_lamp_unknown_and_neighbor_plate_cannot_raise_reported_confidence(self):
        reports=ObservationReports()
        reports.update(dict(observation='north_wait',task='signal',detections=[dict(color='green',confidence=.6),dict(color='unknown',confidence=.99)]),stable='green')
        self.assertAlmostEqual(reports.report('north_wait',{})['colors']['green']['confidence'],.6)
        target=dict(confidence=.65,ocr_candidates=[dict(text='粤B12345',ocr_confidence=.8)])
        neighbor=dict(confidence=.99,ocr_candidates=[dict(text='苏A99999',ocr_confidence=.99)])
        reports.update(dict(observation='observe_plate_1',task='plate',detections=[target,neighbor]),targets=[target])
        self.assertEqual(reports.report('observe_plate_1',dict(plates=[]))['candidate']['text'],'粤B12345')


class TerminalLogTests(unittest.TestCase):
    def test_summary_is_last_plain_text_even_after_late_warning_and_reset(self):
        with tempfile.TemporaryDirectory() as tmp,contextlib.redirect_stdout(io.StringIO()) as output:
            log=TerminalLog(tmp);log.emit('巡航就绪');log.begin(19)
            log.emit('[识别点结果] A区：社区人员，置信度80.0%',color='33')
            log.warning('/move_base','真实警告')
            summary=mission_summary([{'name':'finish'}],['finish'],{}, {},'stopped','视觉覆盖不足')
            log.finish(summary);path=Path(tmp)/LOG_NAME;finished=path.read_text()
            log.warning('/move_base','迟到消息');log.emit('退出提示')
            self.assertEqual(path.read_text(),finished)
            self.assertTrue(finished.rstrip().endswith('========== 汇总结束 =========='))
            self.assertIn('未通过',finished);self.assertIn('真实警告',finished)
            self.assertNotIn('\033[',finished);self.assertIn('\033[33m',output.getvalue())
            log.prepare();log.begin(19)
            self.assertNotIn('视觉覆盖不足',path.read_text())
            log.finish(mission_summary([],[],{}, {},'complete','诊断完成',False),True)
            self.assertIn('全部完成',path.read_text())


if __name__=='__main__':unittest.main()
