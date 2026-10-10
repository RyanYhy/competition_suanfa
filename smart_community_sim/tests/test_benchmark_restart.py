"""A passive observer must not mix measurements across patrol restarts."""
import importlib.util,json,sys,threading,unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
spec=importlib.util.spec_from_file_location('live_benchmark',Path(__file__).resolve().parents[1]/'scripts/navigation_benchmark.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class ObserverRestartTests(unittest.TestCase):
    def observer(self):
        b=module.Benchmark.__new__(module.Benchmark);b.lock=threading.RLock()
        b.continuous=True;b.done=True;b.saved=True;b.truth=deque([object()]);b.odom={1:object()}
        b.rows=[{'old':True}];b.goals=[{'old':True}];b.received=10;b.dropped=2;b.latest_truth=object()
        b.started=0;b.finished=1;b.phase='complete';b.detail='old run';return b

    def test_restart_clears_previous_samples_and_goals(self):
        b=self.observer();b.on_status(SimpleNamespace(data=json.dumps({'state':'navigating','detail':'start_centerline'})))
        self.assertFalse(b.saved);self.assertFalse(b.done)
        self.assertEqual(b.rows,[]);self.assertEqual(b.goals,[]);self.assertFalse(b.truth);self.assertFalse(b.odom)
        self.assertEqual(b.received,0);self.assertEqual(b.dropped,0);self.assertIsNone(b.latest_truth)
        self.assertEqual(b.phase,'navigating');self.assertGreater(b.started,0)

    def test_completed_observer_does_not_fill_idle_buffers(self):
        b=self.observer();b.on_truth(object());b.on_odom(object())
        self.assertEqual(b.received,10);self.assertEqual(len(b.truth),1);self.assertEqual(len(b.odom),1)
