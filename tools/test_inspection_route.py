"""Pure offline checks; no ROS node is constructed."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import math
import yaml

spec=importlib.util.spec_from_file_location('route',Path(__file__).with_name('inspection_route.py'))
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)

class RouteTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=0.,y=0.,z=0.,yaw=0.)
        self.a=r.Arrival(100,self.pose,3.)
        self.nav=dict(phase='reached',reason='arrival_latched',reached=True)

    def samples(self,p=None):
        for stamp in (101,102,103):self.a.sample(stamp,p or self.pose)

    def test_old_arrival_cannot_advance_without_reset(self):
        self.samples()
        self.assertFalse(self.a.check(10,self.nav,True))
        self.assertFalse(self.a.check(20,self.nav,True))

    def test_three_fresh_distinct_samples_and_dwell(self):
        self.a.reset_seen=True
        for stamp in (99,100,101,101):self.a.sample(stamp,self.pose)
        self.assertFalse(self.a.check(10,self.nav,True))
        self.samples()
        self.assertFalse(self.a.check(11,self.nav,True))
        self.assertFalse(self.a.check(13.99,self.nav,True))
        self.assertTrue(self.a.check(14,self.nav,True))

    def test_wrong_position_or_yaw_is_not_arrival(self):
        self.a.reset_seen=True
        for p in (dict(self.pose,x=.101),dict(self.pose,yaw=math.radians(21))):
            self.a.samples.clear();self.samples(p)
            self.assertFalse(self.a.check(10,self.nav,True))
            self.assertFalse(self.a.check(20,self.nav,True))

    def test_movement_resets_dwell(self):
        self.a.reset_seen=True;self.samples()
        self.a.check(10,self.nav,True)
        self.assertFalse(self.a.check(12,self.nav,False))
        self.assertFalse(self.a.check(13,self.nav,True))
        self.assertFalse(self.a.check(15,self.nav,True))
        self.assertTrue(self.a.check(16,self.nav,True))

    def test_approach_even_at_position_does_not_complete(self):
        self.a.reset_seen=True;self.samples()
        self.assertFalse(self.a.check(20,dict(self.nav,phase='approach'),True))

    def test_yaw_wrap(self):
        self.assertTrue(r.inside(dict(self.pose,yaw=math.pi-.01),dict(self.pose,yaw=-math.pi+.01)))

    def test_config_existing_coordinates_and_single_lap(self):
        cfg=r.load_config(r.DEFAULT_CONFIG)
        self.assertEqual(cfg['sequence'],['START','P1','P2','P3','P4','START'])
        self.assertIsNone(cfg['points']['START'])
        self.assertAlmostEqual(cfg['points']['P1']['x'],1.532753825)

    def test_invalid_pose_and_sequence_rejected(self):
        cfg=r.load_config(r.DEFAULT_CONFIG)
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'route.yaml'
            cfg['points']['P1']['x']=float('nan')
            r.atomic_write(file,yaml.safe_dump(cfg))
            with self.assertRaises(ValueError):r.load_config(file)
            cfg=r.load_config(r.DEFAULT_CONFIG);cfg['sequence'].append('P1')
            r.atomic_write(file,yaml.safe_dump(cfg))
            with self.assertRaises(ValueError):r.load_config(file)

if __name__=='__main__':unittest.main()
