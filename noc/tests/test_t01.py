"""Host dry runs of setup_06a and t01_af_on with fake genomix components."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.common import setup_06a
from noc.common import telekyb_io as io
from noc.tests import t01_af_on as T
from noc.tests.fake_genomix import make_module, FakeWorld


class TestReaders(unittest.TestCase):
    def test_rotor_measure_and_uav_input(self):
        m = make_module()
        out, valid = io.read_rotor_measure(m.rotorcraft)
        self.assertEqual(out['n_rotors'], 6)
        self.assertTrue(np.all(np.isnan(out['velocity'][6:])))
        self.assertTrue(valid['velocity'])
        out, valid = io.read_uavpos_uav_input(m.uavpos)
        self.assertAlmostEqual(out['thrust'][2], 26.7)

    def test_invalid_sample_shapes(self):
        out, valid = io.invalid_sample(('velocity', 'n_rotors'))
        self.assertEqual(out['velocity'].shape, (io.MAX_ROTORS,))
        self.assertFalse(valid['velocity'])


class TestSetup06a(unittest.TestCase):
    def test_connects_after_checks(self):
        m = make_module()
        rep = setup_06a.setup(m)
        self.assertEqual(m.connections, [('uavatt', {'local': 'rotor_measure',
                                                     'remote': 'rotorcraft/rotor_measure'})])
        self.assertTrue(rep['doc_check']['checked'])

    def test_missing_port_raises_and_does_not_connect(self):
        m = make_module(rotor_measure_ok=False)
        with self.assertRaises(setup_06a.PortCheckError):
            setup_06a.setup(m)
        self.assertEqual(m.connections, [])


class TestSafety(unittest.TestCase):
    def test_seg_dist(self):
        a, b = np.zeros(3), np.array([1.0, 0, 0])
        self.assertAlmostEqual(T.seg_dist(np.array([0.5, 0.3, 0]), a, b), 0.3)
        self.assertAlmostEqual(T.seg_dist(np.array([2.0, 0, 0]), a, b), 1.0)

    def _row(self, m):
        return {k: r(getattr(m, c)) for k, (c, r, _) in T.all_sources(m.phynt).items()}

    def test_force_trip_needs_persistence_and_calibration(self):
        m = make_module()
        m.world.pos = np.array([1.37, 0, 1.0])
        m.world.f_ext = np.array([-4.0, 0, 0])
        s = T.Safety(T.SAFETY)
        seg = (np.array([1.2, 0, 1.0]), np.array([1.55, 0, 1.0]))
        for _ in range(10):
            trips, _ = s.check(self._row(m), False, seg)   # WO not calibrated -> ignored
            self.assertEqual(trips, [])
        for k in range(T.SAFETY['f_persist']):
            trips, sig = s.check(self._row(m), True, seg)
        self.assertTrue(any('external_wrench' in t for t in trips), trips)

    def test_rotor_low_only_when_flying(self):
        m = make_module()
        m.world.rotor_vel = [10.0] * 6
        s = T.Safety(T.SAFETY)
        seg = (m.world.pos.copy(), m.world.pos.copy())
        for _ in range(5):
            trips, _ = s.check(self._row(m), False, seg)
        self.assertEqual(trips, [])  # on the ground


class TestDryRun(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def _patch(self, world):
        mod = make_module(world)
        return mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run'),
                                   CHECK_DIR=os.path.join(self.tmp, 'chk'),
                                   TF=0.4, T_WO_ZERO=0.05, SNAPSHOT_TIMES={'contact_1': 0.2}), mod

    def test_check_mode(self):
        p, mod = self._patch(FakeWorld())
        with p:
            rc = T.run_check(load=lambda d: mod)
        self.assertEqual(rc, 0)
        with open(os.path.join(self.tmp, 'chk', 'check.json')) as f:
            res = json.load(f)
        self.assertTrue(res['preflight']['wrench_meas']['valid']['force'])

    def test_flight_normal(self):
        p, mod = self._patch(FakeWorld())
        with p:
            T.run_flight(load=lambda d: mod)
        d = np.load(os.path.join(self.tmp, 'run', 't01_af_on.npz'))
        meta = json.loads(str(d['meta_json']))
        self.assertIsNone(meta['abort'])
        self.assertEqual(len(d['i']), 20)
        self.assertTrue(all(g[5] for g in mod.world.gotos))  # every waypoint goto used ack=True
        for k in ('uav_input.thrust', 'rotor_meas.velocity', 'phynt_des.vel', 'phynt_des.valid.vel',
                  'wrench_meas.force', 'ext_wrench.force', 'phynt_log_bytes', 'safety.f_cmd_xy'):
            self.assertIn(k, d.files)

    def test_flight_safety_trip(self):
        w = FakeWorld()
        w.f_ext = np.array([-5.0, 0, 0])
        p, mod = self._patch(w)
        with p:
            T.run_flight(load=lambda d: mod)
        d = np.load(os.path.join(self.tmp, 'run', 't01_af_on.npz'))
        meta = json.loads(str(d['meta_json']))
        self.assertIsNotNone(meta['abort'])
        self.assertIn('external_wrench', meta['abort']['reasons'][0])
        ev = [e['name'] for e in json.loads(str(d['events_json']))]
        self.assertIn('SAFETY retreat goto', ev)
        self.assertIn('SAFETY land goto', ev)
        self.assertEqual(ev[-1], 'stop')
        self.assertFalse(mod.world.gotos[-1][5])  # retreat/land are blocking (no ack)


if __name__ == '__main__':
    unittest.main()
