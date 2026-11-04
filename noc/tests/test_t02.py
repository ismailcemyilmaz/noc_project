"""Host dry runs of t02_live_k with fake genomix components (compressed timeline)."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.common import telekyb_io as io
from noc.tests import t02_live_k as T
from noc.tests.fake_genomix import make_module, FakeWorld


class TestAFHelpers(unittest.TestCase):
    def test_read_af_parameters(self):
        m = make_module()
        out, valid = io.read_af_parameters(m.phynt)
        self.assertEqual(out['K'].shape, (6,))
        self.assertEqual(out['J'].shape, (9,))
        self.assertEqual(out['mass'], 5.0)
        self.assertTrue(valid['K'] and not valid['ts'])

    def test_read_af_parameters_bad_shape_raises(self):
        m = make_module()
        m.world.af['K'] = [10.0, 100.0]
        with self.assertRaises(io.PortFormatError):
            io.read_af_parameters(m.phynt)

    def test_invalid_sample_af(self):
        out, valid = io.invalid_sample(io.AF_FIELDS)
        self.assertEqual(out['B'].shape, (6,))
        self.assertTrue(np.isnan(out['mass']))
        self.assertFalse(any(valid.values()))

    def test_baseline_matches_fake_and_command_changes_only_kx(self):
        base = T.af_baseline()
        m = make_module()
        read, _ = io.read_af_parameters(m.phynt)
        self.assertTrue(T.af_equal(read, base)[0])
        cmd = T.af_command(base, 15.0)
        self.assertEqual(cmd['K'][0], 15.0)
        np.testing.assert_array_equal(cmd['K'][1:], base['K'][1:])
        np.testing.assert_array_equal(cmd['B'], base['B'])   # B not rescaled
        np.testing.assert_array_equal(cmd['J'], base['J'])
        self.assertEqual(cmd['mass'], base['mass'])

    def test_schedule(self):
        self.assertEqual([k for _, k in T.K_SCHEDULE], [10.0, 12.0, 15.0, 20.0, 15.0, 10.0])
        self.assertEqual(np.diff([t for t, _ in T.K_SCHEDULE]).tolist(), [12.0] * 5)
        self.assertEqual(T.T_REPEAT, 102.0)
        self.assertLess(T.T_REPEAT + T.N_REPEAT * T.DT, T.WAYPOINTS[3][0])   # repeats end before retreat
        self.assertEqual(T.WAYPOINTS[2][1][:3], (T.X_CONTACT, 0.0, 1.0))


FAST = dict(DT=0.001, T_WO_ZERO=0.02, TF=0.40, T_SETTLED=0.05, T_REPEAT=0.15, N_REPEAT=20,
            WAYPOINTS=[(0.0, (0.0, 0.0, 1.0, 0, 5.0)), (0.01, (1.2, 0.0, 1.0, 0, 4.0)),
                       (0.03, (1.55, 0.0, 1.0, 0, 2.0)), (0.30, (1.0, 0.0, 1.0, 0, 4.0)),
                       (0.33, (0.0, 0.0, 0.0, 0, 5.0))],
            K_SCHEDULE=[(0.05 + i * 0.015, k) for i, k in enumerate([10.0, 12.0, 15.0, 20.0, 15.0, 10.0])],
            SNAPSHOT_TIMES={'settled': 0.05})


class TestDryRun(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def _run(self, world=None, check=False):
        mod = make_module(world or FakeWorld())
        with mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run'),
                                 CHECK_DIR=os.path.join(self.tmp, 'chk'), **FAST):
            rc = (T.run_check if check else T.run_flight)(load=lambda d: mod)
        return rc, mod

    def _load(self):
        d = np.load(os.path.join(self.tmp, 'run', T.NPZ_NAME))
        meta = json.loads(str(d['meta_json']))
        calls = json.loads(str(d['k_calls_json']))
        ev = [e['name'] for e in json.loads(str(d['events_json']))]
        return d, meta, calls, ev

    def test_flight_normal(self):
        rc, mod = self._run()
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 0)
        self.assertIsNone(meta['abort'])
        sched = [c for c in calls if c['kind'] == T.CALL_SCHEDULE]
        rep = [c for c in calls if c['kind'] == T.CALL_REPEAT]
        self.assertEqual([c['K_x'] for c in sched], [12.0, 15.0, 20.0, 15.0, 10.0])
        self.assertEqual(len(rep), 20)
        self.assertTrue(all(c['verify'] for c in calls))
        self.assertTrue(all(c['set_s'] is not None and c['set_s'] >= 0 for c in calls))
        self.assertTrue(all(c['K_x'] == 10.0 for c in rep))
        base = T.af_baseline()
        for c in mod.world.af_calls:   # only K[0] ever changes
            self.assertEqual(c['mass'], base['mass'])
            np.testing.assert_allclose(c['B'], base['B'])
            np.testing.assert_allclose(c['J'], base['J'])
            np.testing.assert_allclose(c['K'][1:], base['K'][1:])
        self.assertEqual(mod.world.af['K'][0], 10.0)
        for k in ('k_target_x', 'af.K', 'af.mass', 'af.B', 'af_call.set_s', 'af_call.verify', 'af_match',
                  'pom.pos', 'phynt_des.pos', 'man_des.pos', 'ext_wrench.force', 'uav_input.thrust',
                  'wrench_meas.force', 'rotor_meas.velocity', 'late_s', 'read_s'):
            self.assertIn(k, d.files)
        self.assertTrue(np.all(d['af_match'][np.isfinite(d['af_match'])] == 1.0))
        self.assertEqual(sorted(set(d['k_target_x'].tolist())), [10.0, 12.0, 15.0, 20.0])
        self.assertTrue(all(g[5] for g in mod.world.gotos))   # ack=True
        self.assertEqual(ev[-1], 'stop')
        self.assertTrue(os.path.exists(os.path.join(self.tmp, 'run', 'k_calls.json')))

    def test_k_not_applied_aborts(self):
        w = FakeWorld()
        w.af_ignore_k = True
        rc, mod = self._run(w)
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('AFMismatch', meta['abort']['reasons'][-1])
        self.assertFalse(calls[-1]['verify'])
        self.assertEqual(len(calls), 1)
        self.assertIn('SAFETY retreat goto', ev)
        self.assertIn('SAFETY land goto', ev)
        self.assertEqual(ev[-1], 'stop')

    def test_set_error_aborts_and_is_recorded(self):
        w = FakeWorld()
        w.af_raise = RuntimeError('genomix: service failed')
        rc, mod = self._run(w)
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('genomix: service failed', calls[-1]['error'])
        self.assertIn('set_af_parameters: RuntimeError', meta['abort']['reasons'][-1])
        self.assertEqual(ev[-1], 'stop')

    def test_force_trip_aborts(self):
        w = FakeWorld()
        w.f_ext = np.array([-5.0, 0, 0])
        rc, mod = self._run(w)
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('external_wrench', meta['abort']['reasons'][0])
        self.assertEqual(ev[-1], 'stop')

    def test_initial_af_mismatch_refuses_takeoff(self):
        w = FakeWorld()
        w.af['K'][0] = 12.0
        with self.assertRaises(T.AFMismatch):
            self._run(w)
        d, meta, calls, ev = self._load()
        self.assertEqual(w.gotos, [])
        self.assertIn('AFMismatch', meta['error'])
        self.assertEqual(ev[-1], 'stop')

    def test_check_ok(self):
        rc, mod = self._run(check=True)
        self.assertEqual(rc, 0)
        with open(os.path.join(self.tmp, 'chk', 'check.json')) as f:
            res = json.load(f)
        self.assertTrue(res['initial_af']['ok'])
        self.assertTrue(res['k_calls'][0]['verify'])
        self.assertTrue(res['preflight']['af']['ok'])
        self.assertEqual(mod.world.gotos, [])

    def test_check_fails_on_af_mismatch(self):
        w = FakeWorld()
        w.af['mass'] = 4.0
        rc, mod = self._run(w, check=True)
        self.assertEqual(rc, 1)
        with open(os.path.join(self.tmp, 'chk', 'check.json')) as f:
            self.assertIn('AFMismatch', json.load(f)['error'])


if __name__ == '__main__':
    unittest.main()
