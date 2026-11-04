"""Host dry runs of t03_k_ramp with fake genomix components (compressed timeline)."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.tests import t03_k_ramp as T
from noc.tests.t02_live_k import af_baseline
from noc.tests.fake_genomix import make_module, FakeWorld


class TestProfile(unittest.TestCase):
    def test_nominal_profile(self):
        pts, ph = T.k_profile()
        self.assertEqual(pts, [(30.0, 10.0), (50.0, 10.0), (51.0, 20.0), (71.0, 20.0), (72.0, 10.0), (92.0, 10.0)])
        self.assertEqual([p[0] for p in ph], ['hold', 'ramp', 'hold', 'ramp', 'hold'])
        self.assertEqual(ph[-1][2], T.WAYPOINTS[3][0])          # last hold ends at the retreat
        self.assertLess(ph[0][1], T.TF)
        self.assertGreater(T.T_SETTLED, T.WAYPOINTS[2][0] + T.WAYPOINTS[2][1][4])
        for lbl, t in T.SNAPSHOT_TIMES.items():               # no snapshot inside a ramp
            self.assertFalse(any(p[0] == 'ramp' and p[1] <= t <= p[2] for p in ph), lbl)

    def test_k_at_ramp_steps(self):
        _, ph = T.k_profile()
        ks = [T.k_at(round(i * T.DT, 10), ph)[0] for i in range(int(49.9 / T.DT), int(51.1 / T.DT))]
        steps = np.diff([k for k in ks])
        self.assertTrue(np.all((np.abs(steps) < 1e-9) | (np.abs(steps - 0.2) < 1e-6)), steps)
        self.assertEqual(T.k_at(50.0, ph)[1], T.PHASE_RAMP)
        self.assertEqual(T.k_at(51.0, ph)[:2], (20.0, T.PHASE_HOLD))
        self.assertEqual(T.k_at(29.0, ph), (None, T.PHASE_NONE, -1))
        self.assertEqual(T.k_at(92.0, ph)[0], 10.0)
        self.assertEqual(max(k for k in ks), 20.0)


FAST = dict(DT=0.001, T_WO_ZERO=0.02, TF=0.40, T_SETTLED=0.05, HOLD_S=0.08, RAMP_RATE=500.0,
            WAYPOINTS=[(0.005, (0.0, 0.0, 1.0, 0, 5.0)), (0.01, (1.2, 0.0, 1.0, 0, 4.0)),
                       (0.03, (1.55, 0.0, 1.0, 0, 2.0)), (0.30, (1.0, 0.0, 1.0, 0, 4.0)),
                       (0.33, (0.0, 0.0, 0.0, 0, 5.0))],
            SNAPSHOT_TIMES={'settled': 0.05}, PENDING_TIMEOUT_S=0.05)


def world(**kw):
    w = FakeWorld()
    w.des_valid_after_goto = True
    w.ext_empty_until_goto = True
    for k, v in kw.items():
        setattr(w, k, v)
    return w


class TestDryRun(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def _run(self, w=None, check=False, **extra):
        mod = make_module(w or world())
        with mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run'),
                                 CHECK_DIR=os.path.join(self.tmp, 'chk'), **{**FAST, **extra}):
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
        self.assertEqual(rc, 0, meta['abort'])
        self.assertIsNone(meta['abort'])
        # 10 -> 20 at 500 N/m/s with dt 1 ms: 0.5 N/m per call, 20 calls per ramp
        ks = [c['K_x'] for c in calls]
        self.assertEqual(len(calls), 40)
        self.assertEqual(max(ks), 20.0)
        self.assertEqual(ks[19], 20.0)
        self.assertEqual(ks[-1], 10.0)
        np.testing.assert_allclose(np.abs(np.diff(ks[:20])), 0.5, atol=1e-6)
        self.assertTrue(all(c['kind'] == T.CALL_RAMP and c['verify'] for c in calls))
        base = af_baseline()
        for c in mod.world.af_calls:   # only K[0] ever changes
            self.assertEqual(c['mass'], base['mass'])
            np.testing.assert_allclose(c['B'], base['B'])
            np.testing.assert_allclose(c['J'], base['J'])
            np.testing.assert_allclose(c['K'][1:], base['K'][1:])
        self.assertTrue(np.all(d['af_match'][np.isfinite(d['af_match'])] == 1.0))
        self.assertTrue(np.all(np.abs(d['af.K'][:, 0] - d['k_target_x']) <= 1e-9))
        self.assertIn(T.PHASE_RAMP, d['k_phase'].tolist())
        # pending: EMPTY_POSTER before the first goto is counted as pending, not as a read error
        self.assertEqual(meta['read_error_counts'], {})
        self.assertGreater(meta['read_error_counts_pending'].get('ext_wrench', 0), 0)
        self.assertTrue(all(v is not None for v in meta['pending']['ready_wall'].values()))
        self.assertEqual(json.load(open(os.path.join(self.tmp, 'run', 'preflight.json')))['ext_wrench']['status'],
                         'pending')
        self.assertTrue(d['pending.ext_wrench'][0] and not d['pending.ext_wrench'][-1])
        self.assertIn('ramp 10->20', ev)
        self.assertIn('hold K_x=20', ev)
        self.assertEqual(ev[-1], 'stop')

    def test_pending_never_ready_aborts(self):
        rc, mod = self._run(world(des_valid_after_goto=False))
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('pending sources not ready', meta['abort']['reasons'][-1])
        self.assertIn('phynt_des', meta['abort']['reasons'][-1])
        self.assertIsNone(meta['pending']['ready_wall']['phynt_des'])
        self.assertEqual(calls, [])
        self.assertIn('SAFETY land goto', ev)
        self.assertEqual(ev[-1], 'stop')

    def test_read_error_after_ready_is_counted(self):
        w = world()
        rc, mod = self._run(w)
        d, meta, *_ = self._load()
        self.assertEqual(meta['read_error_counts'], {})
        # a genuine read error later in flight is counted as a read error
        w2 = world()
        mod2 = make_module(w2)
        orig = mod2.uavatt.wrench_measure
        n = {'c': 0}

        def flaky():
            n['c'] += 1
            if n['c'] == 200:
                raise RuntimeError('poster read failed')
            return orig()
        mod2.uavatt.wrench_measure = flaky
        with mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run2'), CHECK_DIR=self.tmp, **FAST):
            T.run_flight(load=lambda dd: mod2)
        meta2 = json.loads(str(np.load(os.path.join(self.tmp, 'run2', T.NPZ_NAME))['meta_json']))
        self.assertEqual(meta2['read_error_counts'], {'wrench_meas': 1})

    def test_k_not_applied_aborts(self):
        rc, mod = self._run(world(af_ignore_k=True))
        d, meta, calls, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('AFMismatch', meta['abort']['reasons'][-1])
        self.assertEqual(len(calls), 1)
        self.assertEqual(ev[-1], 'stop')

    def test_nonpending_preflight_failure_refuses_takeoff(self):
        mod = make_module(world(), rotor_measure_ok=False)
        mod.rotorcraft.rotor_measure = lambda: (_ for _ in ()).throw(RuntimeError('no poster'))
        with mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run'), CHECK_DIR=self.tmp, **FAST), \
                mock.patch('noc.common.setup_06a.setup', lambda m: {}):
            with self.assertRaises(RuntimeError):
                T.run_flight(load=lambda d: mod)
        self.assertEqual(mod.world.gotos, [])

    def test_check_ok_with_pending(self):
        rc, mod = self._run(check=True)
        self.assertEqual(rc, 0)
        with open(os.path.join(self.tmp, 'chk', 'check.json')) as f:
            res = json.load(f)
        self.assertEqual(res['preflight']['ext_wrench']['status'], 'pending')
        self.assertFalse(res['preflight']['ext_wrench']['ok'])     # EMPTY_POSTER recorded as is
        self.assertEqual([c['K_x'] for c in res['k_calls']], [10.0, 20.0, 10.0])
        self.assertEqual(mod.world.af['K'][0], 10.0)
        self.assertEqual(mod.world.gotos, [])


if __name__ == '__main__':
    unittest.main()
