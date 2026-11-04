"""a03_k_ramp end to end on a t03 dry run plus synthetic component logs (series-stiffness
model with first-order lag), and on a t02 dry run for the comparison. Gate thresholds come
from the committed criteria file."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.analysis import a03_k_ramp as A
from noc.tests import t03_k_ramp as T3
from noc.tests import t02_live_k as T2
from noc.tests import test_a02
from noc.tests.fake_genomix import make_module, FakeWorld

HOLD = 0.3
SLOW = dict(DT=0.002, T_WO_ZERO=0.02, TF=1.35, T_SETTLED=0.1, HOLD_S=HOLD, RAMP_RATE=100.0,
            WAYPOINTS=[(0.005, (0.0, 0.0, 1.0, 0, 5.0)), (0.01, (1.2, 0.0, 1.0, 0, 4.0)),
                       (0.03, (1.55, 0.0, 1.0, 0, 0.05)), (1.2, (1.0, 0.0, 1.0, 0, 4.0)),
                       (1.25, (0.0, 0.0, 0.0, 0, 5.0))],
            SNAPSHOT_TIMES={}, PENDING_TIMEOUT_S=0.2)
D0, TAU, NOISE = 0.184, 0.02, 0.0005
PATCH_A = dict(MA_S=0.005, DT=0.002, RAMP_GAP_S=0.05, T02_WIN_S=0.02)


def synth(log_dir, npz, f_scale=1.0, fx_step=None, xd_step=None):
    d = np.load(os.path.join(log_dir, npz))
    t_npz, k_npz = d['t_wall'], d['k_target_x']
    ts = np.arange(t_npz[0], t_npz[-1], 0.0005)
    k = k_npz[np.clip(np.searchsorted(t_npz, ts, side='right') - 1, 0, None)]
    F_ss = D0 / (1 / 5.0 + 1 / k)
    a = 1 - np.exp(-np.diff(ts) / TAU)

    def lag(y):
        out = np.empty_like(y); out[0] = y[0]
        for i in range(1, y.size):
            out[i] = out[i - 1] + a[i - 1] * (y[i] - out[i - 1])
        return out
    rng = np.random.default_rng(0)
    dev = lag(-F_ss / k) + rng.normal(0, NOISE / 5, ts.size)   # t02: max per-row |dxd| 0.06 mm
    F = lag(F_ss) * f_scale + rng.normal(0, 10 * NOISE, ts.size)
    if fx_step is not None:          # (t_wall, size): one-row step in fx
        F[ts >= fx_step[0]] += fx_step[1]
    if xd_step is not None:
        dev[ts >= xd_step[0]] += xd_step[1]
    x_nom = np.full_like(ts, 1.55)
    z = np.zeros_like(ts)
    test_a02.write_log(os.path.join(log_dir, 'uavpos.log'), ['ts', 'fx', 'xd', 'e_x', 'e_vx'], [ts, F, x_nom + dev, z, z])
    test_a02.write_log(os.path.join(log_dir, 'maneuver.log'), ['ts', 'x', 'vx', 'vy', 'vz'], [ts, x_nom, z, z, z])
    test_a02.write_log(os.path.join(log_dir, 'pom.log'), ['ts', 'x'], [ts, x_nom - D0])
    test_a02.write_log(os.path.join(log_dir, 'phynt.log'), ['ts', 'efx'], [ts, -F])


class TestA03(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.base = os.path.join(cls.tmp, 'base')
        w = FakeWorld(); w.des_valid_after_goto = True; w.ext_empty_until_goto = True
        mod = make_module(w)
        with mock.patch.multiple(T3, OUT_DIR=cls.base, CHECK_DIR=os.path.join(cls.tmp, 'c'), **SLOW):
            assert T3.run_flight(load=lambda d: mod) == 0
        cls.t02 = os.path.join(cls.tmp, 't02')
        mod2 = make_module(FakeWorld())
        with mock.patch.multiple(T2, OUT_DIR=cls.t02, CHECK_DIR=os.path.join(cls.tmp, 'c2'), **test_a02.SLOW):
            assert T2.run_flight(load=lambda d: mod2) == 0
        test_a02.synth_logs(cls.t02)
        cls.crit, cls.info = A.load_criteria()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def setUp(self):
        self.run = os.path.join(self.tmp, f'run_{self._testMethodName}')
        shutil.copytree(self.base, self.run)

    def _analyse(self, **kw):
        synth(self.run, T3.NPZ_NAME, **kw)
        with mock.patch.multiple(A, **PATCH_A):
            return A.analyse_t03(self.run, self.crit)

    def _ramps(self):
        d = np.load(os.path.join(self.run, T3.NPZ_NAME))
        calls = json.loads(str(d['k_calls_json']))
        with mock.patch.multiple(A, **PATCH_A):
            return A.group_ramps(calls)

    def test_criteria_file_is_committed_and_used(self):
        self.assertEqual(self.crit['criteria']['C3_ramp_max_abs_dfx']['max_N'], 0.064)
        self.assertEqual(self.crit['criteria']['C4_max_abs_dxd']['max_m'], 0.001)
        self.assertEqual(self.crit['criteria']['C5_steady_F_vs_series_model']['tol'], 0.15)
        self.assertTrue(self.info['committed'])

    def test_model_run_passes(self):
        R, *_ = self._analyse()
        self.assertEqual([r['K_x'] for r in R['C5']['holds']], [10.0, 20.0, 10.0])
        self.assertEqual([(r['k_from'], r['k_to']) for r in R['C3']['ramps']], [(10.0, 20.0), (20.0, 10.0)])
        self.assertEqual([r['n_calls'] for r in R['C3']['ramps']], [50, 50])
        for k in ('C1', 'C2', 'C3', 'C4', 'C5'):
            self.assertTrue(R['gate_A2'][k], (k, R[k]))
        self.assertTrue(R['gate_A2']['passed'])
        self.assertEqual(R['C1']['read_errors'], 0)
        self.assertGreater(R['C1']['read_errors_pending'].get('ext_wrench', 0), 0)
        # 2-sigma settling: first-order lag (tau 20 ms) after a 0.1 s ramp -> well inside the hold
        for r in R['C5']['holds'][1:]:
            s = r['settling_2sigma']['t_settle_s']
            self.assertIsNotNone(s)
            self.assertLess(s, HOLD / 2)

    def test_fx_step_in_ramp_fails_C3(self):
        r = self._ramps()[0]
        R, *_ = self._analyse(fx_step=((r['start'] + r['end']) / 2, 0.1))
        self.assertFalse(R['gate_A2']['C3'])
        self.assertFalse(R['C3']['ramps'][0]['passed'])
        self.assertTrue(R['C3']['ramps'][1]['passed'])
        self.assertFalse(R['gate_A2']['passed'])

    def test_fx_step_outside_ramp_does_not_fail_C3(self):
        r = self._ramps()[0]
        R, *_ = self._analyse(fx_step=(r['end'] + 0.1, 0.1))
        self.assertTrue(R['gate_A2']['C3'])

    def test_xd_jump_fails_C4(self):
        r = self._ramps()[1]
        R, *_ = self._analyse(xd_step=(r['end'] + 0.1, 0.002))
        self.assertFalse(R['gate_A2']['C4'])
        self.assertGreater(R['C4']['value_m'], 0.001)

    def test_force_off_by_30pct_fails_C5(self):
        R, *_ = self._analyse(f_scale=1.3)
        self.assertFalse(R['gate_A2']['C5'])
        self.assertFalse(R['gate_A2']['passed'])

    def test_read_error_fails_C1(self):
        p = os.path.join(self.run, T3.NPZ_NAME)
        d = dict(np.load(p))
        meta = json.loads(str(d['meta_json'])); meta['read_error_counts'] = {'pom': 1}
        d['meta_json'] = json.dumps(meta)
        np.savez(p, **d)
        R, *_ = self._analyse()
        self.assertFalse(R['gate_A2']['C1'])

    def test_k_mismatch_cycle_fails_C2(self):
        p = os.path.join(self.run, T3.NPZ_NAME)
        d = dict(np.load(p))
        d['af.K'] = d['af.K'].copy(); d['af.K'][len(d['af.K']) // 2, 0] += 0.2
        np.savez(p, **d)
        R, *_ = self._analyse()
        self.assertFalse(R['gate_A2']['C2'])
        self.assertEqual(R['C2']['cycles_match'], R['C2']['cycles'] - 1)

    def test_settling_2sigma_unit(self):
        t = np.linspace(0, 10, 10001)
        y = np.where(t < 2, 1.0, np.exp(-(t - 2) / 0.5)) + 0.0
        y = y + 0.01 * np.sin(40 * t)
        s = A.settling_2sigma(t, y, 2.0, 10.0, (5.0, 10.0))
        # band = 2 * std(0.01 sin) = 0.0141; exp(-x/0.5) < band - 0.01 => ~2.6 s, > x where exp = band+0.01
        self.assertAlmostEqual(s['band_m'], 2 * 0.01 / np.sqrt(2), delta=1e-3)
        self.assertTrue(1.5 < s['t_settle_s'] < 3.0, s)
        self.assertFalse(s['censored'])
        self.assertIsNone(A.settling_2sigma(t, y, None, 10.0, (5.0, 10.0))['t_settle_s'])
        # still decaying at the steady start -> censored lower bound
        s2 = A.settling_2sigma(t, y, 2.0, 10.0, (3.0, 10.0))
        self.assertTrue(s2['censored'])
        self.assertAlmostEqual(s2['t_settle_s'], 1.0)

    def test_settling_2sigma_noise_only_not_censored(self):
        rng = np.random.default_rng(1)
        t = np.linspace(0, 20, 20001)
        y = rng.normal(0, 1e-3, t.size)
        s = A.settling_2sigma(t, A.A2.moving_average(t, y, 0.05), 0.0, 20.0, (10.0, 20.0))
        self.assertLess(s['out_frac_steady'], 0.15)
        self.assertIsNotNone(s['t_settle_s'])

    def test_main_with_t02_comparison(self):
        synth(self.run, T3.NPZ_NAME)
        out = os.path.join(self.tmp, 'out')
        with mock.patch.multiple(A, **PATCH_A), mock.patch.multiple(A2_PATCH_TARGET, **A2_PATCH):
            rc = A.main(self.run, self.t02, out)
        self.assertEqual(rc, 0)
        with open(os.path.join(out, 'metrics.json')) as f:
            M = json.load(f)
        self.assertTrue(M['gate_A2']['passed'])
        self.assertEqual([s['k_to'] for s in M['t02']['steps']], [12.0, 15.0, 20.0, 15.0, 10.0])
        self.assertEqual(len(M['t02']['per_K']), 6)
        self.assertEqual(M['criteria']['sha256'], self.info['sha256'])
        for f in ('summary.md', 'k_ramp.png'):
            self.assertTrue(os.path.exists(os.path.join(out, f)))
        with open(os.path.join(out, 'summary.md')) as f:
            self.assertIn('Gate A2: PASSED', f.read())


from noc.analysis import a02_live_k as A2_PATCH_TARGET   # noqa: E402
A2_PATCH = dict(MA_S=0.005)

if __name__ == '__main__':
    unittest.main()
