"""a04_ft_check end to end: a t04 dry run (fake genomix + fake F/T logger) whose component
logs and ft.log are then replaced by synthetic signals built from the run's own events,
with known bias, noise and lags. Gate thresholds come from the committed criteria file."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.analysis import a04_ft_check as A
from noc.tests import t04_ft_check as T
from noc.tests import test_a02
from noc.tests.fake_genomix import make_module, FakeWorld
from noc.tests.fake_ft import FakeFtLogger, wrench_line

SLOW = dict(DT=0.01, T_WO_ZERO=0.5, TF=5.6,
            WAYPOINTS=[(0.05, (0.0, 0.0, 1.0, 0, 5.0)), (0.1, (1.2, 0.0, 1.0, 0, 4.0)),
                       (2.2, (1.55, 0.0, 1.0, 0, 2.0)), (3.8, (1.2, 0.0, 1.0, 0, 3.0)),
                       (5.4, (0.0, 0.0, 0.0, 0, 5.0))],
            WINDOWS={'FREE1': (1.0, 2.2), 'CONTACT': (2.6, 3.8), 'FREE2': (4.2, 5.4)},
            SNAPSHOT_TIMES={}, FT_WAIT_S=1.0)
F_C, LAG_WO, LAG_FX, BIAS_X = -0.6, 0.05, 0.08, 0.01


def goto_wall(run, x):
    with open(os.path.join(run, 'events.json')) as f:
        ev = json.load(f)
    return [e['wall_start'] for e in ev if e['name'].startswith('maneuver.goto') and e['args']['args'][0] == x]


def synth(run, ft_noise=0.003, free2_offset=0.0, gap=None, bad_line=False, free1_x=1.2, seed=0):
    rng = np.random.default_rng(seed)
    d = np.load(os.path.join(run, T.NPZ_NAME))
    t0, t1 = d['t_wall'][0] - 0.5, d['t_wall'][-1] + 0.5
    c_on = goto_wall(run, 1.55)[0] + 0.1
    c_off = goto_wall(run, 1.2)[-1] + 0.1
    with open(os.path.join(run, 'events.json')) as f:
        ev = json.load(f)
    f2a = [e['wall_start'] for e in ev if e['name'] == 'window FREE2 start'][0]

    def Ftrue(t):
        return np.where((t >= c_on) & (t < c_off), F_C, 0.0)
    # F/T at 1 kHz: header stamps = sim time (offset 50 s), host time = wall + 3 ms + buffering jitter
    tw = np.arange(t0, t1, 0.001)
    if gap is not None:
        tw = tw[(tw < gap[0]) | (tw > gap[1])]
    fx = Ftrue(tw) + BIAS_X + rng.normal(0, ft_noise, tw.size) + np.where(tw >= f2a, free2_offset, 0.0)
    host = np.maximum.accumulate(tw + 0.003 + np.abs(rng.normal(0, 0.002, tw.size)))   # arrival order
    with open(os.path.join(run, 'ft.log'), 'w') as f:
        f.write('# synthetic\n')
        for i in range(tw.size):
            f.write(wrench_line(host[i], tw[i] - t0 + 50.0, (fx[i], 0.0, -0.098)))
            if bad_line and i == tw.size // 2:
                f.write('not a wrench line\n')
    # component logs at 1 kHz
    ts = np.arange(t0, t1, 0.001)
    z = np.zeros_like(ts)
    x = np.where(ts < goto_wall(run, 1.55)[0], free1_x, np.where(ts < goto_wall(run, 1.2)[-1], 1.37, 1.2))
    test_a02.write_log(os.path.join(run, 'pom.log'), ['ts', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'], [ts, x, z, z + 1, z, z, z])
    test_a02.write_log(os.path.join(run, 'phynt.log'), ['ts', 'efx'], [ts, Ftrue(ts - LAG_WO) + rng.normal(0, 0.01, ts.size)])
    test_a02.write_log(os.path.join(run, 'uavpos.log'), ['ts', 'fx', 'xd'], [ts, -Ftrue(ts - LAG_FX) + rng.normal(0, 0.01, ts.size), x])
    test_a02.write_log(os.path.join(run, 'maneuver.log'), ['ts', 'x'], [ts, x])


class TestA04(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.base = os.path.join(cls.tmp, 'base')
        w = FakeWorld(); w.des_valid_after_goto = True; w.ext_empty_until_goto = True
        mod = make_module(w)
        with mock.patch.multiple(T, OUT_DIR=cls.base, CHECK_DIR=os.path.join(cls.tmp, 'c'), **SLOW):
            assert T.run_flight(load=lambda d: mod, make_logger=lambda p: FakeFtLogger(p, world=w)) == 0
        cls.crit, cls.info = A.load_criteria()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def setUp(self):
        self.run = os.path.join(self.tmp, f'run_{self._testMethodName}')
        shutil.copytree(self.base, self.run)

    def _analyse(self, **kw):
        synth(self.run, **kw)
        return A.analyse(self.run, self.crit)[0]

    def test_criteria_committed_and_used(self):
        self.assertEqual(self.crit['criteria']['C4_contact_force_above_noise']['factor'], 5.0)
        self.assertEqual(self.crit['criteria']['C2_timestamps_continuous_and_fast']['min_rate_hz'], 100.0)
        self.assertTrue(self.info['committed'])

    def test_model_run_passes_with_known_lags_and_bias(self):
        R = self._analyse()
        for k in ('C1', 'C2', 'C3', 'C4', 'C5', 'C6'):
            self.assertTrue(R['gate_B1'][k], (k, R[k]))
        self.assertTrue(R['gate_B1']['passed'])
        self.assertAlmostEqual(R['C3']['bias'], BIAS_X, delta=0.002)
        self.assertAlmostEqual(R['C4']['contact_mean_corrected'], F_C, delta=0.02)
        self.assertTrue(R['C4']['sign_as_expected'])
        self.assertAlmostEqual(R['C5']['ratio'], 1.0, delta=0.05)
        est = R['comparison']['estimates']
        self.assertAlmostEqual(est['WO efx']['lag_s'], LAG_WO, delta=0.011)
        self.assertAlmostEqual(est['-uavpos fx']['lag_s'], LAG_FX, delta=0.011)
        self.assertLess(est['WO efx']['rmse_contact_N'], 0.03)
        self.assertLess(est['WO efx']['rmse_span_N'], est['WO efx']['rmse_span_no_lag_N'])
        self.assertAlmostEqual(R['C2']['rtf'], 1.0, delta=0.01)
        self.assertGreater(R['C2']['rate_stamp_hz'], 900)
        self.assertLess(R['C2']['host_minus_env_p99_ms'], 10)    # envelope removes the fixed 3 ms + jitter floor

    def test_noise_fails_C4(self):
        R = self._analyse(ft_noise=0.2)
        self.assertFalse(R['gate_B1']['C4'])

    def test_bias_drift_fails_C5(self):
        R = self._analyse(free2_offset=-0.5)
        self.assertFalse(R['gate_B1']['C5'])
        self.assertTrue(R['gate_B1']['C4'])

    def test_stamp_gap_fails_C2(self):
        c_on = goto_wall(self.run, 1.55)[0]
        R = self._analyse(gap=(c_on + 0.5, c_on + 0.6))
        self.assertFalse(R['gate_B1']['C2'])
        self.assertGreater(R['C2']['max_gap'], 0.05)

    def test_bad_line_fails_C1(self):
        R = self._analyse(bad_line=True)
        self.assertFalse(R['gate_B1']['C1'])
        self.assertEqual(R['C1']['ft_bad_lines'], 1)

    def test_tip_near_wall_fails_C3(self):
        R = self._analyse(free1_x=1.4)
        self.assertFalse(R['gate_B1']['C3'])

    def test_af_mismatch_fails_C6(self):
        p = os.path.join(self.run, T.NPZ_NAME)
        d = dict(np.load(p)); d['af_match'] = d['af_match'].copy(); d['af_match'][5] = 0.0
        np.savez(p, **d)
        R = self._analyse()
        self.assertFalse(R['gate_B1']['C6'])

    def test_rolling_min(self):
        t = np.arange(10.0)
        y = np.array([5, 3, 4, 9, 1, 7, 8, 2, 6, 0], float)
        np.testing.assert_array_equal(A.rolling_min(t, y, 1.0), [3, 3, 3, 1, 1, 1, 2, 2, 0, 0])

    def test_main_outputs(self):
        synth(self.run)
        out = os.path.join(self.tmp, 'out')
        self.assertEqual(A.main(self.run, out), 0)
        with open(os.path.join(out, 'metrics.json')) as f:
            M = json.load(f)
        self.assertTrue(M['gate_B1']['passed'])
        self.assertEqual(M['criteria']['sha256'], self.info['sha256'])
        with open(os.path.join(out, 'summary.md')) as f:
            s = f.read()
        self.assertIn('Gate B1: PASSED', s)
        self.assertIn('/noc/ee_ft', s)
        self.assertTrue(os.path.exists(os.path.join(out, 'ft_check.png')))


if __name__ == '__main__':
    unittest.main()
