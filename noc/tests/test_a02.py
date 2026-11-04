"""a02_live_k end to end on a t02 dry run plus synthetic component logs that follow
the series-stiffness model (F = d0 / (1/Kpxy + 1/K_x)) with first-order transients."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.analysis import a02_live_k as A
from noc.tests import t02_live_k as T
from noc.tests.fake_genomix import make_module, FakeWorld

HOLD = 0.2
SLOW = dict(DT=0.002, T_WO_ZERO=0.02, TF=1.5, T_SETTLED=0.1, T_REPEAT=0.1 + 6 * HOLD, N_REPEAT=20,
            WAYPOINTS=[(0.0, (0.0, 0.0, 1.0, 0, 5.0)), (0.01, (1.2, 0.0, 1.0, 0, 4.0)),
                       (0.03, (1.55, 0.0, 1.0, 0, 0.05)), (1.4, (1.0, 0.0, 1.0, 0, 4.0)),
                       (1.45, (0.0, 0.0, 0.0, 0, 5.0))],
            K_SCHEDULE=[(0.1 + i * HOLD, k) for i, k in enumerate([10.0, 12.0, 15.0, 20.0, 15.0, 10.0])],
            SNAPSHOT_TIMES={})
D0, TAU, NOISE = 0.184, 0.02, 0.0005


def write_log(path, header, cols):
    with open(path, 'w') as f:
        f.write(' '.join(header) + '\n')
        for row in np.column_stack(cols):
            f.write(' '.join(f'{v:.9f}' for v in row) + '\n')


def synth_logs(log_dir, f_scale=1.0):
    d = np.load(os.path.join(log_dir, T.NPZ_NAME))
    t_npz, k_npz = d['t_wall'], d['k_target_x']
    ts = np.arange(t_npz[0], t_npz[-1], 0.0005)
    k = k_npz[np.clip(np.searchsorted(t_npz, ts, side='right') - 1, 0, None)]
    # first-order response of the steady deviation to the K_x schedule
    F_ss = D0 / (1 / 5.0 + 1 / k)
    a = 1 - np.exp(-np.diff(ts) / TAU)

    def lag(y):   # continuous first-order response (uavpos fx and xd are continuous)
        out = np.empty_like(y); out[0] = y[0]
        for i in range(1, y.size):
            out[i] = out[i - 1] + a[i - 1] * (y[i] - out[i - 1])
        return out
    rng = np.random.default_rng(0)
    dev = lag(-F_ss / k) + rng.normal(0, NOISE, ts.size)
    F = lag(F_ss) * f_scale + rng.normal(0, 10 * NOISE, ts.size)
    x_nom = np.full_like(ts, 1.55)
    xb = x_nom - D0
    z = np.zeros_like(ts)
    write_log(os.path.join(log_dir, 'uavpos.log'), ['ts', 'fx', 'xd', 'e_x', 'e_vx'], [ts, F, x_nom + dev, z, z])
    write_log(os.path.join(log_dir, 'maneuver.log'), ['ts', 'x', 'vx', 'vy', 'vz'], [ts, x_nom, z, z, z])
    write_log(os.path.join(log_dir, 'pom.log'), ['ts', 'x'], [ts, xb])
    write_log(os.path.join(log_dir, 'phynt.log'), ['ts', 'efx'], [ts, -F])


class TestA02(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.run_dir = os.path.join(self.tmp, 'run')
        mod = make_module(FakeWorld())
        with mock.patch.multiple(T, OUT_DIR=self.run_dir, CHECK_DIR=os.path.join(self.tmp, 'c'), **SLOW):
            self.assertEqual(T.run_flight(load=lambda d: mod), 0)

    def _analyse(self):
        with mock.patch.multiple(A, MA_S=0.005, SETTLE_ABS_M=0.0005, CALL_WIN_S=0.02):
            return A.analyse(self.run_dir)

    def test_model_run_passes_gate(self):
        synth_logs(self.run_dir)
        R, d, L, wins = self._analyse()
        self.assertEqual([r['K_x'] for r in R['per_K']], [10.0, 12.0, 15.0, 20.0, 15.0, 10.0])
        for r in R['per_K']:
            self.assertAlmostEqual(r['F_N']['mean'], r['pred_d0_given']['F_N'], delta=0.01)
            self.assertLess(abs(r['F_err_rel_d0_given']), 0.02)
            self.assertAlmostEqual(r['k_eff'], r['pred_d0_given']['k_series'], delta=0.1)
        self.assertEqual(R['timing']['n_repeat'], 20)
        self.assertEqual(R['applied_K']['calls_verified'], 25)
        self.assertTrue(R['gate_A']['servo_uninterrupted'], R['servo'])
        self.assertTrue(R['gate_A']['applied_K_verified'])
        self.assertTrue(R['gate_A']['F_within_15pct'])
        # settling from the set call: a few TAU for the steps (first-order, 10 % band)
        st = [r['settling_s'] for r in R['per_K'][1:]]
        self.assertTrue(all(s is not None and s < 0.1 for s in st), st)
        # gate needs n_repeat == meta n_repeat (20 here) and p95 < 20 ms (fake calls are µs)
        self.assertTrue(R['gate_A']['p95_lt_20ms'])
        self.assertTrue(R['gate_A']['passed'])
        out = os.path.join(self.tmp, 'out')
        with mock.patch.multiple(A, MA_S=0.005, SETTLE_ABS_M=0.0005, CALL_WIN_S=0.02):
            A.main(self.run_dir, out)
        with open(os.path.join(out, 'metrics.json')) as f:
            self.assertIn('gate_A', json.load(f))
        self.assertTrue(os.path.exists(os.path.join(out, 'live_k.png')))

    def test_force_off_by_30pct_fails_gate(self):
        synth_logs(self.run_dir, f_scale=1.3)
        R, *_ = self._analyse()
        self.assertFalse(R['gate_A']['F_within_15pct'])
        self.assertFalse(R['gate_A']['passed'])

    def test_servo_gap_detected(self):
        synth_logs(self.run_dir)
        p = os.path.join(self.run_dir, 'uavpos.log')
        with open(p) as f:
            lines = f.readlines()
        d = np.load(os.path.join(self.run_dir, T.NPZ_NAME))
        calls = json.loads(str(d['k_calls_json']))
        t_call = calls[0]['t_wall']
        keep = [lines[0]] + [l for l in lines[1:] if not (t_call - 0.0 <= float(l.split()[0]) <= t_call + 0.15)]
        with open(p, 'w') as f:
            f.writelines(keep)
        R, *_ = self._analyse()
        self.assertFalse(R['gate_A']['servo_uninterrupted'])
        self.assertTrue(R['servo']['call_windows'][0]['interrupted'])

    def test_prediction_values(self):
        self.assertEqual([round(A.predict(k)['F_N'], 2) for k in (10, 12, 15, 20)], [0.61, 0.65, 0.69, 0.74])


if __name__ == '__main__':
    unittest.main()
