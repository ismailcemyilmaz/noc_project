"""Host dry runs of t04_ft_check with fake genomix components and an in-process fake F/T
logger (compressed timeline)."""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from noc.tests import t04_ft_check as T
from noc.tests.fake_genomix import make_module, FakeWorld
from noc.tests.fake_ft import FakeFtLogger
from noc.common.flight import SAFETY

FAST = dict(DT=0.002, T_WO_ZERO=0.05, TF=1.0,
            WAYPOINTS=[(0.01, (0.0, 0.0, 1.0, 0, 5.0)), (0.03, (1.2, 0.0, 1.0, 0, 4.0)),
                       (0.30, (1.55, 0.0, 1.0, 0, 2.0)), (0.60, (1.2, 0.0, 1.0, 0, 3.0)),
                       (0.90, (0.0, 0.0, 0.0, 0, 5.0))],
            WINDOWS={'FREE1': (0.10, 0.30), 'CONTACT': (0.40, 0.60), 'FREE2': (0.70, 0.90)},
            SNAPSHOT_TIMES={'free1': 0.2}, FT_WAIT_S=1.0, CHECK_GROUND_S=0.2)


def world():
    w = FakeWorld()
    w.des_valid_after_goto = True
    w.ext_empty_until_goto = True
    return w


class TestT04(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)

    def _run(self, w=None, check=False, logger_kw=None, **extra):
        w = w or world()
        mod = make_module(w)
        mk = lambda path: FakeFtLogger(path, world=w, **(logger_kw or {}))
        with mock.patch.multiple(T, OUT_DIR=os.path.join(self.tmp, 'run'), CHECK_DIR=os.path.join(self.tmp, 'chk'),
                                 **{**FAST, **extra}), \
                mock.patch('noc.tests.t03_k_ramp.PENDING_TIMEOUT_S', 0.2):
            rc = (T.run_check if check else T.run_flight)(load=lambda d: mod, make_logger=mk)
        return rc, mod

    def _load(self):
        d = np.load(os.path.join(self.tmp, 'run', T.NPZ_NAME))
        meta = json.loads(str(d['meta_json']))
        ev = [e['name'] for e in json.loads(str(d['events_json']))]
        return d, meta, ev

    def test_flight_normal(self):
        rc, mod = self._run()
        d, meta, ev = self._load()
        self.assertEqual(rc, 0, meta['abort'])
        self.assertTrue(meta['ft_stream']['ok'])
        self.assertTrue(meta['ft_logger']['alive_at_stop'])
        self.assertEqual(meta['ft_logger']['n_bad'], 0)
        live = meta['ft_live']
        self.assertEqual(sorted(live), ['CONTACT', 'FREE1', 'FREE2'])
        self.assertAlmostEqual(live['FREE1']['fz']['mean'], -0.098, delta=0.002)
        self.assertLess(live['CONTACT']['fx']['mean'] - live['FREE1']['fx']['mean'], -0.5)
        self.assertAlmostEqual(live['FREE2']['fx']['mean'], live['FREE1']['fx']['mean'], delta=0.01)
        for n in ('FREE1', 'CONTACT', 'FREE2'):
            self.assertIn(f'window {n} start', ev)
            self.assertIn(f'window {n} end', ev)
        self.assertEqual(meta['n_af_set_calls'], 0)
        self.assertEqual(mod.world.af_calls, [])                       # AF never changed
        self.assertTrue(np.all(d['af_match'] == 1.0))
        self.assertEqual(meta['safety'], SAFETY)
        self.assertEqual(meta['safety_version'], 2)
        self.assertEqual(meta['read_error_counts'], {})
        self.assertGreater(meta['read_error_counts_pending'].get('ext_wrench', 0), 0)
        self.assertGreater(d['ft.n'][-1], 100)
        self.assertEqual(meta['model']['topic'], '/noc/ee_ft')
        self.assertEqual(ev[-1], 'stop')
        self.assertTrue(all(g[5] for g in mod.world.gotos))           # ack=True

    def test_no_ft_stream_refuses_takeoff(self):
        with self.assertRaises(RuntimeError) as cm:
            self._run(logger_kw={'silent': True})
        self.assertIn('F/T stream', str(cm.exception))
        d, meta, ev = self._load() if os.path.exists(os.path.join(self.tmp, 'run', T.NPZ_NAME)) else (None, None, [])
        self.assertIn('F/T stream', meta['error'])
        self.assertFalse(meta['ft_stream']['ok'])

    def test_ft_dies_in_flight_is_recorded_not_aborted(self):
        rc, mod = self._run(logger_kw={'die_after': 0.35})
        d, meta, ev = self._load()
        self.assertEqual(rc, 0)
        self.assertFalse(meta['ft_logger']['alive_at_stop'])           # C1 will fail in a04

    def test_pending_never_ready_aborts(self):
        w = world(); w.des_valid_after_goto = False
        rc, mod = self._run(w)
        d, meta, ev = self._load()
        self.assertEqual(rc, 1)
        self.assertIn('pending sources not ready', meta['abort']['reasons'][-1])
        self.assertIn('SAFETY land goto', ev)

    def test_initial_af_mismatch_refuses_takeoff(self):
        w = world(); w.af['K'][0] = 12.0
        with self.assertRaises(Exception):
            self._run(w)
        self.assertEqual(w.gotos, [])

    def test_check_mode(self):
        rc, mod = self._run(check=True)
        self.assertEqual(rc, 0)
        with open(os.path.join(self.tmp, 'chk', 'check.json')) as f:
            res = json.load(f)
        self.assertTrue(res['ft_stream']['ok'])
        self.assertTrue(res['ft_ground_weight_check']['within_20pct'])
        self.assertEqual(res['preflight']['ext_wrench']['status'], 'pending')
        self.assertEqual(mod.world.gotos, [])

    def test_check_mode_fails_without_stream(self):
        rc, mod = self._run(check=True, logger_kw={'silent': True})
        self.assertEqual(rc, 1)


if __name__ == '__main__':
    unittest.main()
