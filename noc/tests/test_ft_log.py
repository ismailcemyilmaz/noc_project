"""F/T recording chain: ft_stamp.py, ft_log parser / tail, t04 FtLogger with a real
subprocess (process group, SIGTERM), log_ft.sh without gz."""

import io as _io
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

from noc.common import ft_log
from noc.experiments import ft_stamp
from noc.tests.fake_ft import wrench_line
from noc.tests.t04_ft_check import FtLogger

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# observed with gz topic --json-output (gz-transport 13, host): int64 sec as string, zeros omitted
REAL = '{"header":{"stamp":{"sec":"12","nsec":5000000}},"force":{"x":-0.5,"z":0.098},"torque":{"y":0.06}}'


class TestStampAndParse(unittest.TestCase):
    def test_stamp_single_and_multiline(self):
        inp = _io.StringIO(REAL + '\n' + 'warning: something\n' + '{\n "force": {\n  "x": 1.5\n }\n}\n')
        out, err = _io.StringIO(), _io.StringIO()
        clock = iter([100.0, 101.0]).__next__
        ft_stamp.main(inp, out, err, clock)
        lines = out.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertTrue(lines[0].startswith('100.000000\t{'))
        self.assertIn('ignored non-JSON line', err.getvalue())
        r = ft_log.parse_line(lines[1])
        self.assertEqual(r[0], 101.0)
        self.assertTrue(np.isnan(r[1]))           # no header -> NaN stamp
        self.assertEqual(r[2:5], (1.5, 0.0, 0.0))

    def test_parse_real_format(self):
        r = ft_log.parse_line('5.0\t' + REAL)
        self.assertEqual(r, (5.0, 12.005, -0.5, 0.0, 0.098, 0.0, 0.06, 0.0))

    def test_tail_partial_and_bad_lines(self):
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d)
        p = os.path.join(d, 'ft.log')
        t = ft_log.FtTail(p)
        self.assertEqual(t.poll(), 0)
        with open(p, 'w') as f:
            f.write('# c\n' + wrench_line(1.0, 0.5, (-0.1, 0, 0.2)) + wrench_line(1.1, 0.6, (0, 0, 0))[:30])
        self.assertEqual(t.poll(), 1)
        with open(p, 'a') as f:
            f.write(wrench_line(1.1, 0.6, (0, 0, 0))[30:] + 'garbage line\n')
        self.assertEqual(t.poll(), 1)
        self.assertEqual((t.n, t.n_bad, t.n_comment), (2, 1, 1))
        A, info = ft_log.read_ft(p)
        self.assertEqual((info['n'], info['n_bad']), (2, 1))
        np.testing.assert_allclose(A['stamp'], [0.5, 0.6])
        s = ft_log.window_stats(A, 0.9, 1.2)
        self.assertEqual(s['n'], 2)
        self.assertAlmostEqual(s['fx']['mean'], -0.05)


GEN = ("import time,json\n"
       "t0=time.time()\n"
       "while True:\n"
       "  t=time.time()-t0; print(json.dumps({'header':{'stamp':{'sec':str(int(t)),'nsec':int((t%1)*1e9)}},"
       "'force':{'x':-0.6}}), flush=True); time.sleep(0.002)\n")


class TestFtLoggerProcess(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, self.d)

    def test_real_subprocess_group_is_stopped(self):
        p = os.path.join(self.d, 'ft.log')
        cmd = ['sh', '-c', f'"{sys.executable}" -c "$GEN" | "{sys.executable}" -u '
                           f'"{os.path.join(REPO, "noc", "experiments", "ft_stamp.py")}" >> "{p}"']
        os.environ['GEN'] = GEN
        lg = FtLogger(p, cmd)
        lg.start()
        res = lg.wait_stream(50, 5.0)
        self.assertTrue(res['ok'], res)
        time.sleep(0.2)
        lg.stop()
        self.assertTrue(lg.info['alive_at_stop'])
        self.assertIsNotNone(lg.info['returncode'])
        n = lg.info['n_messages']
        time.sleep(0.3)
        A, info = ft_log.read_ft(p)
        self.assertEqual(info['n'], n)              # nothing written after stop: group is dead
        self.assertEqual(info['n_bad'], 0)
        self.assertTrue(np.all(np.diff(A['stamp']) > 0))
        self.assertTrue(np.all(A['fx'] == -0.6))

    def test_log_ft_sh_without_gz(self):
        p = os.path.join(self.d, 'ft.log')
        bin_ = os.path.join(self.d, 'bin')       # PATH with the tools log_ft.sh needs, but no gz
        os.makedirs(bin_)
        for tool in ('sh', 'dirname', 'date', 'stdbuf', 'python3'):
            w = shutil.which(tool)
            if w:
                os.symlink(w, os.path.join(bin_, tool))
        r = subprocess.run([shutil.which('sh'), os.path.join(REPO, 'noc', 'experiments', 'log_ft.sh'), p],
                           env={'PATH': bin_}, capture_output=True, text=True, timeout=10)
        self.assertEqual(r.returncode, 127)
        self.assertIn('gz not found', r.stderr)

    def test_stream_timeout_reported(self):
        p = os.path.join(self.d, 'ft.log')
        lg = FtLogger(p, ['sh', '-c', 'sleep 30'])
        lg.start()
        res = lg.wait_stream(50, 0.3)
        lg.stop()
        self.assertFalse(res['ok'])
        self.assertEqual(res['n'], 0)


if __name__ == '__main__':
    unittest.main()
