"""af_on_baseline.json must match the (unchanged) 06a setup() it documents."""

import json
import math
import os
import re
import unittest

from noc.common import setup_06a

CFG = os.path.join(setup_06a.REPO, 'noc', 'config', 'af_on_baseline.json')


def _num(src, key):
    m = re.search(r"'%s':\s*([-0-9.]+)" % re.escape(key), src)
    return float(m.group(1))


def _list(src, name):
    m = re.search(r'%s\s*=\s*\[([^\]]*)\]' % re.escape(name), src)
    return [float(x) for x in m.group(1).split(',')]


class TestAfOnConfig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(CFG) as f:
            cls.cfg = json.load(f)
        with open(setup_06a.MODEL_06A) as f:
            cls.src = f.read()

    def test_uavpos_uavatt_gains(self):
        for comp, keys in (('uavpos', ('Kpxy', 'Kpz', 'Kvxy', 'Kvz', 'Kixy', 'Kiz')),
                           ('uavatt', ('Kqxy', 'Kqz', 'Kwxy', 'Kwz', 'wmin', 'wmax'))):
            for k in keys:
                self.assertEqual(self.cfg[comp][k], _num(self.src, k), f'{comp}.{k}')

    def test_af_wo(self):
        self.assertEqual(self.cfg['phynt_af']['K'], _list(self.src, 'af_K'))
        self.assertEqual(self.cfg['phynt_af']['mass'], float(re.search(r'af_mass\s*=\s*([0-9.]+)', self.src).group(1)))
        wo = re.search(r'set_wo_gains\(K=\[([^\]]*)\]', self.src).group(1)
        self.assertEqual(self.cfg['phynt_wo']['K'], [float(x) for x in wo.split(',')])

    def test_contact_point(self):
        x_wall = float(re.search(r'X_WALL\s*=\s*([0-9.]+)', self.src).group(1))
        l_bar = float(re.search(r'L_BAR\s*=\s*([0-9.]+)', self.src).group(1))
        self.assertTrue(math.isclose(self.cfg['scenario']['BODY_X_CONTACT'], x_wall - l_bar + 0.15))

    def test_setup_connection(self):
        c = self.cfg['setup']['extra_connection']
        self.assertEqual((c['local'], c['remote']), (setup_06a.LOCAL_PORT, setup_06a.REMOTE_PORT))


if __name__ == '__main__':
    unittest.main()
