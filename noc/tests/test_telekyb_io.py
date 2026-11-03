import math
import unittest

import numpy as np

from noc.common import telekyb_io as io

# structures copied from logs/t00_probe.txt
MAN_BEFORE_GOTO = {'desired': {'ts': {'sec': 0, 'nsec': 0}, 'intrinsic': 0, 'pos': None, 'att': None,
                               'vel': None, 'avel': None, 'acc': None, 'aacc': None, 'jerk': None,
                               'snap': None}}
PHYNT_DES = {'desired': {'ts': {'sec': 1793703637, 'nsec': 422402000}, 'intrinsic': 0,
                         'pos': {'x': -5.6e-07, 'y': 1.42e-05, 'z': 0.2198},
                         'att': {'qw': 1.0, 'qx': 0, 'qy': 0, 'qz': 4.8e-05},
                         'vel': {'vx': 0, 'vy': 0, 'vz': 0}, 'avel': {'wx': 0, 'wy': 0, 'wz': 0},
                         'acc': {'ax': 0, 'ay': 0, 'az': 0}, 'aacc': None, 'jerk': None, 'snap': None}}


class Fake:
    def __init__(self, **replies):
        for k, v in replies.items():
            setattr(self, k, (lambda v=v: v))


class TestTelekybIO(unittest.TestCase):
    def test_none_fields_marked(self):
        out, valid = io.read_maneuver_desired(Fake(desired=MAN_BEFORE_GOTO))
        self.assertFalse(valid['pos'])
        self.assertFalse(valid['ts'])
        self.assertTrue(np.all(np.isnan(out['pos'])))
        self.assertTrue(math.isnan(out['ts']))

    def test_valid_fields(self):
        out, valid = io.read_phynt_desired(Fake(desired=PHYNT_DES))
        self.assertTrue(valid['pos'] and valid['ts'])
        self.assertAlmostEqual(out['pos'][2], 0.2198)
        self.assertAlmostEqual(out['ts'], 1793703637.422402, places=5)

    def test_wrong_key_raises(self):
        with self.assertRaises(io.PortFormatError):
            io.read_external_wrench(Fake(external_wrench={'wrench': {}}))

    def test_wrench_none(self):
        rep = {'external_wrench': {'ts': {'sec': 1, 'nsec': 0}, 'intrinsic': 0, 'force': None,
                                   'force_cov': None, 'torque': {'x': 0.1, 'y': 0, 'z': 0},
                                   'torque_cov': None}}
        out, valid = io.read_external_wrench(Fake(external_wrench=rep))
        self.assertFalse(valid['force'])
        self.assertTrue(valid['torque'])

    def test_recorder_aligned(self):
        r = io.Recorder()
        r.add('m', io.read_maneuver_desired(Fake(desired=MAN_BEFORE_GOTO)))
        r.add_scalar('t', 0.0)
        a = r.to_arrays()
        self.assertEqual(a['m.pos'].shape, (1, 3))
        self.assertEqual(a['m.valid.pos'].tolist(), [False])


if __name__ == '__main__':
    unittest.main()
