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


def _input_port(*a):
    raise TypeError("partial() missing 1 required positional argument: 'path'")


WRENCH = {'wrench_measure': {'ts': {'sec': 2, 'nsec': 0}, 'intrinsic': 0,
                             'force': {'x': 0.0, 'y': 0.0, 'z': 26.7}, 'force_cov': None,
                             'torque': {'x': 0, 'y': 0, 'z': 0}, 'torque_cov': None}}


class TestSourcesPreflightSnapshot(unittest.TestCase):
    def comps(self, with_get_reference=True):
        ph = Fake(desired=PHYNT_DES, reference=None, wrench_measure=None,
                  external_wrench={'external_wrench': WRENCH['wrench_measure']},
                  get_af_parameters={'af': {'K': [10] * 6}}, get_wo_parameters={'wo': {}},
                  get_body_parameters={'body': {}}, log_info={'miss': 0, 'total': 0})
        ph.reference = _input_port
        ph.wrench_measure = _input_port
        if with_get_reference:
            ph.get_reference = lambda: {'reference': PHYNT_DES['desired']}
        return {'pom': Fake(), 'phynt': ph, 'maneuver': Fake(desired=MAN_BEFORE_GOTO),
                'uavatt': Fake(wrench_measure=WRENCH)}

    def test_no_input_ports_and_sources(self):
        c = self.comps()
        c['pom'].frame = lambda port: {'frame': PHYNT_DES['desired']}
        src = io.build_sources(c['phynt'])
        self.assertEqual(io.describe_sources(src)['phynt_ref'], 'phynt.read_phynt_get_reference')
        self.assertEqual(io.describe_sources(src)['wrench_meas'], 'uavatt.read_uavatt_wrench_measure')
        rep = io.preflight(c, src)
        self.assertTrue(all(v['ok'] for v in rep.values()), rep)

    def test_reference_fallback(self):
        c = self.comps(with_get_reference=False)
        src = io.build_sources(c['phynt'])
        self.assertEqual(io.describe_sources(src)['phynt_ref'], 'maneuver.read_maneuver_desired')

    def test_preflight_records_failure(self):
        c = self.comps()  # pom has no frame -> AttributeError
        rep = io.preflight(c, io.build_sources(c['phynt']))
        self.assertFalse(rep['pom']['ok'])
        self.assertIn('AttributeError', rep['pom']['error'])
        self.assertTrue(rep['wrench_meas']['ok'])

    def test_snapshot_records_failure(self):
        ph = self.comps()['phynt']
        ph.log_info = _input_port
        s = io.phynt_snapshot(ph)
        self.assertIn('TypeError', s['log_info']['error'])
        self.assertEqual(s['get_af_parameters'], {'af': {'K': [10] * 6}})


if __name__ == '__main__':
    unittest.main()
