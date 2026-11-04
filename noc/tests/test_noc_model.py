"""Structure of gazebo/models/noc-tilthex and gazebo/worlds/noc-wall-world.world against the
originals (which must stay unchanged), plus sdformat validation when `gz sdf` is available."""

import os
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET

from noc.common import ft_log

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
ORIG_MODEL = os.path.join(REPO, 'gazebo', 'models', 'mrsim-tilthex', 'model.sdf')
NOC_MODEL = os.path.join(REPO, 'gazebo', 'models', 'noc-tilthex', 'model.sdf')
ORIG_WORLD = os.path.join(REPO, 'gazebo', 'worlds', 'hexa-fa-wall-world.world')
NOC_WORLD = os.path.join(REPO, 'gazebo', 'worlds', 'noc-wall-world.world')
EE_PARTS = ('ee-vbar', 'ee-hbar', 'ee-tip')


def canon(el):
    return ET.tostring(el, encoding='unicode').split()


class TestModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.o = ET.parse(ORIG_MODEL).getroot().find('model')
        cls.n = ET.parse(NOC_MODEL).getroot().find('model')

    def test_originals_unchanged_in_git(self):
        r = subprocess.run(['git', '-C', REPO, 'diff', '--quiet', 'HEAD', '--',
                            'gazebo/models/mrsim-tilthex', 'gazebo/worlds/hexa-fa-wall-world.world',
                            'src/06a-physical-interaction'])
        self.assertEqual(r.returncode, 0)

    def test_ee_parts_moved_identical(self):
        ob, nb, ne = self.o.find("link[@name='base']"), self.n.find("link[@name='base']"), self.n.find("link[@name='ee']")
        self.assertEqual(ne.find('pose').text.split(), ['0'] * 6)   # ee frame = model frame = base frame
        self.assertEqual(ob.find('pose'), None)
        for kind in ('visual', 'collision'):
            o_ee = [e for e in ob.findall(kind) if e.get('name').startswith('ee-')]
            n_ee = ne.findall(kind)
            self.assertEqual(len(o_ee), 3)
            self.assertEqual([e.get('name') for e in o_ee], [e.get('name') for e in n_ee])
            for a, b in zip(o_ee, n_ee):
                self.assertEqual(canon(a), canon(b), a.get('name'))
            self.assertFalse([e for e in nb.findall(kind) if e.get('name').startswith('ee-')])
            # every non-ee element of base unchanged
            self.assertEqual([canon(e) for e in ob.findall(kind) if not e.get('name').startswith('ee-')],
                             [canon(e) for e in nb.findall(kind)])
        self.assertEqual(canon(ob.find('inertial')), canon(nb.find('inertial')))

    def test_rotors_plugin_untouched(self):
        self.assertEqual([canon(e) for e in self.o.findall('include')], [canon(e) for e in self.n.findall('include')])
        oj = [canon(j) for j in self.o.findall('joint')]
        nj = [canon(j) for j in self.n.findall('joint') if j.get('name') != 'ee_joint']
        self.assertEqual(oj, nj)
        self.assertEqual(canon(self.o.find('plugin')), canon(self.n.find('plugin')))
        self.assertEqual(self.n.find('plugin').find('link').text, 'base')

    def test_ee_joint_and_sensor(self):
        mi = ft_log.model_info(NOC_MODEL)
        self.assertEqual(mi, dict(model='noc-tilthex', ee_mass_kg=0.01, ee_com_m=[0.3, 0.0, -0.125],
                                  joint='ee_joint', joint_type='fixed', parent='base', child='ee',
                                  sensor='ee_ft', topic='/noc/ee_ft', update_rate_hz=500.0,
                                  frame='child', measure_direction='child_to_parent'))
        s = self.n.find("joint[@name='ee_joint']/sensor")
        self.assertEqual(s.find('always_on').text, 'true')
        ine = self.n.find("link[@name='ee']/inertial/inertia")
        self.assertEqual([float(ine.find(k).text) for k in ('ixx', 'iyy', 'izz')], [1e-6, 3e-4, 3e-4])


class TestWorld(unittest.TestCase):
    def test_world_differs_only_in_model_and_ft_plugin(self):
        o = ET.parse(ORIG_WORLD).getroot().find('world')
        n = ET.parse(NOC_WORLD).getroot().find('world')
        inc = n.find('include')
        self.assertEqual(inc.find('uri').text, 'model://noc-tilthex')
        self.assertEqual(inc.find('name').text, 'hr6')
        self.assertEqual(canon(inc.find('pose')), canon(o.find('include/pose')))
        plugins = {p.get('filename'): p for p in n.findall('plugin')}
        self.assertIn('gz-sim-forcetorque-system', plugins)
        opt = plugins['optitrack-gazebo.so'].find('publish')
        self.assertEqual((opt.find('link').text, opt.find('name').text), ('hr6::base', 'HR_6'))
        for tag in ('physics', 'light'):
            self.assertEqual(canon(o.find(tag)), canon(n.find(tag)))
        self.assertEqual([canon(m) for m in o.findall('model')], [canon(m) for m in n.findall('model')])
        self.assertEqual([canon(p) for p in o.findall('plugin')],
                         [canon(p) for p in n.findall('plugin') if p.get('filename') != 'gz-sim-forcetorque-system'])

    def test_sim_noc_is_copy_with_new_world(self):
        a = open(os.path.join(REPO, 'src', '06a-physical-interaction', 'simulation.sh')).read().splitlines()
        b = open(os.path.join(REPO, 'noc', 'experiments', 'sim_noc.sh')).read().splitlines()
        strip = lambda ls: [l for l in ls if not l.startswith('#') and not l.startswith('gz_world=')]
        self.assertEqual(strip(a), strip(b))
        self.assertIn('gz_world=/shared-workspace/gazebo/worlds/noc-wall-world.world', b)
        self.assertIn('export GZ_SIM_RESOURCE_PATH=/shared-workspace/gazebo/models:$GZ_SIM_RESOURCE_PATH', b)

    @unittest.skipUnless(shutil.which('gz') and os.path.isdir(os.path.expanduser('~/Downloads/mrsim-rotor')),
                         'gz sdf or an mrsim-rotor copy not available')
    def test_gz_sdf_valid(self):
        env = dict(os.environ)
        p = os.path.join(REPO, 'gazebo', 'models') + ':' + os.path.expanduser('~/Downloads')
        env.update(GZ_SIM_RESOURCE_PATH=p, SDF_PATH=p)
        for f in (NOC_MODEL, NOC_WORLD):
            r = subprocess.run(['gz', 'sdf', '-k', f], capture_output=True, text=True, env=env, timeout=60)
            self.assertEqual((r.returncode, r.stdout.strip()), (0, 'Valid.'), r.stderr)


if __name__ == '__main__':
    unittest.main()
