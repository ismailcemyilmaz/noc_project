"""
06a setup with the uavatt rotor_measure fix (F1 in noc/results/00b_diagnosis.md).

src/06a-physical-interaction/model_hexa_fa.py is NOT modified: it is loaded as a
module, its setup() is called unchanged, then one extra connection is made:

    uavatt.rotor_measure (in)  <-  rotorcraft/rotor_measure (out)

Before connecting, the port name is verified twice; on failure an exception is
raised and nothing is connected:
  1. documentation: noc/docs/genom/uavatt/README.adoc declares
     '=== rotor_measure (in)' of type '::or_rotorcraft::output'
     (skipped with a recorded note if the doc file is absent);
  2. live interface: the rotorcraft handle exposes 'rotor_measure' and reading it
     returns {'rotor_measure': {'rotor': [...]}} (or_rotorcraft::output).
"""

import importlib.util
import os
import re

from noc.common import telekyb_io as io

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
MODEL_06A = os.path.join(REPO, 'src', '06a-physical-interaction', 'model_hexa_fa.py')
UAVATT_DOC = os.path.join(REPO, 'noc', 'docs', 'genom', 'uavatt', 'README.adoc')

LOCAL_PORT = 'rotor_measure'
REMOTE_PORT = 'rotorcraft/rotor_measure'
PORT_TYPE = '::or_rotorcraft::output'


class PortCheckError(RuntimeError):
    pass


def load_06a(log_dir):
    """Import 06a (connects genomix, loads components) and redirect its LOG_DIR."""
    spec = importlib.util.spec_from_file_location('model_hexa_fa', MODEL_06A)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.LOG_DIR = log_dir  # 06a stop() copies component logs here
    return mod


def check_doc(path=UAVATT_DOC):
    if not os.path.exists(path):
        return {'doc': path, 'checked': False, 'note': 'doc file not present'}
    with open(path) as f:
        txt = f.read()
    m = re.search(r'=== rotor_measure \((in|out)\)(.*?)\n\'\'\'', txt, re.S)
    if not m:
        raise PortCheckError(f'{path}: no rotor_measure port section')
    direction, body = m.group(1), m.group(2)
    if direction != 'in' or PORT_TYPE not in body:
        raise PortCheckError(f'{path}: rotor_measure is ({direction}), type ok={PORT_TYPE in body}')
    return {'doc': path, 'checked': True, 'direction': direction, 'type': PORT_TYPE}


def check_live(rotorcraft):
    if not hasattr(rotorcraft, 'rotor_measure'):
        raise PortCheckError('rotorcraft handle has no rotor_measure port')
    out, valid = io.read_rotor_measure(rotorcraft)  # raises PortFormatError on wrong structure
    return {'live_read': True, 'n_rotors': out['n_rotors'], 'velocity_valid': valid['velocity'],
            'ts_valid': valid['ts']}


def setup(m):
    """06a setup() + verified uavatt rotor_measure connection. Returns a report dict."""
    m.setup()
    rep = {'local': LOCAL_PORT, 'remote': REMOTE_PORT}
    rep['doc_check'] = check_doc()
    rep['live_check'] = check_live(m.rotorcraft)
    rep['connect_reply'] = m.uavatt.connect_port({'local': LOCAL_PORT, 'remote': REMOTE_PORT})
    print(f'[setup_06a] uavatt.{LOCAL_PORT} <- {REMOTE_PORT} connected; {rep}', flush=True)
    return rep
