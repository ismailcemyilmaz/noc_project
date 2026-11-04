"""
Parser for F/T logs written by noc/experiments/log_ft.sh:

    <host receive time> TAB <gz.msgs.Wrench JSON>

The JSON is protobuf JSON as printed by `gz topic --json-output`: default-valued fields
are omitted (a zero force component is absent) and int64 fields such as header.stamp.sec
may be strings. Absent numbers are read as 0; an absent header stamp gives NaN.
Lines starting with '#' are comments.
"""

import json
import math
import os

import numpy as np

COLS = ('host', 'stamp', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz')


def _num(d, k):
    v = (d or {}).get(k, 0)
    return float(v) if v is not None else 0.0


def parse_message(msg):
    """dict (Wrench JSON) -> (stamp, force[3], torque[3])."""
    st = (msg.get('header') or {}).get('stamp')
    stamp = math.nan if st is None else _num(st, 'sec') + 1e-9 * _num(st, 'nsec')
    f, t = msg.get('force') or {}, msg.get('torque') or {}
    return stamp, [_num(f, a) for a in 'xyz'], [_num(t, a) for a in 'xyz']


def parse_line(line):
    """'host TAB json' -> tuple of len(COLS). Raises ValueError on a malformed line."""
    host, _, js = line.rstrip('\n').partition('\t')
    if not js:
        raise ValueError('no TAB separator')
    stamp, f, t = parse_message(json.loads(js))
    return (float(host), stamp, *f, *t)


class FtTail:
    """Incremental reader (live use in t04 and offline). ``poll()`` reads complete new
    lines; an incomplete last line is kept for the next call."""

    def __init__(self, path):
        self.path = path
        self.pos = 0
        self.partial = ''
        self.rows = []
        self.n_bad = 0
        self.bad_examples = []
        self.n_comment = 0

    def poll(self):
        if not os.path.exists(self.path):
            return 0
        with open(self.path) as f:
            f.seek(self.pos)
            chunk = f.read()
            self.pos = f.tell()
        data = self.partial + chunk
        lines = data.split('\n')
        self.partial = lines.pop()       # '' if the chunk ended with a newline
        n0 = len(self.rows)
        for ln in lines:
            if not ln.strip():
                continue
            if ln.startswith('#'):
                self.n_comment += 1
                continue
            try:
                self.rows.append(parse_line(ln))
            except (ValueError, TypeError, AttributeError) as e:
                self.n_bad += 1
                if len(self.bad_examples) < 5:
                    self.bad_examples.append(f'{type(e).__name__}: {e}: {ln[:120]}')
        return len(self.rows) - n0

    @property
    def n(self):
        return len(self.rows)

    def arrays(self):
        a = np.asarray(self.rows, float).reshape(-1, len(COLS))
        return {c: a[:, i] for i, c in enumerate(COLS)}

    def window(self, a, b):
        """Rows with host time in [a, b] -> dict of arrays."""
        A = self.arrays()
        m = (A['host'] >= a) & (A['host'] <= b)
        return {k: v[m] for k, v in A.items()}


def read_ft(path):
    """Whole file -> (arrays dict, info dict with n, n_bad, bad_examples, n_comment,
    trailing_partial)."""
    t = FtTail(path)
    t.poll()
    info = dict(n=t.n, n_bad=t.n_bad, bad_examples=t.bad_examples, n_comment=t.n_comment,
                trailing_partial=bool(t.partial.strip()))
    if t.partial.strip():   # final line without newline: try it, count it bad if malformed
        try:
            t.rows.append(parse_line(t.partial))
            info['n'] = t.n
        except (ValueError, TypeError, AttributeError):
            info['n_bad'] += 1
    return t.arrays(), info


def window_stats(A, a, b, keys=('fx', 'fy', 'fz', 'tx', 'ty', 'tz')):
    m = (A['host'] >= a) & (A['host'] <= b)
    out = {'n': int(m.sum())}
    for k in keys:
        v = A[k][m]
        out[k] = dict(mean=float(v.mean()), std=float(v.std(ddof=1))) if v.size > 1 else None
    return out


# ---------------------------------------------------------------- model facts from the SDF
NOC_MODEL = os.path.join(os.path.dirname(__file__), '..', '..', 'gazebo', 'models', 'noc-tilthex', 'model.sdf')


def model_info(path=NOC_MODEL):
    """ee link mass / CoM and the ee_joint force_torque sensor settings, read from the
    noc-tilthex SDF (single source for t04 and a04)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    model = root.find('model')
    ee = model.find("link[@name='ee']")
    j = model.find("joint[@name='ee_joint']")
    s = j.find("sensor[@type='force_torque']")
    ft = s.find('force_torque')
    return dict(
        model=model.get('name'),
        ee_mass_kg=float(ee.find('inertial/mass').text),
        ee_com_m=[float(x) for x in ee.find('inertial/pose').text.split()[:3]],
        joint=j.get('name'), joint_type=j.get('type'),
        parent=j.find('parent').text, child=j.find('child').text,
        sensor=s.get('name'), topic=s.find('topic').text, update_rate_hz=float(s.find('update_rate').text),
        frame=ft.find('frame').text, measure_direction=ft.find('measure_direction').text)
