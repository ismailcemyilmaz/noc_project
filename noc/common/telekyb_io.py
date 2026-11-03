"""
Readers for telekyb3 GenoM3 ports/services through genomix.

Interface verified by noc/tests/t00_probe.py (logs/t00_probe.txt) and
noc/docs/genom/phynt/README.adoc:
  pom.frame('robot')        -> {'frame': or_pose_estimator::state}
  phynt.desired()           -> {'desired': or_rigid_body::state}
  phynt.reference()         -> {'reference': or_rigid_body::state}       (input port)
  phynt.external_wrench()   -> {'external_wrench': or_wrench_estimator::state}
  phynt.wrench_measure()    -> {'wrench_measure': or_wrench_estimator::state}  (input port)
  maneuver.desired()        -> {'desired': or_rigid_body::state}; before any goto all
                               optional fields are None and ts = 0
  phynt.get_af_parameters() -> {'af': {...}};  get_wo_parameters() -> {'wo': {...}}
  phynt.log_info()          -> {'miss': .., 'total': ..}
  (phynt has no get_wo_data.)

Rules:
  * No exception is caught here. A failing genomix call propagates to the caller.
  * Optional fields that are None become NaN arrays AND a False entry in the
    returned ``valid`` dict. NaN is never silently replaced by 0.
  * A port timestamp of exactly 0 s 0 ns is reported as ``ts = nan`` with
    ``valid['ts'] = False`` (port never written).
"""

import math

import numpy as np

NAN3 = (math.nan,) * 3

# field name -> component keys, for the optional sub-structures we read
_FIELDS = {
    'pos': ('x', 'y', 'z'),
    'att': ('qw', 'qx', 'qy', 'qz'),
    'vel': ('vx', 'vy', 'vz'),
    'avel': ('wx', 'wy', 'wz'),
    'acc': ('ax', 'ay', 'az'),
    'force': ('x', 'y', 'z'),
    'torque': ('x', 'y', 'z'),
}


class PortFormatError(RuntimeError):
    """A genomix reply did not have the expected structure."""


def unwrap(reply, key):
    """genomix replies are ``{key: payload}``. Raise if ``key`` is absent."""
    if not isinstance(reply, dict) or key not in reply:
        keys = list(reply) if isinstance(reply, dict) else type(reply).__name__
        raise PortFormatError(f'expected reply key {key!r}, got {keys}')
    return reply[key]


def ts_to_float(ts):
    """``{'sec','nsec'}`` -> float seconds, NaN if the port was never written."""
    if ts is None:
        return math.nan
    sec, nsec = ts['sec'], ts['nsec']
    if sec == 0 and nsec == 0:
        return math.nan
    return sec + 1e-9 * nsec


def flatten(state, fields):
    """Optional-field struct -> ({field: np.array}, {field: bool valid}).

    Also returns ``ts`` (float, NaN if unset) under key 'ts'.
    """
    out, valid = {}, {}
    t = ts_to_float(state.get('ts'))
    out['ts'] = t
    valid['ts'] = not math.isnan(t)
    for f in fields:
        if f not in state:
            raise PortFormatError(f'field {f!r} missing from {list(state)}')
        keys = _FIELDS[f]
        v = state[f]
        if v is None:
            out[f] = np.full(len(keys), np.nan)
            valid[f] = False
        else:
            out[f] = np.array([float(v[k]) for k in keys])
            valid[f] = True
    return out, valid


# ---------------------------------------------------------------- port readers
STATE_FIELDS = ('pos', 'att', 'vel', 'avel', 'acc')
RB_FIELDS = ('pos', 'att', 'vel', 'acc')
WRENCH_FIELDS = ('force', 'torque')


def read_pom(pom, port='robot'):
    return flatten(unwrap(pom.frame(port), 'frame'), STATE_FIELDS)


def read_phynt_desired(phynt):
    return flatten(unwrap(phynt.desired(), 'desired'), RB_FIELDS)


def read_phynt_reference(phynt):
    return flatten(unwrap(phynt.reference(), 'reference'), RB_FIELDS)


def read_maneuver_desired(maneuver):
    return flatten(unwrap(maneuver.desired(), 'desired'), RB_FIELDS)


def read_external_wrench(phynt):
    return flatten(unwrap(phynt.external_wrench(), 'external_wrench'), WRENCH_FIELDS)


def read_wrench_measure(phynt):
    return flatten(unwrap(phynt.wrench_measure(), 'wrench_measure'), WRENCH_FIELDS)


def invalid_sample(fields):
    """All-NaN sample with every valid flag False (for a read that raised;
    the caller must record the exception text separately)."""
    out = {'ts': math.nan, **{f: np.full(len(_FIELDS[f]), np.nan) for f in fields}}
    return out, {'ts': False, **{f: False for f in fields}}


# name -> (reader, fields); used by recorders that log per-source errors
SOURCES = {
    'pom': (read_pom, STATE_FIELDS),
    'phynt_ref': (read_phynt_reference, RB_FIELDS),
    'phynt_des': (read_phynt_desired, RB_FIELDS),
    'man_des': (read_maneuver_desired, RB_FIELDS),
    'ext_wrench': (read_external_wrench, WRENCH_FIELDS),
    'wrench_meas': (read_wrench_measure, WRENCH_FIELDS),
}


# ---------------------------------------------------------------- services
def phynt_snapshot(phynt):
    """Raw replies of the parameter/log services (JSON-serialisable dicts)."""
    return {
        'get_af_parameters': phynt.get_af_parameters(),
        'get_wo_parameters': phynt.get_wo_parameters(),
        'get_body_parameters': phynt.get_body_parameters(),
        'log_info': phynt.log_info(),
    }


# ---------------------------------------------------------------- recorder
class Recorder:
    """Accumulates flattened samples: one array per '<source>.<field>' plus
    '<source>.valid.<field>' booleans. ``to_arrays`` stacks them."""

    def __init__(self):
        self.cols = {}

    def add(self, source, sample):
        out, valid = sample
        for k, v in out.items():
            self.cols.setdefault(f'{source}.{k}', []).append(v)
        for k, v in valid.items():
            self.cols.setdefault(f'{source}.valid.{k}', []).append(bool(v))

    def add_scalar(self, name, value):
        self.cols.setdefault(name, []).append(value)

    def to_arrays(self):
        n = {len(v) for v in self.cols.values()}
        if len(n) > 1:
            raise ValueError(f'ragged recorder columns: { {k: len(v) for k, v in self.cols.items()} }')
        return {k: np.asarray(v) for k, v in self.cols.items()}
