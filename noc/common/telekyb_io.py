"""
Readers for telekyb3 GenoM3 ports/services through genomix.

Interface: pom.frame / phynt.desired / external_wrench / maneuver.desired verified
live by noc/tests/t00_probe.py (logs/t00_probe.txt); get_reference and
uavatt.wrench_measure taken from noc/docs/genom/{phynt,uavatt}/README.adoc and checked
live by noc/tests/t00_check.py:
  pom.frame('robot')        -> {'frame': or_pose_estimator::state}
  phynt.desired()           -> {'desired': or_rigid_body::state}
  phynt.get_reference()     -> {'reference': or_rigid_body::state}       (attribute service)
  phynt.external_wrench()   -> {'external_wrench': or_wrench_estimator::state}
  uavatt.wrench_measure()   -> {'wrench_measure': or_wrench_estimator::state}  (uavatt OUTPUT port)
  maneuver.desired()        -> {'desired': or_rigid_body::state}; before any goto all
                               optional fields are None and ts = 0
  phynt.get_af_parameters() -> {'af': {...}};  get_wo_parameters() -> {'wo': {...}}
  phynt.log_info()          -> {'miss': .., 'total': ..}
  (phynt has no get_wo_data.)
  Input ports (phynt.reference, phynt.wrench_measure, phynt.state) are NOT read:
  genomix requires a 'path' argument for them (TypeError seen in t00_diag run 1).

Rules:
  * Port readers (read_*) catch nothing: a failing genomix call propagates.
    Only preflight() and phynt_snapshot() record exceptions instead of raising.
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
    'thrust': ('x', 'y', 'z'),
}
MAX_ROTORS = 8  # or_rotorcraft::output rotor sequence bound


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


def read_phynt_get_reference(phynt):
    """phynt attribute service get_reference (copy of the reference phynt uses)."""
    return flatten(unwrap(phynt.get_reference(), 'reference'), RB_FIELDS)


def read_maneuver_desired(maneuver):
    return flatten(unwrap(maneuver.desired(), 'desired'), RB_FIELDS)


def read_external_wrench(phynt):
    return flatten(unwrap(phynt.external_wrench(), 'external_wrench'), WRENCH_FIELDS)


def read_uavatt_wrench_measure(uavatt):
    """uavatt OUTPUT port wrench_measure (wrench from propeller measurements +
    geometric model); this is what 06a connects to phynt/wrench_measure."""
    return flatten(unwrap(uavatt.wrench_measure(), 'wrench_measure'), WRENCH_FIELDS)


def read_uavpos_uav_input(uavpos):
    """uavpos OUTPUT port uav_input; 'thrust' is the force command (uavpos.log fx fy fz)."""
    return flatten(unwrap(uavpos.uav_input(), 'uav_input'), ('thrust',))


def read_rotor_measure(rotorcraft):
    """rotorcraft OUTPUT port rotor_measure (or_rotorcraft::output, doc: uavatt README
    rotor_measure). Returns velocity[MAX_ROTORS] (NaN-padded), n_rotors, latest rotor ts."""
    rm = unwrap(rotorcraft.rotor_measure(), 'rotor_measure')
    if 'rotor' not in rm:
        raise PortFormatError(f"field 'rotor' missing from {list(rm)}")
    rot = rm['rotor'] or []
    vel = np.full(MAX_ROTORS, np.nan)
    tss = []
    for i, r in enumerate(rot[:MAX_ROTORS]):
        vel[i] = float(r['velocity'])
        tss.append(ts_to_float(r.get('ts')))
    t = max([x for x in tss if not math.isnan(x)], default=math.nan)
    out = {'ts': t, 'velocity': vel, 'n_rotors': float(len(rot))}
    valid = {'ts': not math.isnan(t), 'velocity': len(rot) > 0, 'n_rotors': True}
    return out, valid


def invalid_sample(fields):
    """All-NaN sample with every valid flag False (for a read that raised;
    the caller must record the exception text separately)."""
    out, valid = {'ts': math.nan}, {'ts': False}
    for f in fields:
        if f == 'velocity':
            out[f] = np.full(MAX_ROTORS, np.nan)
        elif f == 'n_rotors':
            out[f] = math.nan
        else:
            out[f] = np.full(len(_FIELDS[f]), np.nan)
        valid[f] = False
    return out, valid


def reference_source(phynt):
    """phynt.get_reference if the handle exposes it, else maneuver.desired
    (which 06a wires to phynt/reference). Returns (component name, reader)."""
    if hasattr(phynt, 'get_reference'):
        return 'phynt', read_phynt_get_reference
    return 'maneuver', read_maneuver_desired


def build_sources(phynt):
    """name -> (component name, reader, fields). Only output ports / attribute
    services; input ports are never read from Python."""
    ref_comp, ref_reader = reference_source(phynt)
    return {
        'pom': ('pom', read_pom, STATE_FIELDS),
        'phynt_ref': (ref_comp, ref_reader, RB_FIELDS),
        'phynt_des': ('phynt', read_phynt_desired, RB_FIELDS),
        'man_des': ('maneuver', read_maneuver_desired, RB_FIELDS),
        'ext_wrench': ('phynt', read_external_wrench, WRENCH_FIELDS),
        'wrench_meas': ('uavatt', read_uavatt_wrench_measure, WRENCH_FIELDS),
    }


EXTRA_SOURCES = {
    'uav_input': ('uavpos', read_uavpos_uav_input, ('thrust',)),
    'rotor_meas': ('rotorcraft', read_rotor_measure, ('velocity', 'n_rotors')),
}


def describe_sources(sources):
    return {k: f'{c}.{r.__name__}' for k, (c, r, _) in sources.items()}


def preflight(components, sources):
    """Read every source once. Never raises for a read failure: returns
    {src: {'ok', 'reader', 'error', 'valid', 'raw'}} and prints failures."""
    rep = {}
    for src, (comp, reader, fields) in sources.items():
        e = {'reader': f'{comp}.{reader.__name__}'}
        try:
            out, valid = reader(components[comp])
            e.update(ok=True, error=None, valid=valid,
                     values={k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in out.items()})
        except Exception as ex:  # recorded and printed, not hidden
            e.update(ok=False, error=f'{type(ex).__name__}: {ex}', valid=None, values=None)
        rep[src] = e
    bad = {k: v['error'] for k, v in rep.items() if not v['ok']}
    print(f'[preflight] sources: {describe_sources(sources)}', flush=True)
    for k, v in rep.items():
        print(f'[preflight] {k:12s} {"OK  " if v["ok"] else "FAIL"} {v["reader"]} '
              f'{v["valid"] if v["ok"] else v["error"]}', flush=True)
    print(f'[preflight] {len(rep) - len(bad)}/{len(rep)} sources OK'
          + (f'; FAILED: {sorted(bad)}' if bad else ''), flush=True)
    return rep


# ---------------------------------------------------------------- services
SNAPSHOT_SERVICES = ('get_af_parameters', 'get_wo_parameters', 'get_body_parameters',
                     'get_reference', 'log_info')


def phynt_snapshot(phynt):
    """Call each parameter/log service independently. A failing call is
    recorded as {'error': '<Type>: <msg>'} and does not stop the others."""
    out = {}
    for name in SNAPSHOT_SERVICES:
        try:
            out[name] = getattr(phynt, name)()
        except Exception as ex:  # recorded, not hidden
            out[name] = {'error': f'{type(ex).__name__}: {ex}'}
    return out


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
