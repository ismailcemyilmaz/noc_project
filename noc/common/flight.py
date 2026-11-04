"""
Shared flight-experiment plumbing for noc experiments (made default after
noc/results/01_af_on.md, decisions P1–P3 on 2026-11-04).

Defaults every new noc experiment must use:
  * P1 setup: noc.common.setup_06a.setup(m)  (06a setup() + uavatt.rotor_measure
    <- rotorcraft/rotor_measure; without it the WO/AF chain is dead, see 00b).
  * P2 goto:  flight.goto(m, args)  -> maneuver.goto(*args, ack=True) (non-blocking;
    a blocking goto stalls the 50 Hz recorder for the whole motion, see 00b E1).
  * P3 safety: flight.Safety(SAFETY) — the external-wrench check is active only while
    flying (z > z_fly), so ground contact at take-off/landing does not trip it
    (t01 tripped at landing, 01_af_on §7).

src/06a-physical-interaction/ is never modified; it is loaded via setup_06a.load_06a.
"""

import json
import math
import os
import subprocess
import time

import numpy as np

from noc.common import setup_06a
from noc.common import telekyb_io as io

REPO = setup_06a.REPO
PHYNT_TMP_LOG = '/tmp/phynt.log'

SAFETY_VERSION = 2  # 1: t01 run (f_ext check also on ground); 2: f_ext check only while flying
SAFETY = dict(
    f_max=3.5,         # N: |external_wrench force| (after set_wo_zero, while flying) or |uavpos thrust xy|
    f_persist=5,       # consecutive 50 Hz samples (0.1 s)
    e_max=0.40,        # m: distance of pom position from current waypoint segment (t00 max |e| 0.22)
    e_persist=10,      # 0.2 s
    w_hi=98.0,         # rotor velocity, uavatt wlimit units (wmax 100; t00 measured peak 95.6)
    w_lo=18.0,         # while flying (wmin 16; t00 flying min 22.4)
    w_persist=3,
    z_fly=0.5,         # m: "flying" for the f_ext, e and w_lo checks
    retreat_dx=0.5,    # m away from the wall (-x)
    retreat_s=3.0,
    land_s=5.0,
)


# ---------------------------------------------------------------- small helpers
def file_size(path):
    return os.path.getsize(path) if os.path.exists(path) else None


def git_rev():
    r = subprocess.run(['git', '-C', REPO, 'rev-parse', 'HEAD'], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else f'unavailable: {r.stderr.strip()}'


def jdump(obj, name, out_dir):
    with open(os.path.join(out_dir, name), 'w') as f:
        json.dump(obj, f, indent=1, default=str)


def make_ev(events):
    def ev(name, t_nom, w0, w1, extra=None):
        e = dict(name=name, t_nominal=t_nom, wall_start=w0, wall_end=w1, blocked_s=w1 - w0)
        if extra is not None:
            e['args'] = extra
        events.append(e)
        print(f'[t={t_nom:5.2f}] {name} blocked {w1 - w0:.3f}s {"" if extra is None else extra}', flush=True)
    return ev


def snapshot(m, label, t_nom):
    s = io.phynt_snapshot(m.phynt)
    s['t_nominal'] = t_nom
    s['t_wall'] = time.time()
    s['tmp_phynt_log_bytes'] = file_size(PHYNT_TMP_LOG)
    print(f'[snapshot {label}] {json.dumps(s, default=str)}', flush=True)
    return s


# ---------------------------------------------------------------- components / sources
def components(m):
    return {'pom': m.pom, 'phynt': m.phynt, 'maneuver': m.maneuver, 'uavatt': m.uavatt,
            'uavpos': m.uavpos, 'rotorcraft': m.rotorcraft}


def all_sources(phynt):
    s = io.build_sources(phynt)
    s.update(io.EXTRA_SOURCES)
    return s


def read_all(comps, sources, err_counts):
    """Read every source. A failing read is recorded (exception text in errors[src],
    first 5 per source printed, counted), its sample is all-NaN with valid=False,
    and the loop continues."""
    t0 = time.monotonic()
    row, errors = {}, {}
    for src, (comp, reader, fields) in sources.items():
        try:
            row[src] = reader(comps[comp])
            errors[src] = ''
        except Exception as e:  # recorded, not swallowed
            row[src] = io.invalid_sample(fields)
            errors[src] = f'{type(e).__name__}: {e}'
            err_counts[src] = err_counts.get(src, 0) + 1
            if err_counts[src] <= 5:
                print(f'[read error] {src}: {errors[src]}', flush=True)
    return row, errors, time.monotonic() - t0


# ---------------------------------------------------------------- motion
def goto(m, args, ack=True):
    """P2 default: non-blocking maneuver.goto. Returns the genomix reply."""
    return m.maneuver.goto(*args, ack=True) if ack else m.maneuver.goto(*args)


def retreat(m, ev, t_nom, cfg=SAFETY):
    """Blocking: move away from the wall, then descend. stop() is called by the caller."""
    p = io.read_pom(m.pom)[0]['pos']
    xr = float(p[0] - cfg['retreat_dx']) if p[0] > 1.0 else float(p[0])
    args = (xr, float(p[1]), float(p[2]), 0, cfg['retreat_s'])
    w0 = time.time(); goto(m, args, ack=False); ev('SAFETY retreat goto', t_nom, w0, time.time(), list(args))
    args = (xr, float(p[1]), 0.0, 0, cfg['land_s'])
    w0 = time.time(); goto(m, args, ack=False); ev('SAFETY land goto', t_nom, w0, time.time(), list(args))


# ---------------------------------------------------------------- safety
def seg_dist(p, a, b):
    """Distance of point p from segment a-b (3D)."""
    ab = b - a
    L2 = float(ab @ ab)
    u = 0.0 if L2 == 0 else min(max(float((p - a) @ ab) / L2, 0.0), 1.0)
    return float(np.linalg.norm(p - (a + u * ab)))


class Safety:
    """Persistence-filtered trip checks on one 50 Hz sample (see SAFETY)."""

    version = SAFETY_VERSION

    def __init__(self, cfg=SAFETY):
        self.cfg = cfg
        self.cnt = {'f_ext': 0, 'f_cmd': 0, 'e_seg': 0, 'w_hi': 0, 'w_lo': 0}

    def _tick(self, key, cond, n):
        self.cnt[key] = self.cnt[key] + 1 if cond else 0
        return self.cnt[key] >= n

    def check(self, row, wo_calibrated, seg):
        c = self.cfg
        p = row['pom'][0]['pos']
        fe, fe_v = row['ext_wrench']
        ui, ui_v = row['uav_input']
        rm, rm_v = row['rotor_meas']
        flying = bool(np.isfinite(p[2]) and p[2] > c['z_fly'])
        sig = dict(
            # P3: only while flying -> ground reaction at take-off/landing is ignored
            f_ext=(float(np.linalg.norm(fe['force'])) if (fe_v['force'] and wo_calibrated and flying)
                   else math.nan),
            f_cmd_xy=float(np.hypot(*ui['thrust'][:2])) if ui_v['thrust'] else math.nan,
            e_seg=seg_dist(p, *seg) if (flying and np.all(np.isfinite(p))) else math.nan,
            w_max=float(np.nanmax(rm['velocity'])) if rm_v['velocity'] else math.nan,
            w_min=float(np.nanmin(rm['velocity'])) if rm_v['velocity'] else math.nan,
            flying=flying,
        )
        trips = []
        if self._tick('f_ext', sig['f_ext'] > c['f_max'], c['f_persist']):
            trips.append(f"external_wrench |f|={sig['f_ext']:.2f} N > {c['f_max']}")
        if self._tick('f_cmd', sig['f_cmd_xy'] > c['f_max'], c['f_persist']):
            trips.append(f"uavpos |thrust_xy|={sig['f_cmd_xy']:.2f} N > {c['f_max']}")
        if self._tick('e_seg', sig['e_seg'] > c['e_max'], c['e_persist']):
            trips.append(f"segment error {sig['e_seg']:.3f} m > {c['e_max']}")
        if self._tick('w_hi', sig['w_max'] > c['w_hi'], c['w_persist']):
            trips.append(f"rotor velocity {sig['w_max']:.1f} > {c['w_hi']}")
        if self._tick('w_lo', flying and sig['w_min'] < c['w_lo'], c['w_persist']):
            trips.append(f"rotor velocity {sig['w_min']:.1f} < {c['w_lo']} while flying")
        return trips, sig
