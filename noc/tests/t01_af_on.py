"""
t01_af_on — 06a contact scenario with the rotor_measure fix (F1) and non-blocking
goto (F2). src/06a-* is not modified.

Differences to t00_diag:
  * setup via noc.common.setup_06a.setup(): 06a setup() + verified
    uavatt.rotor_measure <- rotorcraft/rotor_measure (WO gets wrench_measure.force).
  * maneuver.goto(*args, ack=True): returns once the activity is started, so the
    50 Hz loop keeps sampling during motion; waypoint times are those of 06a.
  * extra sources: uavpos uav_input (thrust = force command), rotorcraft rotor_measure
    (propeller velocities); per sample /tmp/phynt.log size.
  * safety monitor (noc.common.flight.SAFETY). On a trip: retreat away from the
    wall, descend, then stop(). The trip reason and signals are saved in meta.json.
    The recorded t01 run (2026-11-04 17:21) used safety version 1; this code is
    version 2 (external-wrench check only while flying, decision P3).

Run inside the container (T2), after simulation.sh is up and Gazebo Play pressed:
  check only, no flight:  cd /shared-workspace && python3 -m noc.tests.t01_af_on --check
  flight:                 cd /shared-workspace && python3 -m noc.tests.t01_af_on
Output: logs/noc/t01_af_on/ (flight) or logs/noc/t01_check/ (check).
"""

import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np

from noc.common import setup_06a
from noc.common import telekyb_io as io
from noc.common import flight
from noc.common.flight import (SAFETY, Safety, seg_dist, components, all_sources, read_all,
                               jdump, file_size, git_rev, snapshot, make_ev, retreat, PHYNT_TMP_LOG)

REPO = setup_06a.REPO
OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't01_af_on')
CHECK_DIR = os.path.join(REPO, 'logs', 'noc', 't01_check')

TF = 80.0
DT = 0.02
T_WO_ZERO = 20.0
SNAPSHOT_TIMES = {'contact_1': 28.0, 'contact_2': 45.0}


def run_check(load=setup_06a.load_06a):
    os.makedirs(CHECK_DIR, exist_ok=True)
    m = load(CHECK_DIR)
    comps = components(m)
    sources = all_sources(m.phynt)
    res = {'sources': io.describe_sources(sources)}
    try:
        res['setup_06a'] = setup_06a.setup(m)
        m.start()
        time.sleep(1.0)  # let uavatt publish wrench_measure from rotor_measure
        res['preflight'] = io.preflight(comps, sources)
        res['snapshot'] = snapshot(m, 'check', 0.0)
    finally:
        try:
            m.stop()
        finally:
            jdump(res, 'check.json', CHECK_DIR)
    pf = res['preflight']
    ok = all(v['ok'] for v in pf.values())
    wm_force = bool(pf['wrench_meas']['ok'] and pf['wrench_meas']['valid']['force'])
    rm_vel = bool(pf['rotor_meas']['ok'] and pf['rotor_meas']['valid']['velocity'])
    print(f'[check] reads {"ALL OK" if ok else "FAILED"}; wrench_measure.force valid: {wm_force}; '
          f'rotor_measure velocity valid: {rm_vel}', flush=True)
    return 0 if (ok and wm_force and rm_vel) else 1


def run_flight(load=setup_06a.load_06a):
    os.makedirs(OUT_DIR, exist_ok=True)
    m = load(OUT_DIR)
    comps = components(m)
    sources = all_sources(m.phynt)
    rec = io.Recorder()
    events, snaps = [], {}
    ev = make_ev(events)
    safety = Safety(SAFETY)
    meta = dict(git_rev=git_rev(), model_06a=setup_06a.MODEL_06A, tf=TF, dt=DT, t_wo_zero=T_WO_ZERO,
                waypoints=m.waypoints, snapshot_times=SNAPSHOT_TIMES, safety=SAFETY,
                sources=io.describe_sources(sources), goto_mode='ack=True',
                safety_version=flight.SAFETY_VERSION, seed=None,
                seed_note='no RNG in this script; simulator has no recorded seed',
                started_unix=time.time(), abort=None)
    error = None
    err_counts = {}
    try:
        w0 = time.time(); meta['setup_06a'] = setup_06a.setup(m); ev('setup', -1.0, w0, time.time())
        w0 = time.time(); m.start(); ev('start', -1.0, w0, time.time())
        meta['preflight'] = io.preflight(comps, sources)
        jdump(meta['preflight'], 'preflight.json', OUT_DIR)
        snaps['after_start'] = snapshot(m, 'after_start', 0.0)

        p0 = io.read_pom(m.pom)[0]['pos']
        seg = (p0.copy(), p0.copy())
        N = int(TF / DT)
        wo_done = False
        wp_idx = 0
        pending = dict(SNAPSHOT_TIMES)
        t0 = time.time()
        for i in range(N):
            ts = i * DT
            if not wo_done and ts >= T_WO_ZERO:
                w0 = time.time(); m.phynt.set_wo_zero(duration=2.0); ev('phynt.set_wo_zero', ts, w0, time.time())
                wo_done = True
            if wp_idx < len(m.waypoints) and ts >= m.waypoints[wp_idx][0]:
                args = m.waypoints[wp_idx][1]
                w0 = time.time(); r = flight.goto(m, args)
                ev('maneuver.goto(ack)', ts, w0, time.time(), {'args': list(args), 'reply': repr(r)[:200]})
                seg = (seg[1], np.array(args[:3], float))
                wp_idx += 1
            for lbl, t_snap in list(pending.items()):
                if ts >= t_snap:
                    snaps[lbl] = snapshot(m, lbl, ts)
                    del pending[lbl]

            t_wall = time.time()
            row, errors, read_s = read_all(comps, sources, err_counts)
            trips, sig = safety.check(row, wo_done, seg)
            rec.add_scalar('i', i)
            rec.add_scalar('t_nominal', ts)
            rec.add_scalar('t_wall', t_wall)
            rec.add_scalar('t_mono', time.monotonic())
            rec.add_scalar('read_s', read_s)
            rec.add_scalar('late_s', (t_wall - t0) - ts)
            rec.add_scalar('wp_idx', wp_idx)
            rec.add_scalar('phynt_log_bytes', file_size(PHYNT_TMP_LOG) or -1)
            for k, v in sig.items():
                rec.add_scalar(f'safety.{k}', v)
            for src, sample in row.items():
                rec.add(src, sample)
                rec.add_scalar(f'{src}.error', errors[src])

            if i % 50 == 0:
                p = row['pom'][0]['pos']; f = row['ext_wrench'][0]['force']
                print(f't {ts:5.1f} pos [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] ext_f [{f[0]:.2f} {f[1]:.2f} {f[2]:.2f}] '
                      f"f_cmd_xy {sig['f_cmd_xy']:.2f} e_seg {sig['e_seg']:.3f} w [{sig['w_min']:.1f},{sig['w_max']:.1f}] "
                      f"phynt.log {file_size(PHYNT_TMP_LOG)} B", flush=True)

            if trips:
                meta['abort'] = dict(t_nominal=ts, t_wall=t_wall, reasons=trips,
                                     signals={k: (None if isinstance(v, float) and math.isnan(v) else v)
                                              for k, v in sig.items()})
                print(f'[SAFETY] TRIP at t={ts:.2f}: {trips} -> retreat, land, stop()', flush=True)
                retreat(m, ev, ts)
                break

            target = (i + 1) * DT
            elapsed = time.time() - t0
            if target > elapsed:
                time.sleep(target - elapsed)

        snaps['before_stop'] = snapshot(m, 'before_stop', TF)
    except BaseException as e:
        error = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        raise
    finally:
        try:
            w0 = time.time(); m.stop(); ev('stop', -1.0, w0, time.time())
        finally:
            meta['finished_unix'] = time.time()
            meta['error'] = error
            meta['read_error_counts'] = err_counts
            meta['tmp_phynt_log_bytes_after_stop'] = file_size(PHYNT_TMP_LOG)
            meta['out_phynt_log_bytes'] = file_size(os.path.join(OUT_DIR, 'phynt.log'))
            jdump(meta, 'meta.json', OUT_DIR)
            jdump(events, 'events.json', OUT_DIR)
            jdump(snaps, 'snapshots.json', OUT_DIR)
            arrays = rec.to_arrays() if rec.cols else {}
            np.savez(os.path.join(OUT_DIR, 't01_af_on.npz'), **arrays,
                     meta_json=json.dumps(meta, default=str),
                     events_json=json.dumps(events, default=str),
                     snapshots_json=json.dumps(snaps, default=str))
            print(f'saved {len(arrays.get("i", []))} samples to {OUT_DIR}; abort={meta["abort"]}', flush=True)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='setup+start+preflight+stop, no flight')
    a = ap.parse_args(argv)
    return run_check() if a.check else run_flight()


if __name__ == '__main__':
    sys.exit(main())
