"""
t02_live_k — live change of the admittance stiffness K_x (phynt AF) during steady
wall contact. src/06a-* is not modified; the flight path is the noc default:
setup_06a.setup (rotor_measure fix), flight.goto (ack=True), flight.Safety (v2).

Scenario (nominal time, s):
   2  goto (0, 0, 1)            takeoff, 5 s
  10  goto (1.2, 0, 1)          approach, 4 s
  20  phynt.set_wo_zero(2 s)    WO calibration in hover
  22  goto (BODY_X_CONTACT, 0, 1)  contact at y = 0, z = 1, 2 s; held until the retreat
  30  steady contact assumed (goto ends at 24 s + ~4.1 s AF settling, 01_af_on §4); K_x = 10
  42, 54, 66, 78, 90  K_x -> 12, 15, 20, 15, 10 (12 s each, ends 102)
 102  200 x set_af_parameters with unchanged values (K_x = 10), one per 50 Hz cycle
 108  retreat goto (1.0, 0, 1), 4 s
 114  land goto (0, 0, 0), 5 s
 122  stop()

Every set_af_parameters call keeps mass, B, J and K[1:] at the values found after
setup (checked against noc/config/af_on_baseline.json before takeoff); only K[0]
changes. B is NOT rescaled with K_x, so the AF damping ratio changes with K_x.
Each call is timed with time.perf_counter and followed by get_af_parameters; the
read-back must equal the commanded (mass, B, K, J). A failing call or a mismatch
aborts the experiment: retreat, land, stop() (recorded in meta.json and k_calls.json).

Per 50 Hz cycle the recorder stores: time (nominal, wall, monotonic, lateness, read
time), pom, phynt desired / reference, maneuver desired, external_wrench, uavpos
uav_input (thrust x = fx), uavatt wrench_measure, rotor velocities, K_x target,
get_af_parameters (mass, K, B, J) and the set/verify timing of that cycle.

Run inside the container (T2), after simulation.sh is up and Gazebo Play pressed:
  check only, no flight:  cd /shared-workspace && python3 -m noc.tests.t02_live_k --check
  flight:                 cd /shared-workspace && python3 -m noc.tests.t02_live_k
Output: logs/noc/t02_live_k/ (flight) or logs/noc/t02_check/ (check).
Exit code: 0 = completed, 1 = check failed or experiment aborted.
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
from noc.common.flight import (SAFETY, Safety, components, all_sources, read_all, jdump, file_size,
                               git_rev, snapshot, make_ev, retreat, PHYNT_TMP_LOG)

REPO = setup_06a.REPO
OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't02_live_k')
CHECK_DIR = os.path.join(REPO, 'logs', 'noc', 't02_check')
CONFIG = os.path.join(REPO, 'noc', 'config', 'af_on_baseline.json')
NPZ_NAME = 't02_live_k.npz'

DT = 0.02
T_WO_ZERO = 20.0
X_CONTACT = 1.55          # 06a BODY_X_CONTACT = X_WALL - L_BAR + 0.15 (checked against the module)
WAYPOINTS = [
    (2.0, (0.0, 0.0, 1.0, 0, 5.0)),
    (10.0, (1.2, 0.0, 1.0, 0, 4.0)),
    (22.0, (X_CONTACT, 0.0, 1.0, 0, 2.0)),
    (108.0, (1.0, 0.0, 1.0, 0, 4.0)),     # retreat from the wall
    (114.0, (0.0, 0.0, 0.0, 0, 5.0)),     # land
]
T_SETTLED = 30.0
HOLD_S = 12.0
K_SEQUENCE = [10.0, 12.0, 15.0, 20.0, 15.0, 10.0]
K_SCHEDULE = [(T_SETTLED + i * HOLD_S, k) for i, k in enumerate(K_SEQUENCE)]   # 30 .. 90
T_REPEAT = T_SETTLED + len(K_SEQUENCE) * HOLD_S                                # 102
N_REPEAT = 200
TF = 122.0
SNAPSHOT_TIMES = {'settled': T_SETTLED, 'before_repeat': T_REPEAT, 'after_repeat': T_REPEAT + 5.0}

CALL_NONE, CALL_SCHEDULE, CALL_REPEAT, CALL_CHECK = 0, 1, 2, 3


class AFMismatch(RuntimeError):
    pass


# ---------------------------------------------------------------- AF helpers
def af_baseline(path=CONFIG):
    """Expected AF after 06a setup(): mass, K, J from the config, B by its rule."""
    with open(path) as f:
        af = json.load(f)['phynt_af']
    mass, K = float(af['mass']), np.asarray(af['K'], float)
    return dict(mass=mass, K=K, B=2.0 * np.sqrt(mass * K), J=np.asarray(af['J'], float))


def af_equal(read, expected, rtol=1e-9, atol=1e-12):
    """read: telekyb_io.read_af_parameters output. Returns (ok, {field: ok})."""
    det = {f: bool(np.allclose(read[f], expected[f], rtol=rtol, atol=atol)) for f in io.AF_FIELDS}
    return all(det.values()), det


def af_command(base, kx):
    """mass, B, J and K[1:] from base; K[0] = kx."""
    K = np.array(base['K'], float)
    K[0] = kx
    return dict(mass=float(base['mass']), B=np.array(base['B'], float), K=K, J=np.array(base['J'], float))


def set_and_verify(phynt, cmd, kind, t_nom, calls):
    """One timed set_af_parameters + get_af_parameters read-back. Appends a record to
    ``calls``. Raises the genomix exception, or AFMismatch, after recording it."""
    rec = dict(kind=kind, t_nominal=t_nom, t_wall=time.time(), K_x=float(cmd['K'][0]),
               set_s=None, get_s=None, verify=None, detail=None, error=None)
    calls.append(rec)
    try:
        p0 = time.perf_counter()
        reply = phynt.set_af_parameters(mass=cmd['mass'], B=cmd['B'].tolist(), K=cmd['K'].tolist(),
                                        J=cmd['J'].tolist())
        rec['set_s'] = time.perf_counter() - p0
        rec['reply'] = repr(reply)[:200]
        p0 = time.perf_counter()
        back, _ = io.read_af_parameters(phynt)
        rec['get_s'] = time.perf_counter() - p0
    except Exception as e:
        rec['error'] = f'{type(e).__name__}: {e}'
        raise
    ok, det = af_equal(back, cmd)
    rec['verify'] = ok
    rec['detail'] = det
    rec['read_back'] = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in back.items()}
    if not ok:
        raise AFMismatch(f'get_af_parameters after set (K_x={cmd["K"][0]}): mismatch {det}')
    return rec


def check_initial_af(m, base):
    """AF as left by 06a setup() must equal the reference config before anything is changed."""
    read, _ = io.read_af_parameters(m.phynt)
    ok, det = af_equal(read, base)
    res = dict(ok=ok, detail=det, read={k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                        for k, v in read.items()})
    if not ok:
        raise AFMismatch(f'AF after setup differs from {CONFIG}: {det} read={res["read"]}')
    return read, res


def check_contact_x(m):
    x = getattr(m, 'BODY_X_CONTACT', None)
    if x is None or not math.isclose(float(x), X_CONTACT, abs_tol=1e-9):
        raise RuntimeError(f'06a BODY_X_CONTACT={x} differs from t02 X_CONTACT={X_CONTACT}')


def sources_for(phynt):
    s = all_sources(phynt)
    s['af'] = ('phynt', io.read_af_parameters, io.AF_FIELDS)
    return s


# ---------------------------------------------------------------- check mode
def run_check(load=setup_06a.load_06a):
    os.makedirs(CHECK_DIR, exist_ok=True)
    m = load(CHECK_DIR)
    comps = components(m)
    sources = sources_for(m.phynt)
    res = {'sources': io.describe_sources(sources)}
    calls = []
    af_ok = False
    try:
        check_contact_x(m)
        res['setup_06a'] = setup_06a.setup(m)
        m.start()
        time.sleep(1.0)  # let uavatt publish wrench_measure from rotor_measure
        res['preflight'] = io.preflight(comps, sources)
        res['snapshot'] = snapshot(m, 'check', 0.0)
        base = af_baseline()
        read, res['initial_af'] = check_initial_af(m, base)
        # one set with the unchanged values (on the ground, no effect) to test the round trip
        set_and_verify(m.phynt, af_command(read, float(read['K'][0])), CALL_CHECK, 0.0, calls)
        af_ok = True
    except Exception as e:
        res['error'] = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        print(f'[check] ERROR {type(e).__name__}: {e}', flush=True)
    finally:
        try:
            m.stop()
        finally:
            res['k_calls'] = calls
            jdump(res, 'check.json', CHECK_DIR)
    pf = res.get('preflight') or {}
    ok = bool(pf) and all(v['ok'] for v in pf.values())
    wm_force = bool(pf.get('wrench_meas', {}).get('ok') and pf['wrench_meas']['valid']['force'])
    rm_vel = bool(pf.get('rotor_meas', {}).get('ok') and pf['rotor_meas']['valid']['velocity'])
    set_ms = calls[-1]['set_s'] * 1e3 if calls and calls[-1]['set_s'] is not None else math.nan
    print(f'[check] reads {"ALL OK" if ok else "FAILED"}; wrench_measure.force valid: {wm_force}; '
          f'rotor_measure velocity valid: {rm_vel}; af round trip: {af_ok} (set {set_ms:.2f} ms)', flush=True)
    return 0 if (ok and wm_force and rm_vel and af_ok) else 1


# ---------------------------------------------------------------- flight
def run_flight(load=setup_06a.load_06a):
    os.makedirs(OUT_DIR, exist_ok=True)
    m = load(OUT_DIR)
    comps = components(m)
    sources = sources_for(m.phynt)
    rec = io.Recorder()
    events, snaps, calls = [], {}, []
    ev = make_ev(events)
    safety = Safety(SAFETY)
    meta = dict(git_rev=git_rev(), model_06a=setup_06a.MODEL_06A, tf=TF, dt=DT, t_wo_zero=T_WO_ZERO,
                waypoints=WAYPOINTS, k_schedule=K_SCHEDULE, t_settled=T_SETTLED, hold_s=HOLD_S,
                t_repeat=T_REPEAT, n_repeat=N_REPEAT, snapshot_times=SNAPSHOT_TIMES, safety=SAFETY,
                safety_version=flight.SAFETY_VERSION, sources=io.describe_sources(sources),
                goto_mode='ack=True', config=CONFIG, seed=None,
                seed_note='no RNG in this script; simulator has no recorded seed',
                started_unix=time.time(), abort=None)
    error = None
    err_counts = {}
    try:
        check_contact_x(m)
        w0 = time.time(); meta['setup_06a'] = setup_06a.setup(m); ev('setup', -1.0, w0, time.time())
        base_read, meta['initial_af'] = check_initial_af(m, af_baseline())
        base = af_command(base_read, float(base_read['K'][0]))
        w0 = time.time(); m.start(); ev('start', -1.0, w0, time.time())
        meta['preflight'] = io.preflight(comps, sources)
        jdump(meta['preflight'], 'preflight.json', OUT_DIR)
        snaps['after_start'] = snapshot(m, 'after_start', 0.0)

        p0 = io.read_pom(m.pom)[0]['pos']
        seg = (p0.copy(), p0.copy())
        N = int(round(TF / DT))
        wo_done = False
        wp_idx = k_idx = n_rep = 0
        k_target = float(base['K'][0])
        expected = base
        pending = dict(SNAPSHOT_TIMES)
        t0 = time.time()
        for i in range(N):
            ts = i * DT
            if not wo_done and ts >= T_WO_ZERO:
                w0 = time.time(); m.phynt.set_wo_zero(duration=2.0); ev('phynt.set_wo_zero', ts, w0, time.time())
                wo_done = True
            if wp_idx < len(WAYPOINTS) and ts >= WAYPOINTS[wp_idx][0]:
                args = WAYPOINTS[wp_idx][1]
                w0 = time.time(); r = flight.goto(m, args)
                ev('maneuver.goto(ack)', ts, w0, time.time(), {'args': list(args), 'reply': repr(r)[:200]})
                seg = (seg[1], np.array(args[:3], float))
                wp_idx += 1
            for lbl, t_snap in list(pending.items()):
                if ts >= t_snap:
                    snaps[lbl] = snapshot(m, lbl, ts)
                    del pending[lbl]

            call_kind, set_s, get_s, verify = CALL_NONE, math.nan, math.nan, math.nan
            af_abort = None
            try:
                if k_idx < len(K_SCHEDULE) and ts >= K_SCHEDULE[k_idx][0]:
                    kx = K_SCHEDULE[k_idx][1]
                    k_idx += 1
                    if kx == k_target:   # first entry, K_x = 10 at settle: no call, per-cycle af_match checks it
                        ev(f'hold K_x={kx}', ts, time.time(), time.time())
                    else:
                        expected = af_command(base, kx)
                        call_kind = CALL_SCHEDULE
                        r = set_and_verify(m.phynt, expected, call_kind, ts, calls)
                        k_target = kx
                        set_s, get_s, verify = r['set_s'], r['get_s'], 1.0
                        ev(f'set_af K_x={kx}', ts, r['t_wall'], r['t_wall'] + r['set_s'],
                           {'set_ms': r['set_s'] * 1e3, 'get_ms': r['get_s'] * 1e3})
                elif T_REPEAT <= ts and n_rep < N_REPEAT:
                    call_kind = CALL_REPEAT
                    r = set_and_verify(m.phynt, expected, call_kind, ts, calls)
                    n_rep += 1
                    set_s, get_s, verify = r['set_s'], r['get_s'], 1.0
                    if n_rep in (1, N_REPEAT):
                        ev(f'set_af repeat {n_rep}/{N_REPEAT}', ts, r['t_wall'], r['t_wall'] + r['set_s'])
            except Exception as e:   # recorded in calls + meta, experiment aborted below
                af_abort = f'{type(e).__name__}: {e}'
                verify = 0.0

            t_wall = time.time()
            row, errors, read_s = read_all(comps, sources, err_counts)
            trips, sig = safety.check(row, wo_done, seg)
            af_now = row['af'][0]
            af_match = (float(af_equal(af_now, expected)[0]) if all(row['af'][1][f] for f in io.AF_FIELDS)
                        else math.nan)
            rec.add_scalar('i', i)
            rec.add_scalar('t_nominal', ts)
            rec.add_scalar('t_wall', t_wall)
            rec.add_scalar('t_mono', time.monotonic())
            rec.add_scalar('read_s', read_s)
            rec.add_scalar('late_s', (t_wall - t0) - ts)
            rec.add_scalar('wp_idx', wp_idx)
            rec.add_scalar('k_idx', k_idx)
            rec.add_scalar('k_target_x', k_target)
            rec.add_scalar('af_call.kind', call_kind)
            rec.add_scalar('af_call.set_s', set_s)
            rec.add_scalar('af_call.get_s', get_s)
            rec.add_scalar('af_call.verify', verify)
            rec.add_scalar('af_match', af_match)
            rec.add_scalar('phynt_log_bytes', file_size(PHYNT_TMP_LOG) or -1)
            for k, v in sig.items():
                rec.add_scalar(f'safety.{k}', v)
            for src, sample in row.items():
                rec.add(src, sample)
                rec.add_scalar(f'{src}.error', errors[src])

            if i % 50 == 0:
                p = row['pom'][0]['pos']; f = row['ext_wrench'][0]['force']
                print(f't {ts:5.1f} pos [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] ext_f [{f[0]:.2f} {f[1]:.2f} {f[2]:.2f}] '
                      f"K_x {k_target:g} af.K_x {af_now['K'][0]:g} f_cmd_xy {sig['f_cmd_xy']:.2f} "
                      f"e_seg {sig['e_seg']:.3f} w [{sig['w_min']:.1f},{sig['w_max']:.1f}] late {(t_wall - t0) - ts:.3f}",
                      flush=True)

            if af_abort is not None:
                trips = trips + [f'set_af_parameters: {af_abort}']
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
            meta['n_af_calls'] = len(calls)
            meta['tmp_phynt_log_bytes_after_stop'] = file_size(PHYNT_TMP_LOG)
            meta['out_phynt_log_bytes'] = file_size(os.path.join(OUT_DIR, 'phynt.log'))
            jdump(meta, 'meta.json', OUT_DIR)
            jdump(events, 'events.json', OUT_DIR)
            jdump(snaps, 'snapshots.json', OUT_DIR)
            jdump(calls, 'k_calls.json', OUT_DIR)
            arrays = rec.to_arrays() if rec.cols else {}
            np.savez(os.path.join(OUT_DIR, NPZ_NAME), **arrays,
                     meta_json=json.dumps(meta, default=str),
                     events_json=json.dumps(events, default=str),
                     snapshots_json=json.dumps(snaps, default=str),
                     k_calls_json=json.dumps(calls, default=str))
            print(f'saved {len(arrays.get("i", []))} samples, {len(calls)} set_af calls to {OUT_DIR}; '
                  f'abort={meta["abort"]}', flush=True)
    return 1 if meta['abort'] else 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='setup+start+preflight+AF round trip+stop, no flight')
    a = ap.parse_args(argv)
    return run_check() if a.check else run_flight()


if __name__ == '__main__':
    sys.exit(main())
