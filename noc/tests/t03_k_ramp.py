"""
t03_k_ramp — K_x of the phynt AF ramped (not stepped) during steady wall contact.
Gate A2 criteria were committed before this run: noc/config/t03_k_ramp_criteria.json.

Built on the t02 infrastructure (AF helpers, set_and_verify, initial AF check from
noc.tests.t02_live_k; setup_06a / flight.goto(ack) / Safety v2 from noc.common).
src/06a-* is not modified.

Scenario (nominal time, s):
   2  goto (0, 0, 1)              takeoff, 5 s
  10  goto (1.2, 0, 1)            approach, 4 s
  20  phynt.set_wo_zero(2 s)      WO calibration in hover
  22  goto (BODY_X_CONTACT, 0, 1) contact, 2 s; held until the retreat
  30  K_x = 10 hold   (20 s)
  50  ramp 10 -> 20 at 10 N/m/s (1 s, one set_af_parameters per 50 Hz cycle, 0.2 N/m each)
  51  K_x = 20 hold   (20 s)
  71  ramp 20 -> 10   (1 s)
  72  K_x = 10 hold   (20 s)
  92  retreat goto (1.0, 0, 1), 4 s
  98  land goto (0, 0, 0), 5 s
 106  stop()

Only K[0] changes; AF mass, B, J and K[1:] stay at the values read after setup
(checked against noc/config/af_on_baseline.json before takeoff). Every call is timed
and read back; an error or mismatch aborts (retreat, land, stop).

Pending sources: phynt desired and external_wrench are not written by phynt before
the first reference (t02: frozen or EMPTY_POSTER until the first goto,
noc/results/02_live_k.md §1). Before the first goto they are recorded as 'pending'
(preflight and per cycle; their read errors go to read_error_counts_pending). After
the first goto each must become ready (no read error, all fields valid, ts newer than
the last ts seen before the goto) within PENDING_TIMEOUT_S, otherwise abort.
Any other preflight failure refuses takeoff.

Run inside the container (T2), after simulation.sh is up and Gazebo Play pressed:
  check only, no flight:  cd /shared-workspace && python3 -m noc.tests.t03_k_ramp --check
  flight:                 cd /shared-workspace && python3 -m noc.tests.t03_k_ramp
Output: logs/noc/t03_k_ramp/ (flight) or logs/noc/t03_check/ (check).
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
from noc.common.flight import (SAFETY, Safety, components, read_all, jdump, file_size,
                               git_rev, snapshot, make_ev, retreat, PHYNT_TMP_LOG)
from noc.tests.t02_live_k import (AFMismatch, af_baseline, af_equal, af_command, set_and_verify,
                                  check_initial_af, check_contact_x, sources_for, X_CONTACT,
                                  CALL_NONE, CALL_CHECK)

REPO = setup_06a.REPO
OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't03_k_ramp')
CHECK_DIR = os.path.join(REPO, 'logs', 'noc', 't03_check')
CONFIG = os.path.join(REPO, 'noc', 'config', 'af_on_baseline.json')
CRITERIA = os.path.join(REPO, 'noc', 'config', 't03_k_ramp_criteria.json')
NPZ_NAME = 't03_k_ramp.npz'

DT = 0.02
T_WO_ZERO = 20.0
WAYPOINTS = [
    (2.0, (0.0, 0.0, 1.0, 0, 5.0)),
    (10.0, (1.2, 0.0, 1.0, 0, 4.0)),
    (22.0, (X_CONTACT, 0.0, 1.0, 0, 2.0)),
    (92.0, (1.0, 0.0, 1.0, 0, 4.0)),     # retreat from the wall
    (98.0, (0.0, 0.0, 0.0, 0, 5.0)),     # land
]
T_SETTLED = 30.0
HOLD_S = 20.0
RAMP_RATE = 10.0          # N/m/s
K_SEQUENCE = [10.0, 20.0, 10.0]
TF = 106.0
SNAPSHOT_TIMES = {'settled': 30.0, 'before_ramp1': 49.0, 'after_ramp1': 53.0,
                  'before_ramp2': 70.0, 'after_ramp2': 74.0}
PENDING_SOURCES = ('phynt_des', 'ext_wrench')
PENDING_TIMEOUT_S = 2.0
K_DECIMALS = 6            # ramp values rounded so that read-back comparisons are exact

CALL_RAMP = 4
PHASE_NONE, PHASE_HOLD, PHASE_RAMP = 0, 1, 2


# ---------------------------------------------------------------- K profile
def k_profile(t_settled=None, hold_s=None, rate=None, seq=None):
    """Breakpoints [(t, K_x)] of the piecewise-linear schedule and the list of
    phases [(kind, t_start, t_end, k_start, k_end)]. Reads module globals at call
    time so tests can compress the timeline."""
    t_settled = T_SETTLED if t_settled is None else t_settled
    hold_s = HOLD_S if hold_s is None else hold_s
    rate = RAMP_RATE if rate is None else rate
    seq = K_SEQUENCE if seq is None else seq
    t = t_settled
    phases = []
    for j, k in enumerate(seq):
        if j:
            dur = abs(k - seq[j - 1]) / rate
            phases.append(('ramp', t, t + dur, seq[j - 1], k))
            t += dur
        phases.append(('hold', t, t + hold_s, k, k))
        t += hold_s
    pts = [(phases[0][1], phases[0][3])] + [(p[2], p[4]) for p in phases]
    return pts, phases


def k_at(ts, phases):
    """(K_x target, phase kind code, phase index) at nominal time ts; (None, NONE, -1) outside."""
    for j, (kind, a, b, k0, k1) in enumerate(phases):
        if a <= ts < b or (j == len(phases) - 1 and ts == b):
            if kind == 'hold':
                return k0, PHASE_HOLD, j
            u = (ts - a) / (b - a)
            return round(k0 + (k1 - k0) * u, K_DECIMALS), PHASE_RAMP, j
    if phases and ts >= phases[-1][2]:
        return phases[-1][4], PHASE_NONE, -1
    return None, PHASE_NONE, -1


# ---------------------------------------------------------------- pending sources
class Pending:
    """Readiness of phynt_des / ext_wrench after the first goto (see module doc)."""

    def __init__(self, sources=PENDING_SOURCES, timeout=None):
        self.sources = tuple(sources)
        self.timeout = PENDING_TIMEOUT_S if timeout is None else timeout
        self.ts_pre = {s: math.nan for s in self.sources}
        self.ready_wall = {s: None for s in self.sources}
        self.goto_wall = None

    def seed(self, preflight):
        for s in self.sources:
            v = (preflight.get(s) or {}).get('values') or {}
            t = v.get('ts')
            if t is not None and np.isfinite(t):
                self.ts_pre[s] = float(t)

    def is_pending(self, s):
        return s in self.sources and self.ready_wall[s] is None

    def update(self, row, errors, now):
        """Call once per cycle after read_all. Returns a trip reason or None."""
        for s in self.sources:
            if self.ready_wall[s] is not None:
                continue
            out, valid = row[s]
            if self.goto_wall is None:
                if not errors[s] and valid.get('ts'):
                    self.ts_pre[s] = float(out['ts'])
                continue
            fresh = math.isnan(self.ts_pre[s]) or (valid.get('ts') and out['ts'] > self.ts_pre[s])
            if not errors[s] and all(valid.values()) and fresh:
                self.ready_wall[s] = now
        if self.goto_wall is not None and now - self.goto_wall > self.timeout:
            late = [s for s in self.sources if self.ready_wall[s] is None]
            if late:
                return f'pending sources not ready {self.timeout:g} s after first goto: {late}'
        return None

    def summary(self):
        return dict(sources=list(self.sources), timeout_s=self.timeout, goto_wall=self.goto_wall,
                    ts_before_goto=self.ts_pre, ready_wall=self.ready_wall,
                    ready_after_goto_s={s: (None if (r is None or self.goto_wall is None) else r - self.goto_wall)
                                        for s, r in self.ready_wall.items()})


def classify_preflight(pf, pending_sources=PENDING_SOURCES):
    """Adds 'status' to every preflight entry: 'pending' for the pending sources
    (whatever the read gave), else 'ok' / 'fail'. Returns the failing non-pending sources."""
    bad = []
    for s, e in pf.items():
        if s in pending_sources:
            e['status'] = 'pending'
        else:
            e['status'] = 'ok' if e['ok'] else 'fail'
            if not e['ok']:
                bad.append(s)
    print(f'[preflight] pending (before first goto): {list(pending_sources)}; failing: {bad}', flush=True)
    return bad


# ---------------------------------------------------------------- check mode
def run_check(load=setup_06a.load_06a):
    os.makedirs(CHECK_DIR, exist_ok=True)
    m = load(CHECK_DIR)
    comps = components(m)
    sources = sources_for(m.phynt)
    res = {'sources': io.describe_sources(sources)}
    calls = []
    af_ok = False
    bad = ['preflight not run']
    try:
        check_contact_x(m)
        res['setup_06a'] = setup_06a.setup(m)
        m.start()
        time.sleep(1.0)
        res['preflight'] = io.preflight(comps, sources)
        bad = classify_preflight(res['preflight'])
        res['snapshot'] = snapshot(m, 'check', 0.0)
        read, res['initial_af'] = check_initial_af(m, af_baseline(CONFIG))
        set_and_verify(m.phynt, af_command(read, float(read['K'][0])), CALL_CHECK, 0.0, calls)
        # largest ramp value on the ground, then back: the K values the flight will send
        for kx in (max(K_SEQUENCE), float(read['K'][0])):
            set_and_verify(m.phynt, af_command(read, kx), CALL_CHECK, 0.0, calls)
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
    ok = not bad and af_ok
    print(f'[check] non-pending reads {"ALL OK" if not bad else "FAILED " + str(bad)}; '
          f'pending: {list(PENDING_SOURCES)}; af round trips: {af_ok} ({len(calls)} calls)', flush=True)
    return 0 if ok else 1


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
    pend = Pending()
    k_pts, phases = k_profile()
    meta = dict(git_rev=git_rev(), model_06a=setup_06a.MODEL_06A, tf=TF, dt=DT, t_wo_zero=T_WO_ZERO,
                waypoints=WAYPOINTS, k_sequence=K_SEQUENCE, hold_s=HOLD_S, ramp_rate=RAMP_RATE,
                t_settled=T_SETTLED, k_breakpoints=k_pts, k_phases=phases, snapshot_times=SNAPSHOT_TIMES,
                pending_sources=list(PENDING_SOURCES), pending_timeout_s=PENDING_TIMEOUT_S,
                safety=SAFETY, safety_version=flight.SAFETY_VERSION, sources=io.describe_sources(sources),
                goto_mode='ack=True', config=CONFIG, criteria=CRITERIA, seed=None,
                seed_note='no RNG in this script; simulator has no recorded seed',
                started_unix=time.time(), abort=None)
    error = None
    err_counts, pend_counts, all_counts = {}, {}, {}
    try:
        check_contact_x(m)
        w0 = time.time(); meta['setup_06a'] = setup_06a.setup(m); ev('setup', -1.0, w0, time.time())
        base_read, meta['initial_af'] = check_initial_af(m, af_baseline(CONFIG))
        base = af_command(base_read, float(base_read['K'][0]))
        if base['K'][0] != K_SEQUENCE[0]:
            raise AFMismatch(f'initial K_x {base["K"][0]} != K_SEQUENCE[0] {K_SEQUENCE[0]}')
        w0 = time.time(); m.start(); ev('start', -1.0, w0, time.time())
        meta['preflight'] = io.preflight(comps, sources)
        bad = classify_preflight(meta['preflight'])
        jdump(meta['preflight'], 'preflight.json', OUT_DIR)
        if bad:
            raise RuntimeError(f'preflight failed for non-pending sources {bad}; no takeoff')
        pend.seed(meta['preflight'])
        snaps['after_start'] = snapshot(m, 'after_start', 0.0)

        p0 = io.read_pom(m.pom)[0]['pos']
        seg = (p0.copy(), p0.copy())
        N = int(round(TF / DT))
        wo_done = False
        wp_idx = 0
        k_target = float(base['K'][0])
        expected = base
        last_phase = -1
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
                if pend.goto_wall is None:
                    pend.goto_wall = w0
                seg = (seg[1], np.array(args[:3], float))
                wp_idx += 1
            for lbl, t_snap in list(pending.items()):
                if ts >= t_snap:
                    snaps[lbl] = snapshot(m, lbl, ts)
                    del pending[lbl]

            call_kind, set_s, get_s, verify = CALL_NONE, math.nan, math.nan, math.nan
            af_abort = None
            kx, phase, ph_idx = k_at(ts, phases)
            if ph_idx != last_phase and ph_idx >= 0:
                kind, a, b, k0, k1 = phases[ph_idx]
                ev(f'{kind} {k0:g}->{k1:g}' if kind == 'ramp' else f'hold K_x={k0:g}', ts, time.time(), time.time())
                last_phase = ph_idx
            try:
                if kx is not None and kx != k_target:
                    expected = af_command(base, kx)
                    call_kind = CALL_RAMP
                    r = set_and_verify(m.phynt, expected, call_kind, ts, calls)
                    r['phase'] = ph_idx
                    k_target = kx
                    set_s, get_s, verify = r['set_s'], r['get_s'], 1.0
            except Exception as e:   # recorded in calls + meta, experiment aborted below
                af_abort = f'{type(e).__name__}: {e}'
                verify = 0.0

            t_wall = time.time()
            row, errors, read_s = read_all(comps, sources, all_counts)
            for s, e in errors.items():
                if e:
                    tgt = pend_counts if pend.is_pending(s) else err_counts
                    tgt[s] = tgt.get(s, 0) + 1
            pend_flags = {s: pend.is_pending(s) for s in PENDING_SOURCES}
            pend_trip = pend.update(row, errors, time.time())
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
            rec.add_scalar('k_phase', phase)
            rec.add_scalar('k_phase_idx', ph_idx)
            rec.add_scalar('k_target_x', k_target)
            rec.add_scalar('af_call.kind', call_kind)
            rec.add_scalar('af_call.set_s', set_s)
            rec.add_scalar('af_call.get_s', get_s)
            rec.add_scalar('af_call.verify', verify)
            rec.add_scalar('af_match', af_match)
            rec.add_scalar('phynt_log_bytes', file_size(PHYNT_TMP_LOG) or -1)
            for s, v in pend_flags.items():
                rec.add_scalar(f'pending.{s}', v)
            for k, v in sig.items():
                rec.add_scalar(f'safety.{k}', v)
            for src, sample in row.items():
                rec.add(src, sample)
                rec.add_scalar(f'{src}.error', errors[src])

            if i % 50 == 0:
                p = row['pom'][0]['pos']; f = row['ext_wrench'][0]['force']
                print(f't {ts:5.1f} pos [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] ext_f [{f[0]:.2f} {f[1]:.2f} {f[2]:.2f}] '
                      f"K_x {k_target:g} af.K_x {af_now['K'][0]:g} pending {[s for s, v in pend_flags.items() if v]} "
                      f"f_cmd_xy {sig['f_cmd_xy']:.2f} e_seg {sig['e_seg']:.3f} "
                      f"w [{sig['w_min']:.1f},{sig['w_max']:.1f}] late {(t_wall - t0) - ts:.3f}", flush=True)

            if af_abort is not None:
                trips = trips + [f'set_af_parameters: {af_abort}']
            if pend_trip is not None:
                trips = trips + [pend_trip]
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
            meta['read_error_counts_pending'] = pend_counts
            meta['read_error_counts_all'] = all_counts
            meta['pending'] = pend.summary()
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
    ap.add_argument('--check', action='store_true', help='setup+start+preflight+AF round trips+stop, no flight')
    a = ap.parse_args(argv)
    return run_check() if a.check else run_flight()


if __name__ == '__main__':
    sys.exit(main())
