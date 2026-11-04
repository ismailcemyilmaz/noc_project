"""
t04_ft_check — real contact force from the Gazebo force_torque sensor on ee_joint
(noc-tilthex in noc-wall-world, started by noc/experiments/sim_noc.sh).
Plan: noc/config/t04_ft_plan.md; gate B1 criteria (committed before this script):
noc/config/t04_ft_criteria.json.

noc defaults: setup_06a.setup (rotor_measure fix), flight.goto (ack=True), flight.Safety v2
(SAFETY unchanged), telekyb_io readers; src/06a-* is not modified. AF is never changed
(K_x = 10): the AF read after setup must equal af_on_baseline.json and get_af_parameters is
read back every cycle (as t02/t03). phynt_des / ext_wrench are pending until the first goto
(t03 rule).

The F/T stream is recorded by noc/experiments/log_ft.sh, started by this script (own
process group) into <out>/ft.log before setup and stopped after stop(). Takeoff is refused
if fewer than FT_MIN_MSGS messages arrive within FT_WAIT_S after start().

Scenario (nominal s):
   2  goto (0, 0, 1) 5 s          takeoff
  10  goto (1.2, 0, 1) 4 s        approach; tip surface x ~ 1.84 (16 cm from the wall)
  20  phynt.set_wo_zero(2 s)
  26-34  FREE1   free hover: F/T bias
  34  goto (1.55, 0, 1) 2 s       contact
  44-54  CONTACT steady contact
  54  goto (1.2, 0, 1) 3 s        retreat
  60-68  FREE2   free hover after the retreat
  68  goto (0, 0, 0) 5 s          land
  76  stop()
At the end of each window the F/T mean/std (child frame, host-time window) is computed
from ft.log and printed / stored in meta['ft_live'] (informational; a04 is authoritative).

Run inside the container (T2), after sim_noc.sh is up and Gazebo Play pressed:
  check only, no flight:  cd /shared-workspace && python3 -m noc.tests.t04_ft_check --check
  flight:                 cd /shared-workspace && python3 -m noc.tests.t04_ft_check
Output: logs/noc/t04_ft_check/ (flight) or logs/noc/t04_check/ (check).
Exit code: 0 = completed, 1 = check failed or experiment aborted.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import traceback

import numpy as np

from noc.common import setup_06a
from noc.common import telekyb_io as io
from noc.common import flight
from noc.common import ft_log
from noc.common.flight import (SAFETY, Safety, components, read_all, jdump, file_size,
                               git_rev, snapshot, make_ev, retreat, PHYNT_TMP_LOG)
from noc.tests.t02_live_k import (af_baseline, af_equal, check_initial_af, check_contact_x,
                                  sources_for, X_CONTACT)
from noc.tests.t03_k_ramp import Pending, classify_preflight, PENDING_SOURCES

REPO = setup_06a.REPO
OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't04_ft_check')
CHECK_DIR = os.path.join(REPO, 'logs', 'noc', 't04_check')
CONFIG = os.path.join(REPO, 'noc', 'config', 'af_on_baseline.json')
CRITERIA = os.path.join(REPO, 'noc', 'config', 't04_ft_criteria.json')
LOG_FT_SH = os.path.join(REPO, 'noc', 'experiments', 'log_ft.sh')
NPZ_NAME = 't04_ft_check.npz'
FT_NAME = 'ft.log'

DT = 0.02
T_WO_ZERO = 20.0
X_FREE = 1.2
WAYPOINTS = [
    (2.0, (0.0, 0.0, 1.0, 0, 5.0)),
    (10.0, (X_FREE, 0.0, 1.0, 0, 4.0)),
    (34.0, (X_CONTACT, 0.0, 1.0, 0, 2.0)),   # contact
    (54.0, (X_FREE, 0.0, 1.0, 0, 3.0)),      # retreat
    (68.0, (0.0, 0.0, 0.0, 0, 5.0)),         # land
]
WINDOWS = {'FREE1': (26.0, 34.0), 'CONTACT': (44.0, 54.0), 'FREE2': (60.0, 68.0)}
TF = 76.0
SNAPSHOT_TIMES = {'free1': 30.0, 'contact': 50.0, 'free2': 64.0}
FT_WAIT_S = 3.0
FT_MIN_MSGS = 50
CHECK_GROUND_S = 2.0


# ---------------------------------------------------------------- F/T logger process
class FtLogger:
    """log_ft.sh in its own process group writing ``path``; ``tail`` reads it."""

    def __init__(self, path, cmd=None):
        self.path = path
        self.cmd = cmd or ['sh', LOG_FT_SH, path]
        self.proc = None
        self.tail = ft_log.FtTail(path)
        self.info = dict(cmd=self.cmd, path=path)

    def start(self):
        self.proc = subprocess.Popen(self.cmd, start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.info.update(pid=self.proc.pid, started=time.time())

    def wait_stream(self, n_min, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.tail.poll()
            if self.tail.n >= n_min:
                break
            if self.proc is not None and self.proc.poll() is not None:
                break
            time.sleep(0.05)
        self.tail.poll()
        res = dict(n=self.tail.n, waited_s=time.time() - t0, n_min=n_min, timeout_s=timeout,
                   ok=self.tail.n >= n_min, logger_exit=None if self.proc is None else self.proc.poll())
        self.info['stream_check'] = res
        return res

    def stop(self):
        if self.proc is None:
            return
        alive = self.proc.poll() is None
        self.info['alive_at_stop'] = alive
        if alive:
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        try:
            _, err = self.proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
            _, err = self.proc.communicate()
        self.info.update(returncode=self.proc.returncode, stopped=time.time(),
                         stderr=(err or b'').decode(errors='replace')[-2000:])
        self.tail.poll()
        self.info.update(n_messages=self.tail.n, n_bad=self.tail.n_bad, bad_examples=self.tail.bad_examples)


def ft_summary(A_stats, label):
    f = [A_stats[k]['mean'] if A_stats.get(k) else math.nan for k in ('fx', 'fy', 'fz')]
    s = [A_stats[k]['std'] if A_stats.get(k) else math.nan for k in ('fx', 'fy', 'fz')]
    print(f'[ft {label}] n {A_stats["n"]} F mean [{f[0]:+.4f} {f[1]:+.4f} {f[2]:+.4f}] '
          f'std [{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}] N (child frame)', flush=True)


def ft_window(logger, a, b):
    logger.tail.poll()
    if logger.tail.n == 0:
        return {'n': 0}
    return ft_log.window_stats(logger.tail.arrays(), a, b)


# ---------------------------------------------------------------- check mode
def run_check(load=setup_06a.load_06a, make_logger=FtLogger):
    os.makedirs(CHECK_DIR, exist_ok=True)
    lg = make_logger(os.path.join(CHECK_DIR, FT_NAME))
    lg.start()
    m = load(CHECK_DIR)
    comps = components(m)
    sources = sources_for(m.phynt)
    res = {'sources': io.describe_sources(sources), 'model': ft_log.model_info()}
    bad, af_ok, ft_ok = ['preflight not run'], False, False
    try:
        check_contact_x(m)
        res['setup_06a'] = setup_06a.setup(m)
        m.start()
        time.sleep(1.0)
        res['preflight'] = io.preflight(comps, sources)
        bad = classify_preflight(res['preflight'])
        res['ft_stream'] = lg.wait_stream(FT_MIN_MSGS, FT_WAIT_S)
        ft_ok = res['ft_stream']['ok']
        w1 = time.time(); time.sleep(CHECK_GROUND_S)
        g = ft_window(lg, w1, time.time())
        res['ft_ground'] = g
        if g['n'] > 1:
            ft_summary(g, 'ground')
            fn = float(np.linalg.norm([g[k]['mean'] for k in ('fx', 'fy', 'fz')]))
            w = res['model']['ee_mass_kg'] * 9.81
            res['ft_ground_weight_check'] = dict(norm_N=fn, expected_N=w, rel_err=(fn - w) / w,
                                                 within_20pct=abs(fn - w) <= 0.2 * w)
            print(f'[check] F/T on ground |F| = {fn:.4f} N, m_ee g = {w:.4f} N '
                  f'({100 * (fn - w) / w:+.1f} %, reported only)', flush=True)
        res['snapshot'] = snapshot(m, 'check', 0.0)
        _, res['initial_af'] = check_initial_af(m, af_baseline(CONFIG))
        af_ok = True
    except Exception as e:
        res['error'] = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        print(f'[check] ERROR {type(e).__name__}: {e}', flush=True)
    finally:
        try:
            m.stop()
        finally:
            lg.stop()
            res['ft_logger'] = lg.info
            jdump(res, 'check.json', CHECK_DIR)
    ok = not bad and af_ok and ft_ok
    print(f'[check] non-pending reads {"ALL OK" if not bad else "FAILED " + str(bad)}; '
          f'F/T stream {"OK" if ft_ok else "FAILED"} ({lg.info.get("n_messages")} msgs, '
          f'{lg.info.get("n_bad")} bad); initial AF {"OK" if af_ok else "FAILED"}', flush=True)
    return 0 if ok else 1


# ---------------------------------------------------------------- flight
def run_flight(load=setup_06a.load_06a, make_logger=FtLogger):
    os.makedirs(OUT_DIR, exist_ok=True)
    lg = make_logger(os.path.join(OUT_DIR, FT_NAME))
    lg.start()
    m = load(OUT_DIR)
    comps = components(m)
    sources = sources_for(m.phynt)
    rec = io.Recorder()
    events, snaps = [], {}
    ev = make_ev(events)
    safety = Safety(SAFETY)
    pend = Pending()
    meta = dict(git_rev=git_rev(), model_06a=setup_06a.MODEL_06A, tf=TF, dt=DT, t_wo_zero=T_WO_ZERO,
                waypoints=WAYPOINTS, windows=WINDOWS, snapshot_times=SNAPSHOT_TIMES,
                pending_sources=list(PENDING_SOURCES), safety=SAFETY, safety_version=flight.SAFETY_VERSION,
                sources=io.describe_sources(sources), goto_mode='ack=True', config=CONFIG, criteria=CRITERIA,
                model=ft_log.model_info(), ft_wait_s=FT_WAIT_S, ft_min_msgs=FT_MIN_MSGS,
                n_af_set_calls=0, seed=None, seed_note='no RNG in this script; simulator has no recorded seed',
                started_unix=time.time(), abort=None, ft_live={})
    error = None
    err_counts, pend_counts, all_counts = {}, {}, {}
    try:
        check_contact_x(m)
        w0 = time.time(); meta['setup_06a'] = setup_06a.setup(m); ev('setup', -1.0, w0, time.time())
        base, meta['initial_af'] = check_initial_af(m, af_baseline(CONFIG))
        w0 = time.time(); m.start(); ev('start', -1.0, w0, time.time())
        meta['preflight'] = io.preflight(comps, sources)
        bad = classify_preflight(meta['preflight'])
        jdump(meta['preflight'], 'preflight.json', OUT_DIR)
        if bad:
            raise RuntimeError(f'preflight failed for non-pending sources {bad}; no takeoff')
        meta['ft_stream'] = lg.wait_stream(FT_MIN_MSGS, FT_WAIT_S)
        print(f'[ft] stream check: {meta["ft_stream"]}', flush=True)
        if not meta['ft_stream']['ok']:
            raise RuntimeError(f'F/T stream: {meta["ft_stream"]["n"]} msgs < {FT_MIN_MSGS} within '
                               f'{FT_WAIT_S} s (logger exit {meta["ft_stream"]["logger_exit"]}); no takeoff')
        pend.seed(meta['preflight'])
        snaps['after_start'] = snapshot(m, 'after_start', 0.0)

        p0 = io.read_pom(m.pom)[0]['pos']
        seg = (p0.copy(), p0.copy())
        N = int(round(TF / DT))
        wo_done = False
        wp_idx = 0
        win_state = {k: None for k in WINDOWS}      # None -> started (wall) -> done
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
            for name, (a, b) in WINDOWS.items():
                st = win_state[name]
                if st is None and ts >= a:
                    win_state[name] = time.time()
                    ev(f'window {name} start', ts, win_state[name], win_state[name])
                elif isinstance(st, float) and ts >= b:
                    w1 = time.time()
                    ev(f'window {name} end', ts, w1, w1)
                    s = ft_window(lg, st, w1)
                    s.update(wall=(st, w1))
                    meta['ft_live'][name] = s
                    if s['n'] > 1:
                        ft_summary(s, name)
                        if name != 'FREE1' and meta['ft_live'].get('FREE1', {}).get('n', 0) > 1:
                            b1 = meta['ft_live']['FREE1']
                            d = [s[k]['mean'] - b1[k]['mean'] for k in ('fx', 'fy', 'fz')]
                            print(f'[ft {name}] minus FREE1 bias: dF [{d[0]:+.4f} {d[1]:+.4f} {d[2]:+.4f}] N '
                                  '(child frame, live, informational)', flush=True)
                    win_state[name] = 'done'
            for lbl, t_snap in list(pending.items()):
                if ts >= t_snap:
                    snaps[lbl] = snapshot(m, lbl, ts)
                    del pending[lbl]

            t_wall = time.time()
            row, errors, read_s = read_all(comps, sources, all_counts)
            for s_, e in errors.items():
                if e:
                    tgt = pend_counts if pend.is_pending(s_) else err_counts
                    tgt[s_] = tgt.get(s_, 0) + 1
            pend_flags = {s_: pend.is_pending(s_) for s_ in PENDING_SOURCES}
            pend_trip = pend.update(row, errors, time.time())
            trips, sig = safety.check(row, wo_done, seg)
            af_now = row['af'][0]
            af_match = (float(af_equal(af_now, base)[0]) if all(row['af'][1][f] for f in io.AF_FIELDS)
                        else math.nan)
            lg.tail.poll()
            last = lg.tail.rows[-1] if lg.tail.n else (math.nan,) * len(ft_log.COLS)
            rec.add_scalar('i', i)
            rec.add_scalar('t_nominal', ts)
            rec.add_scalar('t_wall', t_wall)
            rec.add_scalar('t_mono', time.monotonic())
            rec.add_scalar('read_s', read_s)
            rec.add_scalar('late_s', (t_wall - t0) - ts)
            rec.add_scalar('wp_idx', wp_idx)
            rec.add_scalar('af_match', af_match)
            rec.add_scalar('phynt_log_bytes', file_size(PHYNT_TMP_LOG) or -1)
            rec.add_scalar('ft.n', lg.tail.n)
            for c, v in zip(ft_log.COLS, last):
                rec.add_scalar(f'ft.last.{c}', v)
            for s_, v in pend_flags.items():
                rec.add_scalar(f'pending.{s_}', v)
            for k, v in sig.items():
                rec.add_scalar(f'safety.{k}', v)
            for src, sample in row.items():
                rec.add(src, sample)
                rec.add_scalar(f'{src}.error', errors[src])

            if i % 50 == 0:
                p = row['pom'][0]['pos']; f = row['ext_wrench'][0]['force']
                print(f't {ts:5.1f} pos [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] ext_f [{f[0]:.2f} {f[1]:.2f} {f[2]:.2f}] '
                      f'ft_fx {last[2]:+.3f} ft_n {lg.tail.n} af.K_x {af_now["K"][0]:g} '
                      f"pending {[s_ for s_, v in pend_flags.items() if v]} f_cmd_xy {sig['f_cmd_xy']:.2f} "
                      f"e_seg {sig['e_seg']:.3f} w [{sig['w_min']:.1f},{sig['w_max']:.1f}] "
                      f'late {(t_wall - t0) - ts:.3f}', flush=True)

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
            lg.stop()
            meta['ft_logger'] = lg.info
            meta['finished_unix'] = time.time()
            meta['error'] = error
            meta['read_error_counts'] = err_counts
            meta['read_error_counts_pending'] = pend_counts
            meta['read_error_counts_all'] = all_counts
            meta['pending'] = pend.summary()
            meta['tmp_phynt_log_bytes_after_stop'] = file_size(PHYNT_TMP_LOG)
            meta['out_phynt_log_bytes'] = file_size(os.path.join(OUT_DIR, 'phynt.log'))
            jdump(meta, 'meta.json', OUT_DIR)
            jdump(events, 'events.json', OUT_DIR)
            jdump(snaps, 'snapshots.json', OUT_DIR)
            arrays = rec.to_arrays() if rec.cols else {}
            np.savez(os.path.join(OUT_DIR, NPZ_NAME), **arrays,
                     meta_json=json.dumps(meta, default=str),
                     events_json=json.dumps(events, default=str),
                     snapshots_json=json.dumps(snaps, default=str))
            print(f'saved {len(arrays.get("i", []))} samples, F/T {lg.info.get("n_messages")} msgs '
                  f'({lg.info.get("n_bad")} bad) to {OUT_DIR}; abort={meta["abort"]}', flush=True)
    return 1 if meta['abort'] else 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='setup+start+preflight+F/T stream+stop, no flight')
    a = ap.parse_args(argv)
    return run_check() if a.check else run_flight()


if __name__ == '__main__':
    sys.exit(main())
