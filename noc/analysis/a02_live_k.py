"""
a02_live_k — analysis of the t02 live K_x run (noc/tests/t02_live_k.py).

Sections (all forces are estimates, never contact-force measurements):
  1. set_af_parameters timing (perf_counter, k_calls): mean, p95, max for the 200
     unchanged-value repeats and for the 5 K_x changes; get_af_parameters read-back time.
  2. Servo interruption / jumps. Servo activity proxy: uavpos.log row cadence (≈1 kHz).
     - interruption: a uavpos.log gap > SERVO_STOP_S inside a call window.
       (t01 steady contact: gaps up to 13 ms, whole flight up to 75 ms.)
     - jump: max per-row |Δxd| or |Δfx| in a call window > JUMP_FACTOR × the same
       statistic in the reference window (K_x = 10 hold, before the repeats).
     Call windows: each K_x change ±CALL_WIN_S, and the repeat block.
  3. Applied K: every k_calls entry verified, per-cycle af_match and af.K_x == target.
  4. Per K_x window (one hold each): steady part = last STEADY_FRAC of the hold.
     F = uavpos fx [COMPUTED quasi-static: f_ext,x ≈ -fx], WO f_ext,x (phynt.log efx) [LOG estimate],
     AF deviation = uavpos xd - maneuver x [LOG], d0 measured = maneuver x - pom x.
     Settling time: AF deviation (moving average MA_S) enters and stays in
     ±max(SETTLE_REL·|step|, SETTLE_ABS_M) of its steady mean; first window measured from
     the end of the contact goto, the others from the set_af_parameters call.
  5. Prediction F = d0 / (1/Kpxy + 1/K_x), dev = F / K_x for d0 = D0_GIVEN and for the
     measured d0; k_eff = F / d0_measured compared with the series value and Kpxy.
  Gate A: servo not interrupted and no jump; repeat p95 < P95_MAX_S; applied K verified;
     every window F within ±F_TOL of the prediction with D0_GIVEN.

Usage: .venv/bin/python -m noc.analysis.a02_live_k [log_dir]
Writes noc/results/02_live_k/{metrics.json, live_k.png}
"""

import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis.logs import read_dir
from noc.analysis.baseline_06a import stats, sha256

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs', 'noc', 't02_live_k')
OUT_DIR = os.path.join(REPO, 'noc', 'results', '02_live_k')
NPZ_NAME = 't02_live_k.npz'

KPXY = 5.0                # 06a uavpos Kpxy (af_on_baseline.json)
D0_GIVEN = 0.184          # m, measured in t01 (01_af_on.md §4)
CALL_SCHEDULE, CALL_REPEAT = 1, 2

SERVO_STOP_S = 0.10
JUMP_FACTOR = 3.0
CALL_WIN_S = 1.0
STEADY_FRAC = 0.5
MA_S = 0.5
SETTLE_REL = 0.10
SETTLE_ABS_M = 0.002
P95_MAX_S = 0.020
F_TOL = 0.15


def predict(kx, d0=D0_GIVEN, kp=KPXY):
    F = d0 / (1.0 / kp + 1.0 / kx)
    return dict(F_N=F, dev_m=F / kx, k_series=1.0 / (1.0 / kp + 1.0 / kx))


def tstats(x):
    x = np.asarray([v for v in x if v is not None], float)
    if x.size == 0:
        return None
    return dict(n=int(x.size), mean_ms=float(x.mean() * 1e3), p95_ms=float(np.percentile(x, 95) * 1e3),
                max_ms=float(x.max() * 1e3), min_ms=float(x.min() * 1e3), median_ms=float(np.median(x) * 1e3))


def moving_average(t, y, win):
    """Centered moving average over ``win`` seconds (irregular samples)."""
    if t.size == 0:
        return y
    c = np.concatenate([[0.0], np.cumsum(y)])
    lo = np.searchsorted(t, t - win / 2)
    hi = np.searchsorted(t, t + win / 2, side='right')
    return (c[hi] - c[lo]) / np.maximum(hi - lo, 1)


def settling_time(t, y_ma, t_start, t_end, final, step):
    """Time from t_start until y_ma stays within the band around final up to t_end
    (end of this K_x hold); None if it never does."""
    band = max(SETTLE_REL * abs(step), SETTLE_ABS_M)
    m = (t >= t_start) & (t <= t_end)
    tt, yy = t[m], y_ma[m]
    if tt.size == 0:
        return None, band
    out = np.abs(yy - final) > band
    if not out.any():
        return 0.0, band
    last = np.nonzero(out)[0][-1]
    if last + 1 >= tt.size:
        return None, band
    return float(tt[last + 1] - t_start), band


def window_gaps(ts, a, b):
    """Row gaps touching [a, b]: includes the last row before a and the first after b,
    so a gap spanning the window edge (or the whole window) is seen."""
    lo = max(int(np.searchsorted(ts, a)) - 1, 0)
    hi = int(np.searchsorted(ts, b, side='right')) + 1
    n_in = int(((ts >= a) & (ts <= b)).sum())
    d = np.diff(ts[lo:hi])
    if d.size == 0:
        return dict(n_rows=n_in, max_gap_ms=None, p99_gap_ms=None)
    return dict(n_rows=n_in, max_gap_ms=float(d.max() * 1e3), p99_gap_ms=float(np.percentile(d, 99) * 1e3))


def max_step(ts, y, a, b):
    m = (ts >= a) & (ts <= b)
    d = np.abs(np.diff(y[m]))
    d = d[np.isfinite(d)]
    return float(d.max()) if d.size else None


def load(log_dir):
    d = dict(np.load(os.path.join(log_dir, NPZ_NAME), allow_pickle=False))
    meta = json.loads(str(d.pop('meta_json')))
    events = json.loads(str(d.pop('events_json')))
    snaps = json.loads(str(d.pop('snapshots_json')))
    calls = json.loads(str(d.pop('k_calls_json')))
    return d, meta, events, snaps, calls, read_dir(log_dir, names=('pom', 'uavpos', 'maneuver', 'phynt'))


def k_windows(meta, events, calls, d):
    """[(K_x, t_start_wall, t_end_wall, kind)] one per hold, from the run's own events."""
    starts = []
    for e in events:
        n = e['name']
        if n.startswith('hold K_x=') or n.startswith('set_af K_x='):
            starts.append((float(n.split('=')[1]), e['wall_start']))
    rep = [c['t_wall'] for c in calls if c['kind'] == CALL_REPEAT]
    t_end = rep[0] if rep else float(d['t_wall'][-1])
    out = []
    for j, (k, a) in enumerate(starts):
        b = starts[j + 1][1] if j + 1 < len(starts) else t_end
        out.append((k, a, b))
    return out


def analyse(log_dir=LOG_DIR):
    d, meta, events, snaps, calls, L = load(log_dir)
    R = dict(meta={k: meta.get(k) for k in ('git_rev', 'abort', 'error', 'read_error_counts', 'k_schedule',
                                            'n_repeat', 'hold_s', 'safety_version', 'initial_af')})
    R['inventory'] = {f: dict(bytes=os.path.getsize(os.path.join(log_dir, f)),
                              sha256=sha256(os.path.join(log_dir, f))) for f in sorted(os.listdir(log_dir))
                      if os.path.isfile(os.path.join(log_dir, f))}
    missing = [n for n, v in L.items() if v is None or v.n == 0]
    R['missing_logs'] = missing
    up, mv, pom, ph = L['uavpos'], L['maneuver'], L['pom'], L['phynt']

    # 1) timing
    rep = [c for c in calls if c['kind'] == CALL_REPEAT]
    sch = [c for c in calls if c['kind'] == CALL_SCHEDULE]
    R['timing'] = dict(repeat_set=tstats([c['set_s'] for c in rep]),
                       repeat_get=tstats([c['get_s'] for c in rep]),
                       schedule_set=tstats([c['set_s'] for c in sch]),
                       schedule_calls=[dict(K_x=c['K_x'], set_ms=None if c['set_s'] is None else c['set_s'] * 1e3,
                                            get_ms=None if c['get_s'] is None else c['get_s'] * 1e3,
                                            verify=c['verify'], error=c['error']) for c in sch],
                       n_repeat=len(rep), n_schedule=len(sch))
    lat = d['late_s']
    R['loop'] = dict(dt_p99_ms=float(np.percentile(np.diff(d['t_wall']), 99) * 1e3),
                     late_max_s=float(np.nanmax(lat)), read_s=stats(d['read_s']))

    # 2) servo interruption / jumps
    wins = k_windows(meta, events, calls, d)
    R['k_windows_wall'] = [dict(K_x=k, start=a, end=b) for k, a, b in wins]
    servo = dict(proxy='uavpos.log row cadence', stop_threshold_ms=SERVO_STOP_S * 1e3, jump_factor=JUMP_FACTOR)
    ok_servo = None
    if up is not None and up.n and wins:
        k10 = wins[-1]   # last K_x = 10 hold, right before the repeats
        ref = (k10[1] + (k10[2] - k10[1]) * (1 - STEADY_FRAC), k10[2] - 0.05)
        servo['reference_window'] = dict(start=ref[0], end=ref[1], **window_gaps(up['ts'], *ref),
                                         max_dxd_m=max_step(up['ts'], up['xd'], *ref),
                                         max_dfx_N=max_step(up['ts'], up['fx'], *ref))
        cw = [(f'K_x->{c["K_x"]:g}', c['t_wall'] - CALL_WIN_S, c['t_wall'] + CALL_WIN_S) for c in sch]
        if rep:
            cw.append(('repeat', rep[0]['t_wall'] - 0.05, rep[-1]['t_wall'] + (rep[-1]['set_s'] or 0) + 0.05))
        rows, ok_servo = [], True
        for name, a, b in cw:
            g = window_gaps(up['ts'], a, b)
            sx, sf = max_step(up['ts'], up['xd'], a, b), max_step(up['ts'], up['fx'], a, b)
            rx, rf = servo['reference_window']['max_dxd_m'], servo['reference_window']['max_dfx_N']
            stop = g['max_gap_ms'] is None or g['max_gap_ms'] > SERVO_STOP_S * 1e3
            jump = bool((sx is not None and rx and sx > JUMP_FACTOR * rx) or
                        (sf is not None and rf and sf > JUMP_FACTOR * rf))
            rows.append(dict(window=name, start=a, end=b, **g, max_dxd_m=sx, max_dfx_N=sf,
                             interrupted=bool(stop), jump=jump))
            ok_servo = ok_servo and not stop and not jump
        servo['call_windows'] = rows
        if ph is not None and ph.n and rep:
            servo['phynt_log_repeat'] = window_gaps(ph['ts'], rep[0]['t_wall'], rep[-1]['t_wall'])
    R['servo'] = servo

    # 3) applied K
    am = d['af_match'][np.isfinite(d['af_match'])]
    afk = d['af.K'][:, 0]
    fin = np.isfinite(afk)
    R['applied_K'] = dict(calls=len(calls), calls_verified=int(sum(1 for c in calls if c['verify'])),
                          calls_failed=[c for c in calls if not c['verify']],
                          cycles_af_match=int((am == 1).sum()), cycles_af_read=int(am.size),
                          cycles_kx_equals_target=int((afk[fin] == d['k_target_x'][fin]).sum()),
                          read_errors_af=int(meta.get('read_error_counts', {}).get('af', 0)))
    ok_k = bool(calls) and R['applied_K']['calls_verified'] == len(calls) and am.size > 0 and bool((am == 1).all())

    # 4) + 5) per K window
    per = []
    if not missing:
        tu = up['ts']
        x_nom = np.interp(tu, mv['ts'], mv['x'])
        dev = up['xd'] - x_nom
        d0m = x_nom - np.interp(tu, pom['ts'], pom['x'])
        dev_ma = moving_average(tu, dev, MA_S)
        contact_end = None
        for e in events:
            a = e.get('args') or {}
            args = a.get('args') if isinstance(a, dict) else a
            if e['name'].startswith('maneuver.goto') and args and abs(args[0] - 1.55) < 1e-6:
                contact_end = e['wall_start'] + float(args[4])
        prev_final = 0.0
        for j, (k, a, b) in enumerate(wins):
            s0 = b - (b - a) * STEADY_FRAC
            su = (tu >= s0) & (tu <= b)
            sp = (ph['ts'] >= s0) & (ph['ts'] <= b)
            F = stats(up['fx'][su])
            dv = stats(dev[su])
            d0 = stats(d0m[su])
            final = dv['mean'] if dv else None
            t_set_from = contact_end if (j == 0 and contact_end) else a
            st, band = (settling_time(tu, dev_ma, t_set_from, b, final, final - prev_final)
                        if final is not None else (None, None))
            pg, pm = predict(k), (predict(k, d0=d0['mean']) if d0 else None)
            row = dict(K_x=k, start=a, end=b, steady=(s0, b), F_N=F, wo_efx_N=stats(ph['efx'][sp]),
                       af_dev_m=dv, d0_measured_m=d0, settling_s=st, settling_band_m=band,
                       settling_from=('contact goto end' if (j == 0 and contact_end) else 'set_af call'),
                       pred_d0_given=pg, pred_d0_measured=pm)
            if F and d0:
                row['k_eff'] = F['mean'] / d0['mean']
                row['k_eff_over_Kpxy'] = row['k_eff'] / KPXY
                row['F_err_rel_d0_given'] = (F['mean'] - pg['F_N']) / pg['F_N']
                row['F_err_rel_d0_measured'] = (F['mean'] - pm['F_N']) / pm['F_N']
                row['dev_err_m_d0_given'] = dv['mean'] - (-pg['dev_m'])
            per.append(row)
            if final is not None:
                prev_final = final
    R['per_K'] = per
    ok_F = bool(per) and all(r.get('F_err_rel_d0_given') is not None and abs(r['F_err_rel_d0_given']) <= F_TOL
                             for r in per)
    p95 = R['timing']['repeat_set']['p95_ms'] if R['timing']['repeat_set'] else None
    ok_t = p95 is not None and p95 < P95_MAX_S * 1e3 and R['timing']['n_repeat'] == meta.get('n_repeat', 200)
    complete = meta.get('abort') is None and meta.get('error') is None
    R['gate_A'] = dict(servo_uninterrupted=ok_servo, p95_lt_20ms=ok_t, applied_K_verified=ok_k,
                       F_within_15pct=ok_F, run_complete=complete,
                       passed=bool(ok_servo and ok_t and ok_k and ok_F and complete))
    return R, d, L, wins


def plot(R, d, L, wins, out):
    up, mv, ph = L['uavpos'], L['maneuver'], L['phynt']
    T0 = d['t_wall'][0]
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    t = d['t_wall'] - T0
    ax[0].step(t, d['k_target_x'], where='post', label='K_x target')
    ax[0].plot(t, d['af.K'][:, 0], '.', ms=2, label='get_af_parameters K_x')
    ax[0].set_ylabel('K_x [N/m]'); ax[0].legend()
    if up is not None and up.n and mv is not None and mv.n:
        tu = up['ts'] - T0
        ax[1].plot(tu, up['fx'], lw=0.5, label='uavpos fx [estimate]')
        if ph is not None and ph.n:
            ax[1].plot(ph['ts'] - T0, -ph['efx'], lw=0.5, alpha=0.7, label='-WO efx [estimate]')
        dev = up['xd'] - np.interp(up['ts'], mv['ts'], mv['x'])
        ax[2].plot(tu, dev * 1e3, lw=0.5, label='AF dev (xd - x_nom)')
    for r in R['per_K']:
        a, b = r['steady'][0] - T0, r['steady'][1] - T0
        ax[1].hlines(r['pred_d0_given']['F_N'], a, b, colors='k', linestyles='--')
        ax[2].hlines(-r['pred_d0_given']['dev_m'] * 1e3, a, b, colors='k', linestyles='--')
    ax[1].set_ylabel('F [N]'); ax[1].legend(); ax[1].set_ylim(-0.5, 2.0)
    ax[2].set_ylabel('dev [mm]'); ax[2].legend()
    ax[3].plot(t, d['af_call.set_s'] * 1e3, '.', ms=3, label='set_af_parameters [ms]')
    ax[3].plot(t, d['late_s'] * 1e3, lw=0.5, label='loop lateness [ms]')
    ax[3].set_ylabel('ms'); ax[3].set_xlabel('t [s] (wall, from first sample)'); ax[3].legend()
    fig.suptitle('t02 live K_x (dashed: prediction d0 = 0.184 m; forces are estimates)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'live_k.png'), dpi=120)
    plt.close(fig)


def main(log_dir=LOG_DIR, out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    R, d, L, wins = analyse(log_dir)
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(R, f, indent=1, default=float)
    plot(R, d, L, wins, out_dir)
    g = R['gate_A']
    print(json.dumps(R['timing'], indent=1, default=float))
    for r in R['per_K']:
        print(f"K_x {r['K_x']:5.1f}  F {r['F_N']['mean'] if r['F_N'] else float('nan'):.3f} N  "
              f"pred {r['pred_d0_given']['F_N']:.3f}  dev {r['af_dev_m']['mean'] * 1e3 if r['af_dev_m'] else float('nan'):.1f} mm  "
              f"settle {r['settling_s']}  k_eff {r.get('k_eff', float('nan')):.2f}")
    print('gate A:', g)
    return 0 if g['passed'] else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:2]))
