"""
a03_k_ramp — analysis of the t03 K_x ramp run and Gate A2.

Every gate threshold is read from noc/config/t03_k_ramp_criteria.json (committed before
the flight); nothing in this file redefines them. All forces are estimates, never
contact-force measurements:
  F = uavpos fx [COMPUTED quasi-static], WO efx (phynt.log) [LOG estimate].

Gate A2 (see the criteria file for the exact wording):
  C1 no abort, no error, no read error (pending-phase errors of phynt_des/ext_wrench excluded)
  C2 applied K_x == target in every cycle and every call verified
  C3 per ramp window: max per-row |dfx| <= max_N
  C4 over the K phase: max per-row |dxd| <= max_m
  C5 per hold: steady mean fx within tol of d0 / (1/Kpxy + 1/K_x)

Reported, not gated: settling time with a 2-sigma band (same method for t02 steps and
t03 ramps) and the t02 (steps) vs t03 (ramps) comparison.

Usage: .venv/bin/python -m noc.analysis.a03_k_ramp [t03_log_dir [t02_log_dir]]
Writes noc/results/03_k_ramp/{metrics.json, summary.md, k_ramp.png}
"""

import json
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis.logs import read_dir
from noc.analysis.baseline_06a import stats, sha256
from noc.analysis import a02_live_k as A2

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs', 'noc', 't03_k_ramp')
T02_DIR = os.path.join(REPO, 'logs', 'noc', 't02_live_k')
OUT_DIR = os.path.join(REPO, 'noc', 'results', '03_k_ramp')
CRITERIA = os.path.join(REPO, 'noc', 'config', 't03_k_ramp_criteria.json')
NPZ_NAME = 't03_k_ramp.npz'

CALL_RAMP = 4             # t03_k_ramp.CALL_RAMP
DT = 0.02                 # t03 loop period, used for the ramp window end (criteria: + dt)
RAMP_GAP_S = 0.5          # consecutive ramp calls further apart than this start a new ramp
MA_S = 0.5                # moving average for the AF deviation (criteria: 0.5 s)
M_AF, MASS = 5.0, 2.72    # AF virtual mass (af_on_baseline.json), drone mass (params.MASS)
T02_WIN_S = 1.0           # t02 step windows: call +/- 1 s (as a02)


def load_criteria(path=CRITERIA):
    with open(path) as f:
        c = json.load(f)
    r = subprocess.run(['git', '-C', REPO, 'log', '-1', '--format=%H %cI', '--', os.path.relpath(path, REPO)],
                       capture_output=True, text=True)
    return c, dict(path=os.path.relpath(path, REPO), sha256=sha256(path),
                   committed=r.stdout.strip() or None)


def predict_F(kx, crit):
    c5 = crit['criteria']['C5_steady_F_vs_series_model']
    return c5['d0_m'] / (1.0 / c5['Kpxy'] + 1.0 / kx)


# ---------------------------------------------------------------- generic helpers
def max_abs_step(ts, y, a, b):
    """max |y[i+1] - y[i]| over rows with a <= ts <= b; (value, ts of the later row, n_rows)."""
    m = (ts >= a) & (ts <= b)
    yy, tt = y[m], ts[m]
    if yy.size < 2:
        return None, None, int(yy.size)
    d = np.abs(np.diff(yy))
    ok = np.isfinite(d)
    if not ok.any():
        return None, None, int(yy.size)
    j = int(np.nanargmax(np.where(ok, d, -np.inf)))
    return float(d[j]), float(tt[j + 1]), int(yy.size)


def settling_2sigma(t, y_ma, t_from, t_end, steady):
    """Band = mean +/- 2 sigma of y_ma over the steady part (criteria amendment 1).
    Settling time = time from t_from to the first sample after the last out-of-band
    sample in [t_from, steady start). If y_ma is still out of band right before the
    steady part, the value is steady start - t_from and censored = True (lower bound).
    out_frac_steady: fraction of steady samples outside the band (~5 % for noise)."""
    s = (t >= steady[0]) & (t <= steady[1])
    if s.sum() < 2 or t_from is None:
        return dict(t_settle_s=None, censored=None, sigma_m=None, band_m=None, mean_m=None,
                    out_frac_steady=None)
    mean, sig = float(y_ma[s].mean()), float(y_ma[s].std())
    band = 2.0 * sig
    out_frac = float((np.abs(y_ma[s] - mean) > band).mean())
    m = (t >= t_from) & (t < steady[0])
    tt, yy = t[m], y_ma[m]
    out = np.abs(yy - mean) > band
    censored = False
    if tt.size == 0:
        ts = None
    elif not out.any():
        ts = 0.0
    else:
        last = int(np.nonzero(out)[0][-1])
        if last + 1 >= tt.size:
            ts, censored = float(steady[0] - t_from), True
        else:
            ts = float(tt[last + 1] - t_from)
    return dict(t_settle_s=ts, censored=censored, sigma_m=sig, band_m=band, mean_m=mean,
                out_frac_steady=out_frac)


def af_dev(L):
    up, mv = L['uavpos'], L['maneuver']
    tu = up['ts']
    dev = up['xd'] - np.interp(tu, mv['ts'], mv['x'])
    return tu, dev, A2.moving_average(tu, dev, MA_S)


def window_stats(L, a, b, steady_frac, kx, crit, tu, dev, d0=None):
    up, ph, pom = L['uavpos'], L['phynt'], L['pom']
    s0 = b - (b - a) * steady_frac
    su = (tu >= s0) & (tu <= b)
    F = stats(up['fx'][su])
    row = dict(K_x=kx, start=a, end=b, steady=(s0, b), F_N=F, af_dev_m=stats(dev[su]),
               wo_efx_N=(stats(ph['efx'][(ph['ts'] >= s0) & (ph['ts'] <= b)]) if ph is not None and ph.n else None),
               F_pred_N=predict_F(kx, crit))
    if pom is not None and pom.n:
        d0m = np.interp(tu, L['maneuver']['ts'], L['maneuver']['x']) - np.interp(tu, pom['ts'], pom['x'])
        row['d0_measured_m'] = stats(d0m[su])
    if F:
        row['F_err_rel'] = (F['mean'] - row['F_pred_N']) / row['F_pred_N']
    return row


# ---------------------------------------------------------------- t03
def load_t03(log_dir):
    d = dict(np.load(os.path.join(log_dir, NPZ_NAME), allow_pickle=False))
    meta = json.loads(str(d.pop('meta_json')))
    events = json.loads(str(d.pop('events_json')))
    d.pop('snapshots_json', None)
    calls = json.loads(str(d.pop('k_calls_json')))
    return d, meta, events, calls, read_dir(log_dir, names=('pom', 'uavpos', 'maneuver', 'phynt'))


def group_ramps(calls, dt=None, gap=None):
    dt = DT if dt is None else dt
    gap = RAMP_GAP_S if gap is None else gap
    rc = [c for c in calls if c['kind'] == CALL_RAMP]
    groups = []
    for c in rc:
        if groups and c['t_wall'] - groups[-1][-1]['t_wall'] <= gap:
            groups[-1].append(c)
        else:
            groups.append([c])
    return [dict(start=g[0]['t_wall'], end=g[-1]['t_wall'] + dt, n_calls=len(g),
                 k_first=g[0]['K_x'], k_last=g[-1]['K_x'],
                 k_from=None, k_to=g[-1]['K_x']) for g in groups]


def t03_holds(meta, events, d):
    """[(K_x, wall_start, wall_end)] for every hold, from the run's own events."""
    phase_ev = [e for e in events if e['name'].startswith('hold K_x=') or e['name'].startswith('ramp ')]
    goto_ev = [e for e in events if e['name'].startswith('maneuver.goto')]
    k_end = meta['k_phases'][-1][2] if meta.get('k_phases') else None
    after = [e for e in goto_ev if k_end is not None and e['t_nominal'] >= k_end - 1e-9]
    t_last = after[0]['wall_start'] if after else float(d['t_wall'][-1])
    holds = []
    for j, e in enumerate(phase_ev):
        if not e['name'].startswith('hold'):
            continue
        b = phase_ev[j + 1]['wall_start'] if j + 1 < len(phase_ev) else t_last
        holds.append((float(e['name'].split('=')[1]), e['wall_start'], b))
    return holds, t_last


def contact_end(events, x_contact=1.55):
    for e in events:
        a = e.get('args') or {}
        args = a.get('args') if isinstance(a, dict) else a
        if e['name'].startswith('maneuver.goto') and args and abs(args[0] - x_contact) < 1e-6:
            return e['wall_start'] + float(args[4])
    return None


def analyse_t03(log_dir, crit):
    d, meta, events, calls, L = load_t03(log_dir)
    C = crit['criteria']
    frac = crit['definitions']['steady_frac']
    R = dict(meta={k: meta.get(k) for k in ('git_rev', 'abort', 'error', 'read_error_counts',
                                            'read_error_counts_pending', 'pending', 'k_sequence', 'hold_s',
                                            'ramp_rate', 'k_breakpoints', 'safety_version', 'initial_af')})
    R['inventory'] = {f: dict(bytes=os.path.getsize(os.path.join(log_dir, f)),
                              sha256=sha256(os.path.join(log_dir, f))) for f in sorted(os.listdir(log_dir))
                      if os.path.isfile(os.path.join(log_dir, f))}
    R['missing_logs'] = [n for n, v in L.items() if v is None or v.n == 0]

    # C1
    n_err = int(sum((meta.get('read_error_counts') or {}).values()))
    c1 = dict(abort=meta.get('abort'), error=meta.get('error'), read_errors=n_err,
              read_errors_pending=meta.get('read_error_counts_pending'),
              passed=meta.get('abort') is None and meta.get('error') is None and n_err == 0)
    # C2
    atol = C['C2_applied_K_every_cycle']['atol_N_per_m']
    kv = d['af.valid.K'].astype(bool)
    dk = np.abs(d['af.K'][:, 0] - d['k_target_x'])
    bad_calls = [c for c in calls if not c.get('verify')]
    c2 = dict(cycles=int(kv.size), cycles_af_valid=int(kv.sum()),
              cycles_match=int((kv & (dk <= atol)).sum()), max_abs_diff=float(np.nanmax(dk)) if dk.size else None,
              calls=len(calls), calls_failed=len(bad_calls), atol=atol)
    c2['passed'] = bool(kv.size and kv.all() and c2['cycles_match'] == kv.size and not bad_calls and calls)
    R['C1'], R['C2'] = c1, c2

    holds, t_last = t03_holds(meta, events, d)
    ramps = group_ramps(calls)
    for j, r in enumerate(ramps):
        r['k_from'] = holds[j][0] if j < len(holds) else None
    R['holds_wall'] = [dict(K_x=k, start=a, end=b) for k, a, b in holds]
    per_hold, c3_rows = [], []
    c4 = dict(max_m=C['C4_max_abs_dxd']['max_m'], value_m=None, at=None, window=None, passed=False)
    c5_ok = False
    if not R['missing_logs']:
        up = L['uavpos']
        tu, dev, dev_ma = af_dev(L)
        # C3
        thr = C['C3_ramp_max_abs_dfx']['max_N']
        for r in ramps:
            v, at, n = max_abs_step(tu, up['fx'], r['start'], r['end'])
            xv, _, _ = max_abs_step(tu, up['xd'], r['start'], r['end'])
            w1 = max_abs_step(tu, up['fx'], r['start'] - T02_WIN_S, r['end'] + T02_WIN_S)
            c3_rows.append(dict(**r, n_rows=n, max_dfx_N=v, at=at, max_dxd_m=xv,
                                max_dfx_N_pm1s=w1[0], passed=v is not None and v <= thr))
        # C4
        if holds:
            a, b = holds[0][1], holds[-1][2]
            v, at, n = max_abs_step(tu, up['xd'], a, b)
            c4.update(value_m=v, at=at, window=(a, b), n_rows=n, passed=v is not None and v <= c4['max_m'])
        # C5 + settling
        tol = C['C5_steady_F_vs_series_model']['tol']
        ce = contact_end(events)
        for j, (k, a, b) in enumerate(holds):
            row = window_stats(L, a, b, frac, k, crit, tu, dev)
            t_from = ce if j == 0 else (ramps[j - 1]['end'] - DT if j - 1 < len(ramps) else a)
            row['settling_from'] = 'contact goto end' if j == 0 else 'last ramp call'
            row['settling_2sigma'] = settling_2sigma(tu, dev_ma, t_from, b, row['steady'])
            row['C5_passed'] = row.get('F_err_rel') is not None and abs(row['F_err_rel']) <= tol
            per_hold.append(row)
        c5_ok = bool(per_hold) and all(r['C5_passed'] for r in per_hold) and \
            len(per_hold) == len(meta.get('k_sequence') or [])
    R['C3'] = dict(max_N=C['C3_ramp_max_abs_dfx']['max_N'], ramps=c3_rows,
                   passed=bool(c3_rows) and all(r['passed'] for r in c3_rows) and
                   len(c3_rows) == max(len(meta.get('k_sequence') or []) - 1, 0))
    R['C4'] = c4
    R['C5'] = dict(tol=C['C5_steady_F_vs_series_model']['tol'], holds=per_hold, passed=c5_ok)
    R['gate_A2'] = dict(**{k: bool(R[k]['passed']) for k in ('C1', 'C2', 'C3', 'C4', 'C5')})
    R['gate_A2']['passed'] = all(R['gate_A2'].values())
    return R, d, L, holds, ramps


# ---------------------------------------------------------------- t02 (same metrics, steps)
def analyse_t02(log_dir, crit):
    if not os.path.exists(os.path.join(log_dir, A2.NPZ_NAME)):
        return None
    d, meta, events, snaps, calls, L = A2.load(log_dir)
    if any(v is None or v.n == 0 for v in L.values()):
        return None
    wins = A2.k_windows(meta, events, calls, d)
    frac = crit['definitions']['steady_frac']
    tu, dev, dev_ma = af_dev(L)
    up = L['uavpos']
    ce = contact_end(events)
    sch = [c for c in calls if c['kind'] == A2.CALL_SCHEDULE]
    per, steps = [], []
    for j, (k, a, b) in enumerate(wins):
        row = window_stats(L, a, b, frac, k, crit, tu, dev)
        t_from = ce if j == 0 else a
        row['settling_from'] = 'contact goto end' if j == 0 else 'set call'
        row['settling_2sigma'] = settling_2sigma(tu, dev_ma, t_from, b, row['steady'])
        per.append(row)
    for j, c in enumerate(sch):
        k0 = wins[j][0]
        v, at, n = max_abs_step(tu, up['fx'], c['t_wall'] - T02_WIN_S, c['t_wall'] + T02_WIN_S)
        xv, _, _ = max_abs_step(tu, up['xd'], c['t_wall'] - T02_WIN_S, c['t_wall'] + T02_WIN_S)
        dv = per[j]['af_dev_m']['mean'] if per[j]['af_dev_m'] else np.nan
        steps.append(dict(k_from=k0, k_to=c['K_x'], t_wall=c['t_wall'], max_dfx_N_pm1s=v, at=at, max_dxd_m=xv,
                          pred_dfx_N=MASS * abs(c['K_x'] - k0) * abs(dv) / M_AF))
    return dict(per_K=per, steps=steps, log_dir=os.path.relpath(log_dir, REPO))


# ---------------------------------------------------------------- outputs
def _f(x, fmt='{:.3f}', none='–'):
    return none if x is None or (isinstance(x, float) and not np.isfinite(x)) else fmt.format(x)


def summary_md(R, T2, crit_info):
    g = R['gate_A2']
    L = [f"# t03 summary (auto-generated by a03_k_ramp; estimates, not contact-force measurements)", '',
         f"criteria: `{crit_info['path']}` sha256 {crit_info['sha256'][:12]}…, committed {crit_info['committed']}", '',
         f"**Gate A2: {'PASSED' if g['passed'] else 'FAILED'}**", '',
         '| criterion | result | value |', '|---|---|---|',
         f"| C1 no abort / read error | {'✅' if g['C1'] else '❌'} | abort={R['C1']['abort'] is not None}, "
         f"error={R['C1']['error'] is not None}, read errors {R['C1']['read_errors']} "
         f"(pending excluded: {R['C1']['read_errors_pending']}) |",
         f"| C2 applied K every cycle | {'✅' if g['C2'] else '❌'} | {R['C2']['cycles_match']}/{R['C2']['cycles']} cycles, "
         f"{R['C2']['calls'] - R['C2']['calls_failed']}/{R['C2']['calls']} calls |",
         f"| C3 ramp max abs dfx <= {R['C3']['max_N']} N | {'✅' if g['C3'] else '❌'} | "
         + ', '.join(_f(r['max_dfx_N']) for r in R['C3']['ramps']) + ' N |',
         f"| C4 max abs dxd <= {R['C4']['max_m'] * 1e3:g} mm | {'✅' if g['C4'] else '❌'} | "
         f"{_f(None if R['C4']['value_m'] is None else R['C4']['value_m'] * 1e3, '{:.3f}')} mm |",
         f"| C5 steady F within {R['C5']['tol'] * 100:g} % | {'✅' if g['C5'] else '❌'} | "
         + ', '.join(_f(None if r.get('F_err_rel') is None else 100 * r['F_err_rel'], '{:+.1f}') for r in R['C5']['holds'])
         + ' % |', '']
    L += ['## Holds (steady = last half)', '',
          '| run | K_x | F = fx [N] | pred [N] | err | abs WO efx [N] | AF dev [mm] | settle 2σ [s] | 2σ band [mm] |',
          '|---|---|---|---|---|---|---|---|---|']

    def hold_line(run, r):
        s = r['settling_2sigma']
        return (f"| {run} | {r['K_x']:g} | {_f(r['F_N']['mean'] if r['F_N'] else None)} | {_f(r['F_pred_N'])} | "
                f"{_f(None if r.get('F_err_rel') is None else 100 * r['F_err_rel'], '{:+.1f} %')} | "
                f"{_f(abs(r['wo_efx_N']['mean']) if r.get('wo_efx_N') else None)} | "
                f"{_f(r['af_dev_m']['mean'] * 1e3 if r['af_dev_m'] else None, '{:.1f}')} | "
                f"{'≥ ' if s.get('censored') else ''}{_f(s['t_settle_s'], '{:.2f}')} | "
                f"{_f(None if s['band_m'] is None else s['band_m'] * 1e3, '{:.2f}')} |")
    for r in R['C5']['holds']:
        L.append(hold_line('t03', r))
    if T2:
        for r in T2['per_K']:
            L.append(hold_line('t02', r))
    L += ['', '## K changes', '', '| run | change | window | max abs dfx [N] | max abs dfx ±1 s [N] | max abs dxd [mm] |',
          '|---|---|---|---|---|---|']
    for r in R['C3']['ramps']:
        L.append(f"| t03 ramp | {r['k_from']:g}→{r['k_to']:g} ({r['n_calls']} calls) | ramp | {_f(r['max_dfx_N'])} | "
                 f"{_f(r['max_dfx_N_pm1s'])} | {_f(None if r['max_dxd_m'] is None else r['max_dxd_m'] * 1e3, '{:.3f}')} |")
    if T2:
        for s in T2['steps']:
            L.append(f"| t02 step | {s['k_from']:g}→{s['k_to']:g} | call ±1 s | – | {_f(s['max_dfx_N_pm1s'])} "
                     f"(pred m·ΔK·dev/M_af {_f(s['pred_dfx_N'])}) | "
                     f"{_f(None if s['max_dxd_m'] is None else s['max_dxd_m'] * 1e3, '{:.3f}')} |")
    return '\n'.join(L) + '\n'


def plot(R, d, L, holds, ramps, out, crit):
    T0 = d['t_wall'][0]
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    t = d['t_wall'] - T0
    ax[0].plot(t, d['k_target_x'], label='K_x target')
    ax[0].plot(t, d['af.K'][:, 0], '.', ms=2, label='get_af_parameters K_x')
    ax[0].set_ylabel('K_x [N/m]'); ax[0].legend()
    up, ph = L['uavpos'], L['phynt']
    if up is not None and up.n and L['maneuver'] is not None and L['maneuver'].n:
        tu, dev, dev_ma = af_dev(L)
        ax[1].plot(tu - T0, up['fx'], lw=0.5, label='uavpos fx [estimate]')
        if ph is not None and ph.n:
            ax[1].plot(ph['ts'] - T0, -ph['efx'], lw=0.5, alpha=0.7, label='-WO efx [estimate]')
        ax[2].plot(tu - T0, dev * 1e3, lw=0.4, alpha=0.5, label='AF dev')
        ax[2].plot(tu - T0, dev_ma * 1e3, lw=1.0, label=f'AF dev MA {MA_S:g} s')
        ax[3].plot(tu[1:] - T0, np.abs(np.diff(up['fx'])), lw=0.4, label='|Δfx| per row')
    for r in R['C5']['holds']:
        a, b = r['steady'][0] - T0, r['steady'][1] - T0
        ax[1].hlines(r['F_pred_N'], a, b, colors='k', linestyles='--')
        s = r['settling_2sigma']
        if s['mean_m'] is not None:
            ax[2].fill_between([r['start'] - T0, b], (s['mean_m'] - s['band_m']) * 1e3,
                               (s['mean_m'] + s['band_m']) * 1e3, color='k', alpha=0.12)
    for r in ramps:
        for a_ in ax:
            a_.axvspan(r['start'] - T0, r['end'] - T0, color='orange', alpha=0.25)
    ax[3].axhline(crit['criteria']['C3_ramp_max_abs_dfx']['max_N'], color='r', ls='--', label='C3 limit')
    ax[1].set_ylabel('F [N]'); ax[1].legend(); ax[1].set_ylim(-0.5, 2.0)
    ax[2].set_ylabel('dev [mm]'); ax[2].legend()
    ax[3].set_ylabel('|Δfx| [N]'); ax[3].set_ylim(0, 0.2); ax[3].legend()
    ax[3].set_xlabel('t [s] (wall, from first sample); orange = ramp windows; grey = 2σ band')
    fig.suptitle(f"t03 K_x ramp — gate A2 {'PASSED' if R['gate_A2']['passed'] else 'FAILED'} "
                 '(dashed: F prediction; forces are estimates)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'k_ramp.png'), dpi=120)
    plt.close(fig)


def main(log_dir=LOG_DIR, t02_dir=T02_DIR, out_dir=OUT_DIR, criteria=CRITERIA):
    crit, info = load_criteria(criteria)
    os.makedirs(out_dir, exist_ok=True)
    R, d, L, holds, ramps = analyse_t03(log_dir, crit)
    R['criteria'] = info
    T2 = analyse_t02(t02_dir, crit)
    R['t02'] = T2
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(R, f, indent=1, default=float)
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(summary_md(R, T2, info))
    plot(R, d, L, holds, ramps, out_dir, crit)
    print(summary_md(R, T2, info))
    return 0 if R['gate_A2']['passed'] else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:3]))
