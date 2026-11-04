"""
a04_ft_check — t04 analysis: Gazebo force_torque sensor (ee_joint, /noc/ee_ft) vs the
WO estimate (phynt.log efx) and the controller command (uavpos.log fx); gate B1.

Every gate threshold is read from noc/config/t04_ft_criteria.json (committed before the
flight). Sources and their nature:
  F/T  [SIM SENSOR]  physics-engine constraint wrench of ee on base at ee_joint (child frame
                     = body frame), recorded by log_ft.sh; not the wall contact force itself
                     (differs by m_ee (a - g) and friction terms), uses no control-chain output.
  efx  [LOG estimate] WO external force (rotor speeds + model + pom state).
  fx   [LOG command]  uavpos force command; -fx ~ external force when quasi-static.

Time alignment: F/T header stamps are simulation time. They are mapped to the wall clock
with the rolling lower envelope (2 s) of host receive time - stamp (buffering only delays
reception). Component logs are on the wall clock. Fn = world-x component of the F/T force
(rotation from pom roll/pitch/yaw interpolated at the F/T time).

Usage: .venv/bin/python -m noc.analysis.a04_ft_check [log_dir]
Writes noc/results/04_ft_check/{metrics.json, summary.md, ft_check.png}
"""

import collections
import json
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis.logs import read_dir
from noc.analysis.baseline_06a import sha256, rpy_to_R
from noc.common import ft_log
from noc.common.flight import SAFETY

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs', 'noc', 't04_ft_check')
OUT_DIR = os.path.join(REPO, 'noc', 'results', '04_ft_check')
CRITERIA = os.path.join(REPO, 'noc', 'config', 't04_ft_criteria.json')
NPZ_NAME = 't04_ft_check.npz'
FT_NAME = 'ft.log'

ENVELOPE_S = 2.0          # criteria definitions.Fn
GRID_HZ = 100.0           # criteria reported_not_gated.lag
LAG_MAX_S = 1.0
SPAN_PRE_S, SPAN_POST_S = 2.0, 5.0


def load_criteria(path=CRITERIA):
    with open(path) as f:
        c = json.load(f)
    r = subprocess.run(['git', '-C', REPO, 'log', '-1', '--format=%H %cI', '--', os.path.relpath(path, REPO)],
                       capture_output=True, text=True)
    return c, dict(path=os.path.relpath(path, REPO), sha256=sha256(path), committed=r.stdout.strip() or None)


# ---------------------------------------------------------------- time base / frames
def rolling_min(t, y, half):
    """min of y over samples with t in [t_i - half, t_i + half]; t sorted ascending."""
    n = t.size
    out = np.empty(n)
    dq = collections.deque()       # indices, y increasing
    j = 0                          # next index to add
    lo = 0
    for i in range(n):
        while j < n and t[j] <= t[i] + half:
            while dq and y[dq[-1]] >= y[j]:
                dq.pop()
            dq.append(j)
            j += 1
        while t[lo] < t[i] - half:
            lo += 1
        while dq[0] < lo:
            dq.popleft()
        out[i] = y[dq[0]]
    return out


def ft_wall_time(A, envelope=ENVELOPE_S):
    """Map header stamps to the wall clock; returns (t_wall, offset envelope)."""
    off = A['host'] - A['stamp']
    o = np.argsort(A['stamp'], kind='stable')      # rolling_min needs ascending stamps
    env = np.empty_like(off)
    env[o] = rolling_min(A['stamp'][o], off[o], envelope / 2)
    return A['stamp'] + env, env


def body_to_world_x(L, t, F):
    """World-x component of body-frame forces F (n x 3) at times t (pom roll/pitch/yaw, ZYX)."""
    pom = L['pom']
    r, p, y = (np.interp(t, pom['ts'], pom[k]) for k in ('roll', 'pitch', 'yaw'))
    return np.einsum('nj,nj->n', rpy_to_R(r, p, y)[:, 0, :], F)


def tip_surface_x(L, a, b, c, radius):
    """x of the tip surface (+x side) for every pom sample in [a, b]."""
    pom = L['pom']
    m = (pom['ts'] >= a) & (pom['ts'] <= b)
    R = rpy_to_R(pom['roll'][m], pom['pitch'][m], pom['yaw'][m])
    return pom['x'][m] + R[:, 0, :] @ np.asarray(c, float) + radius


def lag_xcorr(t_grid, ref, est, lag_max, hz):
    """Lag (s) maximising corr(ref(t), est(t + lag)); positive = est lags ref."""
    r = ref - ref.mean()
    e = est - est.mean()
    K = int(round(lag_max * hz))
    best, best_c, cs = 0, -np.inf, []
    for k in range(-K, K + 1):
        if k >= 0:
            a, b = r[:r.size - k], e[k:]
        else:
            a, b = r[-k:], e[:e.size + k]
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        cs.append(c)
        if c > best_c:
            best, best_c = k, c
    return best / hz, best_c


def shifted_rmse(t_grid, ref, est_t, est_y, lag, mask):
    e = np.interp(t_grid + lag, est_t, est_y)
    d = (e - ref)[mask]
    return float(np.sqrt(np.mean(d ** 2))) if d.size else None, float(np.mean(d)) if d.size else None


# ---------------------------------------------------------------- analysis
def load(log_dir):
    d = dict(np.load(os.path.join(log_dir, NPZ_NAME), allow_pickle=False))
    meta = json.loads(str(d.pop('meta_json')))
    events = json.loads(str(d.pop('events_json')))
    d.pop('snapshots_json', None)
    A, info = ft_log.read_ft(os.path.join(log_dir, FT_NAME))
    L = read_dir(log_dir, names=('pom', 'uavpos', 'maneuver', 'phynt'))
    return d, meta, events, A, info, L


def window_walls(events):
    w = {}
    for e in events:
        n = e['name']
        if n.startswith('window '):
            _, name, kind = n.split()
            w.setdefault(name, [None, None])[0 if kind == 'start' else 1] = e['wall_start']
    return {k: tuple(v) for k, v in w.items()}


def goto_walls(events):
    return [(e['t_nominal'], e['wall_start'], (e.get('args') or {}).get('args'))
            for e in events if e['name'].startswith('maneuver.goto')]


def mstats(x):
    x = np.asarray(x, float)
    return dict(n=int(x.size), mean=float(x.mean()), std=float(x.std(ddof=1))) if x.size > 1 else dict(n=int(x.size))


def analyse(log_dir, crit, model=None):
    d, meta, events, A, info, L = load(log_dir)
    C = crit['criteria']
    model = model or ft_log.model_info()
    R = dict(meta={k: meta.get(k) for k in ('git_rev', 'abort', 'error', 'read_error_counts',
                                            'read_error_counts_pending', 'pending', 'ft_stream', 'ft_live',
                                            'safety_version', 'initial_af')},
             ft_logger={k: v for k, v in (meta.get('ft_logger') or {}).items() if k != 'stderr'},
             ft_logger_stderr_tail=(meta.get('ft_logger') or {}).get('stderr', '')[-500:],
             model=model, ft_file=dict(info, path=os.path.join(os.path.relpath(log_dir, REPO), FT_NAME)))
    R['inventory'] = {f: dict(bytes=os.path.getsize(os.path.join(log_dir, f)),
                              sha256=sha256(os.path.join(log_dir, f))) for f in sorted(os.listdir(log_dir))
                      if os.path.isfile(os.path.join(log_dir, f))}
    R['missing_logs'] = [n for n, v in L.items() if v is None or v.n == 0]
    W = window_walls(events)
    G = goto_walls(events)
    R['windows_wall'] = W
    span = (G[0][1], G[-1][1]) if G else (None, None)
    R['flight_span_wall'] = span

    # C1
    lgi = meta.get('ft_logger') or {}
    n_err = int(sum((meta.get('read_error_counts') or {}).values()))
    c1 = dict(ft_messages=info['n'], ft_bad_lines=info['n_bad'], logger_alive_at_stop=lgi.get('alive_at_stop'),
              abort=meta.get('abort'), error=meta.get('error'), read_errors=n_err)
    c1['passed'] = bool(info['n'] >= 1 and info['n_bad'] == 0 and lgi.get('alive_at_stop') is True and
                        meta.get('abort') is None and meta.get('error') is None and n_err == 0)
    R['C1'] = c1

    ok_data = info['n'] > 10 and not R['missing_logs'] and None not in span
    c2 = dict(max_gap_s=C['C2_timestamps_continuous_and_fast']['max_gap_s'],
              min_rate_hz=C['C2_timestamps_continuous_and_fast']['min_rate_hz'], passed=False)
    c3 = dict(passed=False)
    c4 = dict(factor=C['C4_contact_force_above_noise']['factor'], passed=False)
    c5 = dict(ratio_min=C['C5_pre_post_consistent']['ratio_min'], ratio_max=C['C5_pre_post_consistent']['ratio_max'],
              passed=False)
    if ok_data:
        # rows stay in file order (= arrival order): C2 judges the stamps in that order
        tw, env = ft_wall_time(A)
        A['t'] = tw
        # C2 on the flight span (host time)
        ms = (A['host'] >= span[0]) & (A['host'] <= span[1])
        st, ho = A['stamp'][ms], A['host'][ms]
        ds = np.diff(st)
        c2.update(n=int(ms.sum()), strictly_increasing=bool(ds.size and (ds > 0).all()),
                  n_non_increasing=int((ds <= 0).sum()), max_gap=float(ds.max()) if ds.size else None,
                  rate_stamp_hz=float((st.size - 1) / (st[-1] - st[0])) if st.size > 1 and st[-1] > st[0] else 0.0,
                  rate_host_hz=float((ho.size - 1) / (ho[-1] - ho[0])) if ho.size > 1 and ho[-1] > ho[0] else 0.0,
                  rtf=float((st[-1] - st[0]) / (ho[-1] - ho[0])) if ho.size > 1 else None,
                  host_minus_env_p99_ms=float(np.percentile((A['host'] - A['t'])[ms], 99) * 1e3))
        c2['passed'] = bool(c2['strictly_increasing'] and c2['max_gap'] is not None and c2['max_gap'] <= c2['max_gap_s']
                            and c2['rate_stamp_hz'] > c2['min_rate_hz'] and c2['rate_host_hz'] > c2['min_rate_hz'])
        # Fn
        F = np.column_stack([A['fx'], A['fy'], A['fz']])
        A['Fn'] = body_to_world_x(L, A['t'], F)

        def win(name):
            a, b = W.get(name, (None, None))
            if a is None or b is None:
                return np.zeros(A['t'].size, bool)
            return (A['t'] >= a) & (A['t'] <= b)
        w1, wc, w2 = win('FREE1'), win('CONTACT'), win('FREE2')
        # C3
        c3c = C['C3_bias_measured_in_free_flight']
        fn1 = A['Fn'][w1]
        tip = tip_surface_x(L, *W.get('FREE1', (0, 0)), c3c['tip_center_body_m'], c3c['tip_radius_m']) \
            if W.get('FREE1') and None not in W['FREE1'] else np.array([])
        c3.update(n=int(fn1.size), bias=float(fn1.mean()) if fn1.size else None,
                  free_std=float(fn1.std(ddof=1)) if fn1.size > 1 else None,
                  tip_x_max=float(tip.max()) if tip.size else None,
                  tip_limit=c3c['x_wall_m'] - c3c['free_margin_m'], min_samples=c3c['min_samples'])
        c3['passed'] = bool(fn1.size >= c3c['min_samples'] and np.isfinite(c3['bias'] or np.nan) and
                            np.isfinite(c3['free_std'] or np.nan) and tip.size and c3['tip_x_max'] <= c3['tip_limit'])
        # C4 / C5
        bias = c3['bias'] if c3['bias'] is not None else np.nan
        fnc, fn2 = A['Fn'][wc], A['Fn'][w2]
        if fnc.size > 1 and fn2.size > 1 and fn1.size > 1:
            mc = float(fnc.mean() - bias)
            c4.update(contact_mean_corrected=mc, free_std=c3['free_std'], ratio=abs(mc) / c3['free_std'],
                      n=int(fnc.size), sign_as_expected=bool(mc < 0))
            c4['passed'] = bool(abs(mc) > c4['factor'] * c3['free_std'])
            dpre = float(fnc.mean() - fn1.mean())
            dpost = float(fnc.mean() - fn2.mean())
            q = dpre / dpost if dpost != 0 else np.inf
            c5.update(delta_pre=dpre, delta_post=dpost, ratio=float(q), free2_mean=float(fn2.mean()),
                      bias_drift=float(fn2.mean() - fn1.mean()))
            c5['passed'] = bool(np.sign(dpre) == np.sign(dpost) and c5['ratio_min'] <= q <= c5['ratio_max'])
        # bias / window stats in the child frame
        Aw = dict(A, host=A['t'])
        R['ft_windows_child'] = {n: ft_log.window_stats(Aw, *W[n]) for n in ('FREE1', 'CONTACT', 'FREE2')
                                 if W.get(n) and None not in W[n]}
        R['Fn_windows'] = {n: mstats(A['Fn'][m]) for n, m in (('FREE1', w1), ('CONTACT', wc), ('FREE2', w2))}
        # reported: lag and RMSE
        cg = [g for g in G if g[2] and abs(g[2][0] - 1.55) < 1e-6]
        rg = [g for g in G if cg and g[0] > cg[0][0]]
        if cg and rg:
            a, b = cg[0][1] - SPAN_PRE_S, rg[0][1] + SPAN_POST_S
            tg = np.arange(a, b, 1.0 / GRID_HZ)
            ref = np.interp(tg, A['t'], A['Fn']) - bias
            up, ph = L['uavpos'], L['phynt']
            est = {'WO efx': (ph['ts'], ph['efx']), '-uavpos fx': (up['ts'], -up['fx'])}
            mcg = (tg >= W['CONTACT'][0]) & (tg <= W['CONTACT'][1]) if W.get('CONTACT') and None not in W['CONTACT'] \
                else np.zeros(tg.size, bool)
            cmp_ = {}
            for name, (et, ey) in est.items():
                lag, cc = lag_xcorr(tg, ref, np.interp(tg, et, ey), LAG_MAX_S, GRID_HZ)
                rm_span, me_span = shifted_rmse(tg, ref, et, ey, lag, np.ones(tg.size, bool))
                rm_c, me_c = shifted_rmse(tg, ref, et, ey, lag, mcg)
                rm0, _ = shifted_rmse(tg, ref, et, ey, 0.0, np.ones(tg.size, bool))
                cmp_[name] = dict(lag_s=lag, corr=cc, rmse_span_N=rm_span, mean_err_span_N=me_span,
                                  rmse_contact_N=rm_c, mean_err_contact_N=me_c, rmse_span_no_lag_N=rm0,
                                  contact_mean_N=float(ey[(et >= W['CONTACT'][0]) & (et <= W['CONTACT'][1])].mean())
                                  if mcg.any() else None)
            R['comparison'] = dict(span_wall=(a, b), grid_hz=GRID_HZ, lag_search_s=LAG_MAX_S,
                                   ft_contact_mean_corrected_N=c4.get('contact_mean_corrected'), estimates=cmp_)
        R['_series'] = A
    R['C2'], R['C3'], R['C4'], R['C5'] = c2, c3, c4, c5

    # C6
    am = d['af_match']
    af_valid = d['af.valid.K'].astype(bool) & d['af.valid.B'].astype(bool) & d['af.valid.mass'].astype(bool)
    safety_same = json.loads(json.dumps(meta.get('safety'))) == json.loads(json.dumps(SAFETY))
    c6 = dict(safety_version=meta.get('safety_version'), safety_equal_flight_SAFETY=safety_same,
              initial_af_ok=(meta.get('initial_af') or {}).get('ok'), cycles=int(am.size),
              cycles_af_valid=int(af_valid.sum()), cycles_af_match=int((am == 1).sum()),
              n_af_set_calls=meta.get('n_af_set_calls'), abort=meta.get('abort'))
    c6['passed'] = bool(meta.get('safety_version') == 2 and safety_same and meta.get('abort') is None and
                        c6['initial_af_ok'] is True and am.size and af_valid.all() and (am == 1).all() and
                        meta.get('n_af_set_calls') == 0)
    R['C6'] = c6
    R['gate_B1'] = {k: bool(R[k]['passed']) for k in ('C1', 'C2', 'C3', 'C4', 'C5', 'C6')}
    R['gate_B1']['passed'] = all(R['gate_B1'].values())
    return R, d, L


# ---------------------------------------------------------------- outputs
def _f(x, fmt='{:.4f}'):
    return '–' if x is None or (isinstance(x, float) and not np.isfinite(x)) else fmt.format(x)


def summary_md(R, info):
    g = R['gate_B1']
    ok = lambda k: '✅' if g[k] else '❌'
    c2, c3, c4, c5, c6 = R['C2'], R['C3'], R['C4'], R['C5'], R['C6']
    L = ['# t04 summary (auto-generated by a04_ft_check)', '',
         f"criteria: `{info['path']}` sha256 {info['sha256'][:12]}…, committed {info['committed']}", '',
         f"F/T source: Gazebo force_torque sensor `{R['model']['sensor']}` on `{R['model']['joint']}` "
         f"({R['model']['parent']}→{R['model']['child']}), topic `{R['model']['topic']}`, frame {R['model']['frame']}, "
         f"{R['model']['measure_direction']}, {R['model']['update_rate_hz']:g} Hz; file `{R['ft_file']['path']}`.", '',
         f"**Gate B1: {'PASSED' if g['passed'] else 'FAILED'}**", '',
         '| criterion | result | value |', '|---|---|---|',
         f"| C1 recording + read chain | {ok('C1')} | {R['C1']['ft_messages']} msgs, {R['C1']['ft_bad_lines']} bad, "
         f"logger alive {R['C1']['logger_alive_at_stop']}, abort {R['C1']['abort'] is not None}, "
         f"read errors {R['C1']['read_errors']} |",
         f"| C2 stamps continuous, > {c2['min_rate_hz']:g} Hz | {ok('C2')} | max gap {_f(c2.get('max_gap'))} s, "
         f"{_f(c2.get('rate_stamp_hz'), '{:.1f}')} Hz (stamp), {_f(c2.get('rate_host_hz'), '{:.1f}')} Hz (host), "
         f"RTF {_f(c2.get('rtf'), '{:.3f}')} |",
         f"| C3 bias in free flight | {ok('C3')} | n {c3.get('n')}, bias {_f(c3.get('bias'))} N, std "
         f"{_f(c3.get('free_std'))} N, tip x max {_f(c3.get('tip_x_max'), '{:.3f}')} m |",
         f"| C4 contact > {c4['factor']:g}·std | {ok('C4')} | Fn−bias {_f(c4.get('contact_mean_corrected'))} N "
         f"= {_f(c4.get('ratio'), '{:.1f}')}·std |",
         f"| C5 pre/post consistent | {ok('C5')} | Δpre {_f(c5.get('delta_pre'))}, Δpost {_f(c5.get('delta_post'))}, "
         f"ratio {_f(c5.get('ratio'), '{:.2f}')} |",
         f"| C6 safety + K as t02/t03 | {ok('C6')} | safety v{c6['safety_version']}, same {c6['safety_equal_flight_SAFETY']}, "
         f"AF match {c6['cycles_af_match']}/{c6['cycles']} |", '']
    cmp_ = (R.get('comparison') or {}).get('estimates') or {}
    if cmp_:
        L += ['## F/T vs estimates (span: contact goto −2 s … retreat goto +5 s)', '',
              '| estimate | lag [s] | corr | RMSE span [N] | RMSE span, no lag [N] | RMSE CONTACT [N] | mean err CONTACT [N] | contact mean [N] |',
              '|---|---|---|---|---|---|---|---|']
        for n, c in cmp_.items():
            L.append(f"| {n} | {_f(c['lag_s'], '{:+.2f}')} | {_f(c['corr'], '{:.3f}')} | {_f(c['rmse_span_N'])} | "
                     f"{_f(c['rmse_span_no_lag_N'])} | {_f(c['rmse_contact_N'])} | {_f(c['mean_err_contact_N'])} | "
                     f"{_f(c['contact_mean_N'])} |")
        L.append(f"\nF/T contact mean (Fn − bias): {_f(R['comparison']['ft_contact_mean_corrected_N'])} N")
    return '\n'.join(L) + '\n'


def plot(R, L, out):
    A = R.get('_series')
    fig, ax = plt.subplots(3, 1, figsize=(10, 10))
    if A is not None:
        T0 = A['t'][0]
        bias = R['C3'].get('bias') or 0.0
        ax[0].plot(A['t'] - T0, A['Fn'] - bias, lw=0.4, label='F/T Fn − bias [sim sensor]')
        ph, up = L['phynt'], L['uavpos']
        if ph is not None and ph.n:
            ax[0].plot(ph['ts'] - T0, ph['efx'], lw=0.5, alpha=0.8, label='WO efx [estimate]')
        if up is not None and up.n:
            ax[0].plot(up['ts'] - T0, -up['fx'], lw=0.5, alpha=0.6, label='−uavpos fx [command]')
        for n, (a, b) in R['windows_wall'].items():
            if a and b:
                for a_ in ax[:2]:
                    a_.axvspan(a - T0, b - T0, alpha=0.12, color='g' if n.startswith('FREE') else 'r')
        ax[0].set_ylim(-1.5, 1.0); ax[0].set_ylabel('x force [N]'); ax[0].legend(fontsize=8)
        cmp_ = R.get('comparison')
        if cmp_:
            a, b = cmp_['span_wall']
            m = (A['t'] >= a) & (A['t'] <= b)
            ax[1].plot(A['t'][m] - T0, A['Fn'][m] - bias, lw=0.4, label='F/T Fn − bias')
            for n, (et, ey) in (('WO efx', (ph['ts'], ph['efx'])), ('−fx', (up['ts'], -up['fx']))):
                lag = cmp_['estimates']['WO efx' if n == 'WO efx' else '-uavpos fx']['lag_s']
                mm = (et >= a) & (et <= b)
                ax[1].plot(et[mm] - lag - T0, ey[mm], lw=0.6, label=f'{n} shifted by −{lag:+.2f} s')
            ax[1].set_xlim(a - T0, b - T0); ax[1].set_ylabel('[N]'); ax[1].legend(fontsize=8)
        ds = np.diff(A['stamp'])
        ax[2].plot(A['host'][1:] - T0, ds * 1e3, '.', ms=1, label='Δ header stamp [ms]')
        ax[2].plot(A['host'][1:] - T0, (A['host'] - A['t'])[1:] * 1e3, lw=0.5, label='host − mapped wall [ms]')
        ax[2].set_ylabel('ms'); ax[2].set_xlabel('t [s] (wall, from first F/T sample)'); ax[2].legend(fontsize=8)
    fig.suptitle(f"t04 F/T check — gate B1 {'PASSED' if R['gate_B1']['passed'] else 'FAILED'}")
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'ft_check.png'), dpi=120)
    plt.close(fig)


def main(log_dir=LOG_DIR, out_dir=OUT_DIR, criteria=CRITERIA):
    crit, info = load_criteria(criteria)
    os.makedirs(out_dir, exist_ok=True)
    R, d, L = analyse(log_dir, crit)
    R['criteria'] = info
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump({k: v for k, v in R.items() if k != '_series'}, f, indent=1, default=float)
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(summary_md(R, info))
    plot(R, L, out_dir)
    print(summary_md(R, info))
    return 0 if R['gate_B1']['passed'] else 1


if __name__ == '__main__':
    sys.exit(main(*sys.argv[1:2]))
