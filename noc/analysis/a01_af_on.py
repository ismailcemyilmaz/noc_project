"""
a01_af_on — compare t00 (06a as-is, AF without force input) and t01 (rotor_measure
connected, non-blocking goto).

Per run:
  * valid-sample fraction of uavatt wrench_measure.force and phynt external_wrench.force
  * phynt.log data rows, log_info snapshots, /tmp/phynt.log size
  * steady contact: F [COMPUTED quasi-static -uavpos fx] and WO f_ext_x [LOG],
    AF deviation (uavpos xd - maneuver x) [LOG], uavpos fx [LOG]
  * prediction with series stiffness F = d0 / (1/Kpxy + 1/K_af,x), dev = F / K_af,x
    for d0 = 0.19 m (given) and d0 measured (x_nom - x_body)
  * Kv share of the steady force command
  * reference 'vel'/'pos' field validity during motion / static / contact

Usage: .venv/bin/python -m noc.analysis.a01_af_on
Writes noc/results/01_af_on/{metrics.json, compare.png}
"""

import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis.logs import read_dir
from noc.analysis.baseline_06a import stats, segments, in_windows, sha256

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
RUNS = {'t00': (os.path.join(REPO, 'logs', 'noc', 't00'), 't00_diag.npz'),
        't01': (os.path.join(REPO, 'logs', 'noc', 't01_af_on'), 't01_af_on.npz')}
OUT_DIR = os.path.join(REPO, 'noc', 'results', '01_af_on')

KPXY, KVXY = 5.0, 7.0          # 06a setup() uavpos gains
D0_GIVEN = 0.19                # m, reference penetration used in the prediction
X_TASK_NOM, X_TASK_BODY = 1.5, 1.3
SETTLE_S = 1.0
STATIC_V_TOL = 1e-6


def load_run(log_dir, npz_name):
    d = dict(np.load(os.path.join(log_dir, npz_name), allow_pickle=False))
    meta = json.loads(str(d.pop('meta_json')))
    events = json.loads(str(d.pop('events_json')))
    snaps = json.loads(str(d.pop('snapshots_json')))
    return d, meta, events, snaps, read_dir(log_dir)


def frac(m):
    m = np.asarray(m, bool)
    return float(m.mean()) if m.size else None


def goto_windows(events, T0):
    """Motion windows [start, start + duration] for every waypoint goto (blocking or ack)."""
    w = []
    for e in events:
        if e['name'].startswith('maneuver.goto'):
            a = e.get('args')
            args = a['args'] if isinstance(a, dict) else a
            w.append((e['wall_start'] - T0, e['wall_start'] - T0 + float(args[4])))
    return w


def analyse(name, log_dir, npz_name):
    d, meta, events, snaps, L = load_run(log_dir, npz_name)
    R = {'meta': {k: meta.get(k) for k in ('git_rev', 'sources', 'goto_mode', 'abort', 'read_error_counts',
                                             'setup_06a', 'safety')}}
    R['inventory'] = {f: dict(bytes=os.path.getsize(os.path.join(log_dir, f)),
                              sha256=sha256(os.path.join(log_dir, f))) for f in sorted(os.listdir(log_dir))}
    pom, up, mv = L['pom'], L['uavpos'], L['maneuver']
    T0 = pom['ts'][0]
    t = d['t_wall'] - T0
    tp, tu, tm = pom['ts'] - T0, up['ts'] - T0, mv['ts'] - T0

    # phases from component logs
    task_p = (np.interp(tp, tm, mv['x']) > X_TASK_NOM) & (pom['x'] > X_TASK_BODY)
    task_segs = segments(task_p, tp, min_len=0.5)
    vnom = np.sqrt(mv['vx'] ** 2 + mv['vy'] ** 2 + mv['vz'] ** 2)
    st = segments((vnom < STATIC_V_TOL) & (mv['x'] > X_TASK_NOM), tm, min_len=SETTLE_S + 0.5)
    gw = goto_windows(events, T0)
    starts = [a for a, _ in gw]
    steady_ws = [(a + SETTLE_S, min([g for g in starts if g > a + SETTLE_S] or [b])) for a, b in st]
    R['phases'] = dict(task_segments=task_segs, steady_windows=steady_ws, goto_windows=gw)
    task_n = in_windows(t, task_segs)
    steady_n = in_windows(t, steady_ws) & task_n
    motion_n = in_windows(t, gw)
    su = in_windows(tu, steady_ws) & (np.interp(tu, tp, pom['x']) > X_TASK_BODY)

    # 1) valid fractions
    R['valid_frac'] = {k: frac(d[k]) for k in ('wrench_meas.valid.force', 'ext_wrench.valid.force',
                                               'wrench_meas.valid.ts', 'ext_wrench.valid.ts')}
    # 2) phynt.log
    R['phynt_log'] = dict(rows=L['phynt'].n if L.get('phynt') is not None else None,
                          log_info={k: v.get('log_info') for k, v in snaps.items()},
                          tmp_bytes_last=(int(d['phynt_log_bytes'][-1]) if 'phynt_log_bytes' in d else
                                          meta.get('tmp_phynt_log_bytes_after_stop')))

    # 3) steady contact
    fe = d['ext_wrench.force']
    dev = up['xd'] - np.interp(tu, tm, mv['x'])
    x_nom = np.interp(tu, tm, mv['x'])
    xb = np.interp(tu, tp, pom['x'])
    S = dict(n_uavpos=int(su.sum()), n_npz=int(steady_n.sum()),
             F_quasistatic_N=stats(up['fx'][su]),            # [COMPUTED] f_ext,x = -fx  -> F = fx
             wo_f_ext_x_N=stats(fe[steady_n, 0]),             # [LOG] WO estimate
             af_dev_x_m=stats(dev[su]),                       # [LOG] uavpos xd - maneuver x
             uavpos_fx_N=stats(up['fx'][su]),
             e_x_m=stats(up['e_x'][su]),
             body_x_m=stats(xb[su]),
             d0_measured_m=stats(x_nom[su] - xb[su]),
             kv_share_N=stats(KVXY * up['e_vx'][su]),
             kp_share_N=stats(KPXY * up['e_x'][su]))
    if 'uav_input.thrust' in d:
        S['uav_input_thrust_x_N'] = stats(d['uav_input.thrust'][steady_n, 0])
    if 'rotor_meas.velocity' in d:
        v = d['rotor_meas.velocity'][:, :6]
        S['rotor_vel_max'] = float(np.nanmax(v)) if np.isfinite(v).any() else None
    # AF K_x from snapshot
    kaf = None
    for s in snaps.values():
        af = (s.get('get_af_parameters') or {}).get('af') if isinstance(s.get('get_af_parameters'), dict) else None
        if af and af.get('K'):
            kaf = float(af['K'][0]); break
    S['K_af_x'] = kaf
    if kaf and S['d0_measured_m']:
        for lbl, d0 in (('given', D0_GIVEN), ('measured', S['d0_measured_m']['mean'])):
            F = d0 / (1 / KPXY + 1 / kaf)
            S[f'prediction_{lbl}'] = dict(d0=d0, F=F, dev=F / kaf, k_series=1 / (1 / KPXY + 1 / kaf))
        if S['F_quasistatic_N']:
            S['k_eff_measured'] = S['F_quasistatic_N']['mean'] / S['d0_measured_m']['mean']
    R['steady'] = S

    # per steady window: AF deviation (mean, last 0.5 s), WO f_ext_x from phynt.log, F/K_af
    ph = L.get('phynt')
    have_ph = ph is not None and ph.n > 0
    tph = ph['ts'] - T0 if have_ph else None
    win = []
    for a, b in steady_ws:
        m = (tu >= a) & (tu <= b)
        late = (tu >= b - 0.5) & (tu <= b)
        e = dict(window=[a, b], dev_mean_m=float(dev[m].mean()), dev_last05_m=float(dev[late].mean()),
                 fx_mean_N=float(up['fx'][m].mean()))
        if have_ph:
            mp = (tph >= a) & (tph <= b)
            e['wo_efx_mean_N'] = float(ph['efx'][mp].mean())
            if kaf:
                e['wo_efx_over_Kaf_m'] = e['wo_efx_mean_N'] / kaf
        win.append(e)
    R['steady_windows_detail'] = win

    # [COMPUTED] AF model driven by the logged WO force: m d'' + B d' + K d = efx
    if have_ph and kaf:
        af = None
        for s_ in snaps.values():
            g = s_.get('get_af_parameters')
            if isinstance(g, dict) and 'af' in g:
                af = g['af']; break
        m_af, B_af = float(af['mass']), float(af['B'][0])
        o = np.argsort(tph)
        tt, ff = tph[o], ph['efx'][o]
        dsim = np.zeros_like(tt); x = v = 0.0
        for i in range(1, len(tt)):
            dt = tt[i] - tt[i - 1]
            if 0 < dt <= 0.1:
                acc = (ff[i - 1] - B_af * v - kaf * x) / m_af
                v += acc * dt; x += v * dt
            dsim[i] = x
        ds = np.interp(tu, tt, dsim)
        tk = in_windows(tu, task_segs)
        R['af_model_check'] = dict(
            model='m d\'\' + B d\' + K d = efx(phynt.log), explicit Euler on phynt.log ts',
            mass=m_af, B=B_af, K=kaf,
            rms_log_minus_sim_contact_m=float(np.sqrt(np.mean((dev[tk] - ds[tk]) ** 2))),
            corr_contact=float(np.corrcoef(dev[tk], ds[tk])[0, 1]),
            per_window=[dict(window=w_['window'], sim_mean_m=float(ds[in_windows(tu, [tuple(w_['window'])])].mean()),
                             log_mean_m=w_['dev_mean_m']) for w_ in win])
        R['wo_noise'] = dict(efx_free_flight_12_20s=stats(ph['efx'][(tph > 12) & (tph < 20)]),
                             efx_after_zero_22p7_24p5s=stats(ph['efx'][(tph > 22.7) & (tph < 24.5)]))
    dtw = np.diff(d['t_wall'])
    R['python_loop'] = dict(dt_ms=stats(dtw * 1e3), dt_p99_ms=float(np.percentile(dtw, 99) * 1e3),
                            n_dt_gt_100ms=int((dtw > 0.1).sum()), late_max_s=float(d['late_s'].max()),
                            goto_blocked_s=[e['blocked_s'] for e in events if e['name'].startswith('maneuver.goto')])

    # 4) vel/pos fields of reference ports
    V = {}
    for src in ('man_des', 'phynt_ref', 'phynt_des'):
        e = {}
        for ph, m in (('motion', motion_n), ('static', ~motion_n), ('contact', task_n), ('steady_contact', steady_n)):
            vv = d[f'{src}.valid.vel'][m]
            vel = d[f'{src}.vel'][m]
            e[ph] = dict(n=int(m.sum()), vel_valid=frac(vv), pos_valid=frac(d[f'{src}.valid.pos'][m]),
                         vel_none_frac=(None if vv.size == 0 else float(1 - vv.mean())),
                         vel_nonzero_frac=(None if vv.size == 0 else
                                           float((np.nansum(np.abs(vel), axis=1) > 0).mean())))
        V[src] = e
    R['ref_fields'] = V
    R['series'] = dict(t=t, fe=fe, tu=tu, fx=up['fx'], dev=dev, task=task_segs)
    return R


def main(out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    res = {k: analyse(k, *v) for k, v in RUNS.items() if os.path.exists(os.path.join(v[0], v[1]))}
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for k, R in res.items():
        s = R['series']
        ax[0].plot(s['t'], s['fe'][:, 0], lw=0.7, label=f'{k} external_wrench f_x [LOG]')
        ax[1].plot(s['tu'], s['fx'], lw=0.5, label=f'{k} uavpos fx [LOG] (= F quasi-static, COMPUTED)')
        ax[2].plot(s['tu'], s['dev'] * 1e3, lw=0.7, label=f'{k} uavpos xd - maneuver x [LOG]')
    for a_, yl in zip(ax, ('f_x [N]', 'fx [N]', 'AF dev x [mm]')):
        a_.set_ylabel(yl); a_.legend(fontsize=7)
    ax[2].set_xlabel('t [s] since first pom.log sample of each run')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'compare.png'), dpi=110); plt.close(fig)
    for R in res.values():
        R.pop('series')
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(res, f, indent=1, default=str)
    return res


if __name__ == '__main__':
    print(json.dumps(main(), indent=1, default=str)[:30000])
