"""
a00_diag — analysis of the t00_diag run (logs/noc/t00/).

Questions:
  Q1  Does the phynt wrench observer (external_wrench) see the contact force?
  Q2  Is the AF deviation (phynt desired - phynt reference) consistent with K_x and external_wrench?
  Q3  Why is phynt.log empty?
  Q4  Is the contact force explained by Kpxy * e_x?
Plus: read errors per source, Python loop timing.

Tags: [LOG] = logged signal (npz from genomix ports, or component .log),
      [COMPUTED] = derived offline with stated assumptions.

Usage (host): .venv/bin/python -m noc.analysis.a00_diag [log_dir]
Writes noc/results/00b_diagnosis/{metrics.json, *.png}
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
from noc.common import params as P

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs', 'noc', 't00')
OUT_DIR = os.path.join(REPO, 'noc', 'results', '00b_diagnosis')

# 06a setup() values (src/06a-physical-interaction/model_hexa_fa.py), used only as references
KPXY_06A = 5.0
KVXY_06A = 7.0
AF_KX_06A = 10.0

X_TASK_NOM, X_TASK_BODY = 1.5, 1.3
SETTLE_S = 1.0
STATIC_V_TOL = 1e-6


def load(log_dir):
    d = dict(np.load(os.path.join(log_dir, 't00_diag.npz'), allow_pickle=False))
    meta = json.loads(str(d.pop('meta_json')))
    events = json.loads(str(d.pop('events_json')))
    snaps = json.loads(str(d.pop('snapshots_json')))
    return d, meta, events, snaps


def frac(x):
    x = np.asarray(x, bool)
    return float(x.mean()) if x.size else None


def main(log_dir=LOG_DIR, out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    d, meta, events, snaps = load(log_dir)
    L = read_dir(log_dir)
    M = {'meta': meta, 'events': events}
    M['inventory'] = {f: dict(bytes=os.path.getsize(os.path.join(log_dir, f)),
                              sha256=sha256(os.path.join(log_dir, f)))
                      for f in sorted(os.listdir(log_dir))}
    M['component_log_rows'] = {k: (None if v is None else v.n) for k, v in L.items()}

    tw = d['t_wall']
    T0 = L['pom']['ts'][0] if L.get('pom') is not None and L['pom'].n else tw[0]
    t = tw - T0
    M['t0_unix'] = float(T0)

    # ---------------------------------------------------------------- read errors / timing
    srcs = ('pom', 'phynt_ref', 'phynt_des', 'man_des', 'ext_wrench', 'wrench_meas')
    E = {}
    for s in srcs:
        err = d[f'{s}.error'].astype(str)
        bad = err != ''
        msgs, counts = np.unique(err[bad], return_counts=True)
        E[s] = dict(n_err=int(bad.sum()), first_err_t=(float(t[bad][0]) if bad.any() else None),
                    messages={str(m): int(c) for m, c in zip(msgs, counts)},
                    valid_frac={k.split('.valid.')[1]: frac(d[k]) for k in d if k.startswith(f'{s}.valid.')})
    M['read_errors'] = E
    dtw = np.diff(tw)
    M['python_loop'] = dict(dt_wall_ms=stats(dtw * 1e3), late_s=stats(d['late_s']),
                            read_ms=stats(d['read_s'] * 1e3), n_samples=int(len(tw)))

    # ---------------------------------------------------------------- phases (from npz ports)
    pos = d['pom.pos']
    xnom = d['man_des.pos'][:, 0]
    vnom = np.linalg.norm(d['man_des.vel'], axis=1)
    ok_nom = d['man_des.valid.pos'] & d['man_des.valid.vel']
    task = ok_nom & (xnom > X_TASK_NOM) & (pos[:, 0] > X_TASK_BODY)
    free = (pos[:, 2] > 0.5) & ~task & np.isfinite(pos[:, 2])
    steady_ws = [(a + SETTLE_S, b) for a, b in
                 segments(task & (vnom < STATIC_V_TOL), t, min_len=SETTLE_S + 0.5)]
    steady = in_windows(t, steady_ws) & task
    M['phases'] = dict(task_segments=segments(task, t, min_len=0.5), steady_windows=steady_ws,
                       n_free=int(free.sum()), n_task=int(task.sum()), n_steady=int(steady.sum()))

    # ---------------------------------------------------------------- [COMPUTED] controller-side f_hat
    up = L.get('uavpos')
    have_up = up is not None and up.n > 0
    if have_up:
        tu = up['ts'] - T0
        fqs_x = np.interp(t, tu, -up['fx'])               # quasi-static: f_ext = -f_cmd + m g e_z
        ex = np.interp(t, tu, up['e_x'])
        evx = np.interp(t, tu, up['e_vx'])
        fcx = np.interp(t, tu, up['fx'])
    else:
        fqs_x = ex = evx = fcx = np.full_like(t, np.nan)

    # ---------------------------------------------------------------- Q1 WO sees contact?
    fe = d['ext_wrench.force']
    fm = d['wrench_meas.force']
    Q1 = dict(
        ext_force_valid_frac=frac(d['ext_wrench.valid.force']),
        ext_force_free={c: stats(fe[free, i]) for i, c in enumerate('xyz')},
        ext_force_task={c: stats(fe[task, i]) for i, c in enumerate('xyz')},
        ext_force_steady={c: stats(fe[steady, i]) for i, c in enumerate('xyz')},
        wrench_meas_valid_frac=frac(d['wrench_meas.valid.force']),
        wrench_meas_free={c: stats(fm[free, i]) for i, c in enumerate('xyz')},
        wrench_meas_steady={c: stats(fm[steady, i]) for i, c in enumerate('xyz')},
        minus_f_hat_qs_x_steady=stats(fqs_x[steady]) if have_up else None,  # [COMPUTED], sign: f_ext_x
    )
    fs, ff = Q1['ext_force_steady']['x'], Q1['ext_force_free']['x']
    if fs and ff:
        Q1['steady_minus_free_x_N'] = fs['mean'] - ff['mean']
        Q1['steady_minus_free_x_in_free_std'] = (fs['mean'] - ff['mean']) / max(ff['std'], 1e-9)
    if have_up and steady.any():
        ok = steady & np.isfinite(fe[:, 0]) & np.isfinite(fqs_x)
        if ok.sum() > 2:
            Q1['corr_ext_x_vs_f_hat_qs_x_steady'] = float(np.corrcoef(fe[ok, 0], fqs_x[ok])[0, 1])
            Q1['ext_x_minus_f_hat_qs_x_steady'] = stats(fe[ok, 0] - fqs_x[ok])
    # observer bias after set_wo_zero
    wo_snaps = {k: v.get('get_wo_parameters') for k, v in snaps.items()}
    Q1['wo_parameters_snapshots'] = wo_snaps
    M['Q1_wo_sees_contact'] = Q1

    # ---------------------------------------------------------------- Q2 AF deviation vs K_x, f_ext
    dev = d['phynt_des.pos'] - d['phynt_ref.pos']
    af = {k: (v.get('get_af_parameters') or {}).get('af') for k, v in snaps.items()}
    Kx = None
    for k in ('contact_1', 'after_start', 'before_stop'):
        if af.get(k) and af[k].get('K'):
            Kx = float(af[k]['K'][0]); break
    Q2 = dict(af_parameters_snapshots=af, K_x_used=Kx,
              dev_free={c: stats(dev[free, i]) for i, c in enumerate('xyz')},
              dev_task={c: stats(dev[task, i]) for i, c in enumerate('xyz')},
              dev_steady={c: stats(dev[steady, i]) for i, c in enumerate('xyz')})
    if Kx and steady.any():
        # [COMPUTED] static admittance: K_x * dev_x = f_x(ext) (+ af.force offset), sign convention unverified
        ok = steady & np.isfinite(dev[:, 0]) & np.isfinite(fe[:, 0])
        Q2['K_x_times_dev_x_steady_N'] = stats(Kx * dev[ok, 0])
        Q2['ext_force_x_steady_N'] = stats(fe[ok, 0])
        Q2['expected_dev_x_from_ext_m'] = stats(fe[ok, 0] / Kx)
        if ok.sum() > 2 and np.std(fe[ok, 0]) > 0 and np.std(dev[ok, 0]) > 0:
            Q2['corr_dev_x_vs_ext_x'] = float(np.corrcoef(dev[ok, 0], fe[ok, 0])[0, 1])
    M['Q2_af_consistency'] = Q2

    # ---------------------------------------------------------------- Q3 phynt.log
    Q3 = dict(
        log_info={k: v.get('log_info') for k, v in snaps.items()},
        tmp_phynt_log_bytes={k: v.get('tmp_phynt_log_bytes') for k, v in snaps.items()},
        tmp_phynt_log_bytes_after_stop=meta.get('tmp_phynt_log_bytes_after_stop'),
        out_phynt_log_bytes=meta.get('out_phynt_log_bytes'),
        out_phynt_log_rows=M['component_log_rows'].get('phynt'),
    )
    M['Q3_phynt_log'] = Q3

    # ---------------------------------------------------------------- Q4 Kpxy * e_x
    if have_up:
        Q4 = dict(Kpxy_06a=KPXY_06A, Kvxy_06a=KVXY_06A, mass=P.MASS)
        Q4['e_x_steady_m'] = stats(ex[steady])
        Q4['f_cmd_x_steady_N'] = stats(fcx[steady])
        Q4['Kpxy_abs_ex_steady_N'] = stats(KPXY_06A * np.abs(ex[steady]))
        Q4['m_Kpxy_abs_ex_steady_N'] = stats(P.MASS * KPXY_06A * np.abs(ex[steady]))
        Q4['minus_ext_force_x_steady_N'] = stats(-fe[steady, 0])
        # [COMPUTED] least squares on uavpos log rows in flight: fx = a*e_x + b*e_vx + c
        zf = np.interp(tu, t, pos[:, 2]) > 0.5
        A = np.column_stack([up['e_x'][zf], up['e_vx'][zf], np.ones(zf.sum())])
        coef, *_ = np.linalg.lstsq(A, up['fx'][zf], rcond=None)
        res = up['fx'][zf] - A @ coef
        Q4['fit_fx_eq_a_ex_b_evx_c'] = dict(a=float(coef[0]), b=float(coef[1]), c=float(coef[2]),
                                           resid_rms=float(np.sqrt(np.mean(res ** 2))),
                                           n=int(zf.sum()))
        M['Q4_kp_ex'] = Q4
    else:
        M['Q4_kp_ex'] = 'uavpos.log missing or empty'

    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(M, f, indent=1, default=str)

    # ---------------------------------------------------------------- plots
    shade = lambda ax: [ax.axvspan(a, b, color='0.85', lw=0) for a, b in M['phases']['task_segments']]
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    shade(ax[0])
    for i, c in enumerate('xyz'):
        ax[0].plot(t, fe[:, i], lw=0.7, label=f'external_wrench f{c} [LOG]')
    ax[0].plot(t, fqs_x, 'k', lw=0.6, label='f_hat_x quasi-static = -uavpos fx (COMPUTED)')
    ax[0].set_ylabel('force [N]'); ax[0].legend(fontsize=7)
    shade(ax[1])
    for i, c in enumerate('xyz'):
        ax[1].plot(t, fm[:, i], lw=0.7, label=f'wrench_measure f{c} [LOG]')
    ax[1].set_ylabel('force [N]'); ax[1].legend(fontsize=7)
    shade(ax[2])
    for i, c in enumerate('xyz'):
        ax[2].plot(t, dev[:, i] * 1e3, lw=0.7, label=f'phynt desired - reference {c} [LOG]')
    if Kx:
        ax[2].plot(t, fe[:, 0] / Kx * 1e3, 'k--', lw=0.6, label='ext f_x / K_x (COMPUTED)')
    ax[2].set_ylabel('AF deviation [mm]'); ax[2].legend(fontsize=7)
    shade(ax[3])
    ax[3].plot(t, fcx, 'C0', lw=0.6, label='uavpos fx [LOG]')
    ax[3].plot(t, KPXY_06A * ex, 'C3', lw=0.6, label='Kpxy * e_x (COMPUTED, sign as logged)')
    ax[3].set_ylabel('[N]'); ax[3].legend(fontsize=7)
    ax[3].set_xlabel('t [s] since first pom.log sample; grey = contact task phase')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'diag.png'), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    for k, s in enumerate(srcs):
        bad = d[f'{s}.error'].astype(str) != ''
        ax[0].plot(t[bad], np.full(bad.sum(), k), '|', ms=8)
    ax[0].set_yticks(range(len(srcs))); ax[0].set_yticklabels(srcs, fontsize=7)
    ax[0].set_title('port read errors [LOG]', fontsize=9)
    ax[1].plot(t[1:], dtw * 1e3, lw=0.5); ax[1].set_ylabel('python loop dt [ms]')
    ax[1].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'io_timing.png'), dpi=110); plt.close(fig)
    return M


if __name__ == '__main__':
    M = main(*(sys.argv[1:2] or []))
    print(json.dumps(M, indent=1, default=str)[:20000])
