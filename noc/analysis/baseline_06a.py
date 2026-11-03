"""
Baseline analysis of the 06a physical-interaction Gazebo run.

Reads  logs/06a-physical-interaction/gazebo/*.log + simulation_data.npz
Writes noc/results/00_baseline/metrics.json, log_inventory.json, *.png

Everything here is post-processing. Quantities are tagged:
  [LOG]       value read directly from a component log (a logged signal, which may
              itself be an estimate, e.g. pom state)
  [COMPUTED]  derived offline from logged signals + model assumptions

Usage (host):  .venv/bin/python -m noc.analysis.baseline_06a [log_dir]
"""

import hashlib
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis.logs import read_dir
from noc.common import params as P

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs', '06a-physical-interaction', 'gazebo')
OUT_DIR = os.path.join(REPO, 'noc', 'results', '00_baseline')

# --- analysis settings (not physical parameters) ----------------------------
SETTLE_S = 1.0          # ignore this long after a static reference begins
STATIC_V_TOL = 1e-6     # maneuver |v| below this = static reference
ROTOR_NEAR_FRAC = 0.10
X_TASK_NOM = 1.5       # maneuver nominal x above this = contact task (06a BODY_X_CONTACT = 1.55)
X_TASK_BODY = 1.3      # and pom/opti body x above this  # "near a limit" = within 10 % of (wmax - wmin) of the limit
N_ROTORS = P.N_ROTORS


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def rpy_to_R(r, p, y):
    """ZYX Euler (yaw-pitch-roll) -> rotation matrices, shape (N,3,3)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R


def stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return None
    return dict(n=int(x.size), mean=float(x.mean()), std=float(x.std()),
                rms=float(np.sqrt(np.mean(x ** 2))), min=float(x.min()),
                max=float(x.max()), p95_abs=float(np.percentile(np.abs(x), 95)),
                max_abs=float(np.abs(x).max()))


def segments(mask, t, min_len=0.0):
    """Contiguous True runs of mask -> list of (t_start, t_end)."""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    d = np.diff(m)
    starts, ends = np.where(d == 1)[0], np.where(d == -1)[0] - 1
    return [(float(t[s]), float(t[e])) for s, e in zip(starts, ends) if t[e] - t[s] >= min_len]


def in_windows(t, windows):
    m = np.zeros_like(t, bool)
    for a, b in windows:
        m |= (t >= a) & (t <= b)
    return m


def main(log_dir=LOG_DIR, out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    L = read_dir(log_dir)
    M = {}

    # ------------------------------------------------------------------ inventory
    inv = {}
    for name in sorted(os.listdir(log_dir)):
        p = os.path.join(log_dir, name)
        e = dict(bytes=os.path.getsize(p), sha256=sha256(p))
        key = name[:-4] if name.endswith('.log') else None
        if key in L and L[key] is not None:
            lg = L[key]
            e.update(rows=lg.n, rows_dropped_col_mismatch=lg.n_bad, columns=len(lg.header))
            if lg.n:
                e.update(ts_first=float(lg['ts'][0]), ts_last=float(lg['ts'][-1]))
        inv[name] = e
    npz = np.load(os.path.join(log_dir, 'simulation_data.npz'))
    inv['simulation_data.npz']['arrays'] = {k: list(npz[k].shape) for k in npz.files}
    inv['simulation_data.npz']['all_zero_rows'] = {
        k: int(np.all(npz[k] == 0, axis=1).sum()) for k in npz.files if npz[k].ndim == 2}
    with open(os.path.join(out_dir, 'log_inventory.json'), 'w') as f:
        json.dump(inv, f, indent=1)

    pom, up, ua, mv, op, rc = (L[k] for k in ('pom', 'uavpos', 'uavatt', 'maneuver', 'opti', 'rotorcraft'))
    T0 = pom['ts'][0]  # common time origin: first pom sample
    M['t0_unix'] = float(T0)
    tp = pom['ts'] - T0

    # ------------------------------------------------------------------ phases
    tm = mv['ts'] - T0
    vnom = np.sqrt(mv['vx'] ** 2 + mv['vy'] ** 2 + mv['vz'] ** 2)
    static = vnom < STATIC_V_TOL
    nom_i = lambda tq, c: np.interp(tq, tm, mv[c])

    # contact geometry from pom (estimate) and opti (Gazebo mocap)  [COMPUTED]
    def tip_penetration(lg):
        R = rpy_to_R(lg['roll'], lg['pitch'], lg['yaw'])
        p = np.column_stack([lg['x'], lg['y'], lg['z']])
        c = p + R @ P.TIP_CENTER_B
        return c, c[:, 0] + P.TIP_RADIUS - P.X_WALL  # sphere surface vs wall face (normal +x)

    c_pom, pen_pom = tip_penetration(pom)
    c_opt, pen_opt = tip_penetration(op)
    to = op['ts'] - T0
    M['geom_contact_segments_opti'] = segments(pen_opt > 0, to, min_len=0.05)
    # Contact TASK phase: nominal reference beyond reach (x_nom > X_TASK_NOM) and body
    # actually at the wall (x > X_TASK_BODY). Geometric penetration from pom is not
    # used for phase detection: pom-opti position noise (~1.6 mm std) exceeds it.
    task_p = (nom_i(tp, 'x') > X_TASK_NOM) & (pom['x'] > X_TASK_BODY)
    task_o = (nom_i(to, 'x') > X_TASK_NOM) & (op['x'] > X_TASK_BODY)
    contact = task_p  # name kept: "contact" = contact task phase
    contact_segs = segments(task_p, tp, min_len=0.5)
    M['contact_task_segments'] = contact_segs
    # steady contact: task phase AND nominal static, after SETTLE_S
    steady_ws = []
    for a, b in segments(static & (mv['x'] > X_TASK_NOM), tm, min_len=SETTLE_S + 0.5):
        steady_ws.append((a + SETTLE_S, b))
    M['steady_contact_windows'] = steady_ws
    M['pom_minus_opti'] = {c: stats(np.interp(op['ts'], pom['ts'], pom[c]) - op[c])
                           for c in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}

    # static-reference windows (free flight, no contact), settled
    hover_ws = []
    for a, b in segments(static & (mv['z'] > 0.5), tm, min_len=SETTLE_S + 0.5):
        a2 = a + SETTLE_S
        if not np.any(in_windows(tp, [(a2, b)]) & contact):
            hover_ws.append((a2, b, float(nom_i(a2, 'x')), float(nom_i(a2, 'y')), float(nom_i(a2, 'z'))))
    M['hover_windows'] = hover_ws

    # ------------------------------------------------------------------ 1 hover
    H = []
    for a, b, xr, yr, zr in hover_ws:
        m = (tp >= a) & (tp <= b)
        mo = (to >= a) & (to <= b)
        mu = (up['ts'] - T0 >= a) & (up['ts'] - T0 <= b)
        mua = (ua['ts'] - T0 >= a) & (ua['ts'] - T0 <= b)
        H.append(dict(
            window_s=[a, b], ref=[xr, yr, zr],
            pom_pos_minus_ref={c: stats(pom[c][m] - r) for c, r in zip('xyz', (xr, yr, zr))},
            opti_pos_minus_ref={c: stats(op[c][mo] - r) for c, r in zip('xyz', (xr, yr, zr))},
            pom_att_deg={c: stats(np.degrees(pom[c][m])) for c in ('roll', 'pitch', 'yaw')},
            pom_vel={c: stats(pom[c][m]) for c in ('vx', 'vy', 'vz')},
            pom_avel_deg_s={c: stats(np.degrees(pom[c][m])) for c in ('wx', 'wy', 'wz')},
            uavatt_e_r_deg={c: stats(np.degrees(ua[c][mua])) for c in ('e_rx', 'e_ry', 'e_rz')},
            uavpos_f_cmd={c: stats(up[c][mu]) for c in ('fx', 'fy', 'fz')},
        ))
    M['hover'] = H

    # ------------------------------------------------------------------ 2 tracking
    tu = up['ts'] - T0
    flying = (np.interp(tu, tp, pom['z']) > 0.5)
    cu = np.interp(tu, tp, contact.astype(float)) > 0.5
    e = {c: up['e_' + c] for c in 'xyz'}
    en = np.sqrt(e['x'] ** 2 + e['y'] ** 2 + e['z'] ** 2)
    # pom vs maneuver nominal (before admittance filter)
    en_nom = {c: pom[c] - nom_i(tp, c) for c in 'xyz'}
    fly_p = pom['z'] > 0.5
    T = {}
    for lbl, mu, mp in (('flight_no_contact', flying & ~cu, fly_p & ~contact),
                        ('contact', flying & cu, fly_p & contact)):
        T[lbl] = dict(
            uavpos_e_vs_phynt_ref={**{c: stats(e[c][mu]) for c in 'xyz'}, 'norm': stats(en[mu])},
            uavpos_e_v={c: stats(up['e_v' + c][mu]) for c in 'xyz'},
            pom_minus_maneuver_nominal={c: stats(en_nom[c][mp]) for c in 'xyz'},
            duration_s=float(mu.mean() * (tu[-1] - tu[0])),
        )
    M['tracking'] = T

    # ------------------------------------------------------------------ 3 f_ext
    F = {}
    F['phynt_log_rows'] = L['phynt'].n
    F['npz_f_ext_all_zero'] = bool(np.all(npz['f_ext'] == 0))
    F['npz_tau_ext_all_zero'] = bool(np.all(npz['tau_ext'] == 0))
    F['npz_pos_all_zero_rows'] = int(np.all(npz['pos'] == 0, axis=1).sum())
    F['npz_pos_d_all_zero_rows'] = int(np.all(npz['pos_d'] == 0, axis=1).sum())
    F['npz_p_nom_all_zero_rows'] = int(np.all(npz['p_nom'] == 0, axis=1).sum())
    # [COMPUTED] controller-side reconstruction, world frame:
    #   m a = f_cmd + f_ext - m g e_z   ->   f_ext_hat = m a_pom - f_cmd + m g e_z
    # Assumes: uavpos f is the world-frame total force command and is realised exactly
    # by the rotors (no saturation, exact allocation/mass); a from pom estimate.
    a_u = np.column_stack([np.interp(tu, tp, pom[c]) for c in ('ax', 'ay', 'az')])
    f_cmd = np.column_stack([up['fx'], up['fy'], up['fz']])
    f_hat = P.MASS * a_u - f_cmd + np.array([0, 0, P.MASS * P.G_ACC])
    F['f_ext_hat_free_flight'] = {c: stats(f_hat[flying & ~cu, i]) for i, c in enumerate('xyz')}
    F['f_ext_hat_contact'] = {c: stats(f_hat[flying & cu, i]) for i, c in enumerate('xyz')}
    # quasi-static variant (a = 0) during contact
    fqs = -f_cmd + np.array([0, 0, P.MASS * P.G_ACC])
    F['f_ext_hat_quasistatic_contact'] = {c: stats(fqs[flying & cu, i]) for i, c in enumerate('xyz')}
    M['f_ext'] = F

    # ------------------------------------------------------------------ 4 spring vs f_ext
    # [COMPUTED] idealised spring: F = K_WALL * max(pen,0)  (+ variants)
    vx_tip = np.gradient(c_pom[:, 0], tp)
    f_spring = P.K_WALL * np.maximum(pen_pom, 0)
    f_spring_md = P.K_WALL * np.maximum(pen_pom - P.WALL_MIN_DEPTH, 0)
    f_spring_kd = np.where(pen_pom > 0, np.maximum(P.K_WALL * pen_pom + P.D_WALL * vx_tip, 0), 0)
    f_spring_o = P.K_WALL * np.maximum(pen_opt, 0)
    f_hat_x_p = np.interp(tp, tu, f_hat[:, 0])
    fqs_x_p = np.interp(tp, tu, fqs[:, 0])
    fqs_x_o = np.interp(to, tu, fqs[:, 0])
    S = dict(frac_pen_pos_pom=float(np.mean(pen_pom[task_p] > 0)),
             frac_pen_pos_opti=float(np.mean(pen_opt[task_o] > 0)),
             penetration_pom_m=stats(pen_pom[task_p]),
             penetration_opti_m=stats(pen_opt[task_o]),
             penetration_opti_pct_m=np.percentile(pen_opt[task_o], [1, 5, 50, 95, 99]).tolist(),
             penetration_pom_pct_m=np.percentile(pen_pom[task_p], [1, 5, 50, 95, 99]).tolist(),
             spring_pom_K_pen_N=stats(f_spring[task_p]),
             spring_pom_K_pen_minus_min_depth_N=stats(f_spring_md[task_p]),
             spring_pom_K_pen_plus_D_vel_N=stats(f_spring_kd[task_p]),
             spring_opti_K_pen_N=stats(f_spring_o[task_o]),
             minus_f_ext_hat_x_N=stats(-f_hat_x_p[task_p]),
             minus_f_ext_hat_quasistatic_x_N=stats(-fqs_x_p[task_p]))
    sp = in_windows(tp, steady_ws) & task_p
    so = in_windows(to, steady_ws) & task_o
    k_eff = float(np.median(-fqs_x_o[so]) / max(np.percentile(pen_opt[so], 99), 1e-9))
    S['steady'] = dict(
        n_pom=int(sp.sum()), n_opti=int(so.sum()),
        spring_pom_K_pen_N=stats(f_spring[sp]),
        spring_opti_K_pen_N=stats(f_spring_o[so]),
        minus_f_ext_hat_x_N=stats(-f_hat_x_p[sp]),
        minus_f_ext_hat_quasistatic_x_N=stats(-fqs_x_p[sp]),
        diff_spring_pom_minus_hat_qs_N=stats(f_spring[sp] + fqs_x_p[sp]),
        corr_spring_pom_vs_hat_qs=float(np.corrcoef(f_spring[sp], -fqs_x_p[sp])[0, 1]),
        penetration_opti_p99_m=float(np.percentile(pen_opt[so], 99)),
        # lower bound on effective wall stiffness: median quasi-static force / p99 opti penetration
        k_eff_lower_bound_N_per_m=k_eff,
        penetration_needed_at_K_WALL_m=float(np.median(-fqs_x_o[so]) / P.K_WALL),
        pom_pitch_deg=stats(np.degrees(pom['pitch'][sp])),
        pom_yaw_deg=stats(np.degrees(pom['yaw'][sp])),
    )
    S['phynt_ref_minus_nominal'] = {c: stats(up[c + 'd'][tu > tm[0]] - np.interp(tu[tu > tm[0]], tm, mv[c]))
                                    for c in 'xyz'}
    M['spring_vs_fext'] = S

    # ------------------------------------------------------------------ 5 rotors
    tr = rc['ts'] - T0
    cmd = np.column_stack([rc[f'cmd_v{i}'] for i in range(N_ROTORS)])
    meas = np.column_stack([rc[f'meas_v{i}'] for i in range(N_ROTORS)])
    span = P.OMEGA_MAX - P.OMEGA_MIN
    hi_th, lo_th = P.OMEGA_MAX - ROTOR_NEAR_FRAC * span, P.OMEGA_MIN + ROTOR_NEAR_FRAC * span
    fl_r = np.interp(tr, tp, pom['z']) > 0.5
    ct_r = np.interp(tr, tp, contact.astype(float)) > 0.5
    Rr = dict(wmin=P.OMEGA_MIN, wmax=P.OMEGA_MAX, near_hi=hi_th, near_lo=lo_th,
              cmd_nan_rows=int(np.isnan(cmd).any(axis=1).sum()), rows=int(len(tr)))
    for lbl, m in (('flight', fl_r), ('flight_no_contact', fl_r & ~ct_r), ('contact', fl_r & ct_r)):
        d = {}
        for name, arr in (('cmd', cmd), ('meas', meas)):
            a = arr[m]
            ok = np.isfinite(a)
            d[name] = dict(
                per_rotor_min=np.nanmin(a, 0).tolist(), per_rotor_max=np.nanmax(a, 0).tolist(),
                per_rotor_mean=np.nanmean(a, 0).tolist(),
                min_margin_to_wmax=float(P.OMEGA_MAX - np.nanmax(a)),
                min_margin_to_wmin=float(np.nanmin(a) - P.OMEGA_MIN),
                frac_samples_any_rotor_near_wmax=float(np.mean(np.any(ok & (a >= hi_th), 1))),
                frac_samples_any_rotor_near_wmin=float(np.mean(np.any(ok & (a <= lo_th), 1))),
                frac_samples_any_rotor_at_or_above_wmax=float(np.mean(np.any(ok & (a >= P.OMEGA_MAX), 1))),
                frac_samples_any_rotor_at_or_below_wmin=float(np.mean(np.any(ok & (a <= P.OMEGA_MIN), 1))),
            )
        d['max_abs_cmd_minus_meas'] = float(np.nanmax(np.abs(cmd[m] - meas[m])))
        Rr[lbl] = d
    # [COMPUTED] hover prediction from params.compute_G for reference
    G = P.compute_G()
    w2 = np.linalg.lstsq(G, np.array([0, 0, P.MASS * P.G_ACC, 0, 0, 0]), rcond=None)[0]
    Rr['computed_hover_speed_from_compute_G'] = np.sqrt(np.abs(w2)).tolist()
    M['rotors'] = Rr

    # ------------------------------------------------------------------ 6 loop period
    LP = {}
    for name in ('pom', 'uavpos', 'uavatt', 'maneuver', 'opti', 'rotorcraft'):
        ts = L[name]['ts']
        dt = np.diff(ts)
        pos_dt = dt[dt > 0]
        med = float(np.median(pos_dt))
        LP[name] = dict(
            rows=int(len(ts)), span_s=float(ts[-1] - ts[0]),
            mean_rate_hz=float((len(ts) - 1) / (ts[-1] - ts[0])),
            dt_median_ms=med * 1e3, dt_mean_ms=float(dt.mean() * 1e3),
            dt_std_ms=float(dt.std() * 1e3), dt_p99_ms=float(np.percentile(dt, 99) * 1e3),
            dt_max_ms=float(dt.max() * 1e3), dt_min_ms=float(dt.min() * 1e3),
            n_dt_zero=int((dt == 0).sum()), n_dt_negative=int((dt < 0).sum()),
            n_gaps_gt_2x_median=int((dt > 2 * med).sum()),
            n_gaps_gt_1p5x_median=int((dt > 1.5 * med).sum()),
            n_gaps_gt_20ms=int((dt > 0.020).sum()),
            frac_dt_in_0p9_1p1_ms=float(np.mean((dt >= 0.9e-3) & (dt <= 1.1e-3))),
            unique_ts_rate_hz=float((len(np.unique(ts)) - 1) / (ts[-1] - ts[0])),
        )
        if 'delay' in L[name].header:
            LP[name]['delay_col_s'] = stats(L[name]['delay'])
    LP['rotorcraft_reported_rates'] = {c: stats(rc[c][fl_r]) for c in ('imu_rate', 'motor_rate')}
    M['loop_period'] = LP

    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(M, f, indent=1)

    # ------------------------------------------------------------------ plots
    shade = lambda ax: [ax.axvspan(a, b, color='0.85', lw=0) for a, b in contact_segs]

    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for i, c in enumerate('xyz'):
        shade(ax[i])
        ax[i].plot(tm, mv[c], 'k--', lw=1, label='maneuver nominal [LOG]')
        ax[i].plot(tu, up[c + 'd'], 'C1', lw=1, label='uavpos ref = phynt AF output [LOG]')
        ax[i].plot(tp, pom[c], 'C0', lw=1, label='pom estimate [LOG]')
        ax[i].set_ylabel(c + ' [m]')
    ax[0].legend(fontsize=7, loc='upper left')
    shade(ax[3]); ax[3].plot(tu, en, 'C3', lw=0.7); ax[3].set_ylabel('|e_p| uavpos [m]')
    ax[3].set_xlabel('t [s] since first pom sample (grey = contact task phase)')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'tracking.png'), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    shade(ax[0])
    ax[0].plot(np.arange(len(npz['t'])) * 0.02, npz['f_ext'][:, 0], 'C3', lw=2,
               label='npz f_ext_x (phynt WO via genomix) — all 4000 rows = 0 (read failed)')
    ax[0].plot(tu, f_hat[:, 0], 'C0', lw=0.4, alpha=0.6, label='f_ext_hat_x = m a_pom - f_cmd + m g (COMPUTED)')
    ax[0].plot(tu, fqs[:, 0], 'k', lw=0.6, label='f_ext_hat_x quasi-static, a=0 (COMPUTED)')
    ax[0].set_ylabel('f_x [N]'); ax[0].legend(fontsize=7); ax[0].set_ylim(-8, 8)
    shade(ax[1])
    ax[1].plot(tp, pen_pom * 1e3, 'C0', lw=0.5, label='tip penetration from pom (COMPUTED)')
    ax[1].plot(to, pen_opt * 1e3, 'C1', lw=0.8, label='tip penetration from opti (COMPUTED)')
    ax[1].axhline(0, color='k', lw=0.5)
    ax[1].set_ylabel('penetration [mm]'); ax[1].set_ylim(-8, 8); ax[1].legend(fontsize=7)
    shade(ax[2])
    ax[2].plot(tp, f_spring, 'C2', lw=0.5, label='K_WALL*pen(pom) (COMPUTED, idealised)')
    ax[2].plot(to, f_spring_o, 'C1', lw=0.8, label='K_WALL*pen(opti) (COMPUTED, idealised)')
    ax[2].plot(tu, -fqs[:, 0], 'k', lw=0.6, label='-f_ext_hat_x quasi-static (COMPUTED)')
    ax[2].set_ylabel('wall normal force [N]'); ax[2].set_ylim(-1, 5); ax[2].legend(fontsize=7)
    ax[2].set_xlabel('t [s] since first pom sample; npz plotted on its nominal i*0.02 s grid (no wall clock); grey = contact task phase')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fext_spring.png'), dpi=110); plt.close(fig)
    fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for k, (arr, lbl) in enumerate(((cmd, 'cmd'), (meas, 'meas'))):
        shade(ax[k])
        for i in range(N_ROTORS):
            ax[k].plot(tr, arr[:, i], lw=0.6, label=f'rotor {i+1}')
        for v in (P.OMEGA_MIN, P.OMEGA_MAX):
            ax[k].axhline(v, color='r', ls='--', lw=1)
        for v in (lo_th, hi_th):
            ax[k].axhline(v, color='r', ls=':', lw=0.7)
        ax[k].set_ylabel(f'rotorcraft {lbl}_v [LOG]')
    ax[0].legend(fontsize=7, ncol=6)
    ax[1].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'rotors.png'), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    for name in ('pom', 'uavpos', 'uavatt', 'rotorcraft', 'opti', 'maneuver'):
        ts = L[name]['ts']
        ax.semilogy(ts[1:] - T0, np.maximum(np.diff(ts), 1e-6) * 1e3, '.', ms=1, label=name)
    ax.set_ylabel('dt [ms] (log rows)'); ax.set_xlabel('t [s]'); ax.legend(fontsize=7, markerscale=8)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'loop_period.png'), dpi=110); plt.close(fig)

    return M


if __name__ == '__main__':
    M = main(*(sys.argv[1:2] or []))
    print(json.dumps(M, indent=1)[:200000])
