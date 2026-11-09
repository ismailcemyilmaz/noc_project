"""
a04c_impulse_origin — post-hoc, flight-free analysis of the impulsive F/T contact signal in
t04 (logs/noc/t04_ft_check). Not gated. Inputs are the raw t04 logs only.

Time base: F/T header stamps are simulation time. They are mapped to the wall clock with a
0.1 s rolling lower envelope of (host - stamp), which matched pom contact edges within
0.016 / 0.064 s in noc/results/04_ft_check.md section 2 (the committed 2 s envelope did not).
Component logs are on the wall clock (assumption).

Usage: .venv/bin/python -m noc.analysis.a04c_impulse_origin [log_dir] [out_dir]
Writes <out_dir>/{metrics.json, impulse_origin.png}.
"""
import json
import os
import sys

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from noc.analysis import a04_ft_check as a
from noc.analysis.logs import read_dir
from noc.common import params as P

SEED = 0
SHORT_ENV_S = 0.1
THRESHOLDS_N = (0.2, 0.5, 1.0, 2.0)
THR_N = 0.5               # primary pulse threshold on |Fn - bias|
FS = 500.0                # sensor rate = physics rate
X_WALL = P.X_WALL
TIP_C = np.array([0.6, 0.0, -0.125])
TIP_R = 0.04
GAP_TOLS_M = (0.0, 0.0005, 0.001, 0.002)
ETA_HALF_S = 0.2          # event-triggered average half width
N_PERM = 2000
OUT = os.path.join(a.REPO, 'noc', 'results', '04b_impulse_origin')


def pulses(x, thr):
    """Runs of consecutive samples with x < -thr -> (start, stop-exclusive) index arrays."""
    m = np.concatenate([[0], (x < -thr).astype(np.int8), [0]])
    d = np.diff(m)
    return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


def pulse_stats(t, x, thr):
    s, e = pulses(x, thr)
    if s.size < 2:
        return dict(thr=thr, n=int(s.size))
    dur = (e - s) / FS
    ipi = np.diff(t[s])
    pk = np.array([x[i:j].min() for i, j in zip(s, e)])
    J = np.array([x[i:j].sum() / FS for i, j in zip(s, e)])
    return dict(thr=thr, n=int(s.size), rate_hz=float(s.size / (t[-1] - t[0])),
                ipi_median_s=float(np.median(ipi)), ipi_p05_s=float(np.percentile(ipi, 5)),
                ipi_p95_s=float(np.percentile(ipi, 95)), ipi_cv=float(ipi.std() / ipi.mean()),
                dur_median_ms=float(np.median(dur) * 1e3), dur_p95_ms=float(np.percentile(dur, 95) * 1e3),
                dur_max_ms=float(dur.max() * 1e3), frac_one_step=float(np.mean(e - s == 1)),
                frac_two_step=float(np.mean(e - s == 2)),
                peak_median_N=float(np.median(pk)), peak_min_N=float(pk.min()),
                impulse_median_Ns=float(np.median(J)), impulse_sum_Ns=float(J.sum()),
                mean_force_from_pulses_N=float(J.sum() / (t[-1] - t[0])))


def tip_state(pom):
    R = a.rpy_to_R(pom['roll'], pom['pitch'], pom['yaw'])
    r = R @ TIP_C
    v = np.column_stack([pom['vx'], pom['vy'], pom['vz']]) if 'vx' in pom else None
    w = np.column_stack([pom['wx'], pom['wy'], pom['wz']]) if 'wx' in pom else None
    return pom['x'] + r[:, 0] + TIP_R, r, v, w


def regress_law(y, *xs):
    A = np.column_stack(xs)
    ok = np.all(np.isfinite(A), axis=1) & np.isfinite(y)
    c, *_ = np.linalg.lstsq(A[ok], y[ok], rcond=None)
    res = y[ok] - A[ok] @ c
    return [float(v) for v in c], float(1 - res.var() / y[ok].var())


def run(log_dir):
    rng = np.random.default_rng(SEED)
    d, meta, events, A, info, _ = a.load(log_dir)
    L = read_dir(log_dir, names=('pom', 'uavpos', 'uavatt', 'maneuver', 'phynt', 'opti', 'rotorcraft'))
    W, G = a.window_walls(events), a.goto_walls(events)
    off = A['host'] - A['stamp']
    t = A['stamp'] + a.rolling_min(A['stamp'], off, SHORT_ENV_S / 2)
    F = np.column_stack([A['fx'], A['fy'], A['fz']])
    Fn = a.body_to_world_x(L, t, F)
    w1 = (t >= W['FREE1'][0]) & (t <= W['FREE1'][1])
    wc = (t >= W['CONTACT'][0]) & (t <= W['CONTACT'][1])
    b = float(Fn[w1].mean())
    x = Fn[wc] - b
    tc, stc = t[wc], A['stamp'][wc]
    S = dict(seed=SEED, short_env_s=SHORT_ENV_S, bias_N=b, n_contact=int(x.size),
             contact_sim_s=float(stc[-1] - stc[0]), contact_wall_s=float(tc[-1] - tc[0]),
             rtf_contact=float((stc[-1] - stc[0]) / (tc[-1] - tc[0])),
             stamp_step_ms=sorted(set(np.round(np.diff(A['stamp']) * 1e3, 3).tolist()))[:6],
             mean_N=float(x.mean()), median_N=float(np.median(x)))

    # ---- pulses (on sim time: repetition/duration are physics quantities)
    S['pulses_sim_time'] = {f'{th:g}': pulse_stats(stc, x, th) for th in THRESHOLDS_N}
    s, e = pulses(x, THR_N)
    ipi = np.diff(stc[s])

    # ---- spectrum in sim time
    f, P_x = signal.welch(x - x.mean(), fs=FS, nperseg=1024, detrend='constant')
    top = np.argsort(P_x)[-8:][::-1]
    train = np.zeros(x.size); train[s] = 1.0
    f_t, P_t = signal.welch(train - train.mean(), fs=FS, nperseg=1024, detrend='constant')
    top_t = np.argsort(P_t)[-8:][::-1]
    bands = ((0, 0.5), (0.5, 2), (2, 10), (10, 50), (50, 100), (100, 250.1))
    var = np.trapezoid(P_x, f)
    S['spectrum'] = dict(nperseg=1024, df_hz=float(f[1]), peaks_force_hz=f[top].tolist(), peaks_force_psd=P_x[top].tolist(),
                         peaks_train_hz=f_t[top_t].tolist(),
                         band_fraction={f'{lo:g}-{hi:g}': float(np.trapezoid(P_x[(f >= lo) & (f < hi)], f[(f >= lo) & (f < hi)]) / var)
                                        for lo, hi in bands},
                         flatness_0p5_250=float(np.exp(np.mean(np.log(P_x[f >= 0.5]))) / np.mean(P_x[f >= 0.5])))
    # is the pulse train periodic? autocorrelation of the train, excluding lag 0
    tr = train - train.mean()
    ac = signal.correlate(tr, tr, mode='full', method='fft')[tr.size - 1:]
    ac = ac / ac[0]
    lag_max = int(0.5 * FS)
    k = np.argsort(ac[1:lag_max])[-5:][::-1] + 1
    # 95 % band for iid Bernoulli (white)
    S['train_autocorr'] = dict(top_lags_ms=(k * 2.0).tolist(), top_values=ac[k].tolist(),
                               white_95pct_band=float(1.96 / np.sqrt(x.size)))
    # IPI as integer physics steps
    steps = np.round(ipi * FS).astype(int)
    u, c = np.unique(steps, return_counts=True)
    o = np.argsort(c)[::-1][:10]
    S['ipi_steps_top'] = [(int(u[i]), int(c[i])) for i in o]
    S['ipi_exponential_check'] = dict(mean_s=float(ipi.mean()), std_s=float(ipi.std()), cv=float(ipi.std() / ipi.mean()),
                                      note='cv = 1 for a memoryless (Poisson) process')

    # ---- controller / AF frequencies
    M = meta_af = 5.0; K = 10.0
    up, ua = L['uavpos'], L['uavatt']
    fc = (up['ts'] >= W['CONTACT'][0]) & (up['ts'] <= W['CONTACT'][1])
    upall = (up['ts'] >= W['FREE1'][0]) & (up['ts'] <= W['FREE2'][1])
    c_fx, r2fx = regress_law(up['fx'][upall], up['e_x'][upall], up['e_vx'][upall])
    aall = (ua['ts'] >= W['FREE1'][0]) & (ua['ts'] <= W['FREE2'][1])
    c_ty, r2ty = regress_law(ua['ty'][aall], ua['e_ry'][aall], ua['e_wy'][aall])
    Jyy = float(P.J[1, 1]) if hasattr(P, 'J') else None
    ts_rate = lambda ts: float(1.0 / np.median(np.diff(ts)))
    wx_af = float(np.sqrt(K / M))
    Kp, Kv = abs(c_fx[0]), abs(c_fx[1])
    m = float(P.MASS) if hasattr(P, 'MASS') else 2.72
    wp = float(np.sqrt(Kp / m))
    Kq, Kw = abs(c_ty[0]), abs(c_ty[1])
    wa = float(np.sqrt(Kq / Jyy)) if Jyy else None
    S['frequencies'] = dict(
        af=dict(K_x=K, M=M, omega_rad_s=wx_af, f_hz=wx_af / (2 * np.pi)),
        uavpos=dict(law_fx_regression=dict(Kp=c_fx[0], Kv=c_fx[1], r2=r2fx), m_kg=m, omega_rad_s=wp, f_hz=wp / (2 * np.pi),
                    zeta=Kv / (2 * np.sqrt(Kp * m)), log_rate_wall_hz=ts_rate(up['ts'][upall])),
        uavatt=dict(law_ty_regression=dict(Kq=c_ty[0], Kw=c_ty[1], r2=r2ty), Jyy=Jyy, omega_rad_s=wa,
                    f_hz=None if wa is None else wa / (2 * np.pi),
                    zeta=None if wa is None else Kw / (2 * np.sqrt(Kq * Jyy)), log_rate_wall_hz=ts_rate(ua['ts'][aall])),
        physics=dict(step_s=0.002, f_hz=500.0, nyquist_hz=250.0),
        optitrack_hz_sim=100.0,
        pulse_rate_hz=S['pulses_sim_time'][f'{THR_N:g}']['rate_hz'])

    # ---- geometry / contact-separation
    pom, op = L['pom'], L['opti']
    S['log_columns'] = dict(pom=list(pom.keys())[:24], opti=list(op.keys()))
    surf_p, r_p, v_p, w_p = tip_state(pom)
    def geom(src, name):
        ts = src['ts']
        mc = (ts >= W['CONTACT'][0]) & (ts <= W['CONTACT'][1])
        m1 = (ts >= W['FREE1'][0]) & (ts <= W['FREE1'][1])
        R = a.rpy_to_R(src['roll'], src['pitch'], src['yaw'])
        sf = src['x'] + (R @ TIP_C)[:, 0] + TIP_R
        gap = X_WALL - sf[mc]
        return dict(n=int(mc.sum()), rate_wall_hz=float(1 / np.median(np.diff(ts[mc]))),
                    free_std_mm=float(sf[m1].std(ddof=1) * 1e3), free_diff_std_mm=float(np.diff(sf[m1]).std(ddof=1) * 1e3),
                    gap_mean_mm=float(gap.mean() * 1e3), gap_median_mm=float(np.median(gap) * 1e3),
                    gap_std_mm=float(gap.std(ddof=1) * 1e3), gap_min_mm=float(gap.min() * 1e3),
                    gap_max_mm=float(gap.max() * 1e3),
                    gap_pcts_1_5_25_50_75_95_99_mm=(np.percentile(gap, [1, 5, 25, 50, 75, 95, 99]) * 1e3).tolist(),
                    frac_contact_vs_tol={f'{tol * 1e3:g}mm': float(np.mean(gap <= tol)) for tol in GAP_TOLS_M}), ts, sf
    S['geom_opti'], t_op, sf_op = geom(op, 'opti')
    S['geom_pom'], t_po, sf_po = geom(pom, 'pom')
    # F/T contact fractions (any contact sample)
    S['ft_state'] = {f'{th:g}N': float(np.mean(np.abs(x) > th)) for th in (0.05, 0.1, 0.2, 0.5, 1.0)}

    # ---- at pulses: tip velocity, position error, surface; event-triggered vs random
    r_ = a.rpy_to_R(pom['roll'], pom['pitch'], pom['yaw']) @ TIP_C
    vt = pom['vx'] + (w_p[:, 1] * r_[:, 2] - w_p[:, 2] * r_[:, 1])
    tpm = pom['ts']
    man = L['maneuver']
    ex = np.interp(tpm, man['ts'], man['x']) - pom['x']
    ex_up = up['e_x']
    sig = dict(tip_vx_mm_s=(tpm, vt * 1e3), surface_gap_mm=(t_op, (X_WALL - sf_op) * 1e3),
               ex_maneuver_mm=(tpm, ex * 1e3), ex_uavpos_mm=(up['ts'], ex_up * 1e3), fx_cmd_N=(up['ts'], up['fx']),
               ty_cmd_Nm=(ua['ts'], ua['ty']))
    t_ev = tc[s]
    grid = np.arange(-ETA_HALF_S, ETA_HALF_S + 1e-9, 0.01)
    eta = {}
    for name, (ts, y) in sig.items():
        at = np.interp(t_ev, ts, y)
        ref = np.interp(tc, ts, y)
        # permutation: random event times in the window, same count
        nulls = np.array([np.interp(rng.choice(tc, t_ev.size, replace=False), ts, y).mean() for _ in range(N_PERM)])
        p = float(np.mean(np.abs(nulls - ref.mean()) >= abs(at.mean() - ref.mean())))
        eta[name] = dict(at_pulse_mean=float(at.mean()), at_pulse_std=float(at.std(ddof=1)), window_mean=float(ref.mean()),
                         window_std=float(ref.std(ddof=1)), perm_p=p, null_95=[float(np.percentile(nulls, 2.5)), float(np.percentile(nulls, 97.5))],
                         eta_curve=[float(np.interp(t_ev + g, ts, y).mean()) for g in grid])
    S['event_triggered'] = dict(n_events=int(t_ev.size), grid_s=grid.tolist(), signals=eta, time_map_note='0.1 s envelope; jitter ~ 25 ms')

    # first impact state
    cg = [g for g in G if g[2] and abs(g[2][0] - 1.55) < 1e-6][0]
    FnB = Fn - b
    i0 = np.flatnonzero((t > cg[1] - 0.5) & (np.abs(FnB) > 1.0))[0]
    i_min = i0 + int(np.argmin(FnB[i0:i0 + 500]))
    t_hit = float(t[i_min])
    v0 = float(np.interp(t_hit - 0.02, tpm, vt))
    S['first_impact'] = dict(t_after_goto_s=float(t[i0] - cg[1]), peak_N=float(FnB[i_min]), peak_after_first_s=float(t[i_min] - t[i0]),
                             tip_vx_before_m_s=v0, tip_vx_at_m_s=float(np.interp(t_hit, tpm, vt)),
                             base_vx_before_m_s=float(np.interp(t_hit - 0.02, tpm, pom['vx'])),
                             body_x_before=float(np.interp(t_hit - 0.02, tpm, pom['x'])),
                             impulse_first_20ms_Ns=float(FnB[i0:i0 + 10].sum() / FS))
    # coherence: pulse train vs controller commands (sim-time resampled at 500 Hz, wall-mapped)
    coh = {}
    for name in ('fx_cmd_N', 'ty_cmd_Nm', 'tip_vx_mm_s'):
        ts, y = sig[name]
        yy = np.interp(tc, ts, y)
        f_c, C = signal.coherence(train - train.mean(), yy - yy.mean(), fs=FS, nperseg=512)
        coh[name] = dict(max_below_20hz=float(C[(f_c > 0) & (f_c < 20)].max()), mean_below_20hz=float(C[(f_c > 0) & (f_c < 20)].mean()))
    S['coherence_train'] = dict(signals=coh, nperseg=512, note='Welch segments %d; chance level ~ 1/segments' % (x.size // 256))
    return S, dict(t=tc, stc=stc, x=x, f=f, P=P_x, ipi=ipi, sig=sig, t_op=t_op, sf_op=sf_op, W=W, FnB=FnB, tall=t, grid=grid, eta=eta)


def plot(S, D, out):
    x, stc, f, P_x = D['x'], D['stc'], D['f'], D['P']
    W = D['W']
    fig, ax = plt.subplots(3, 2, figsize=(11, 9))
    ax[0, 0].plot(stc - stc[0], x, lw=0.4); ax[0, 0].set_xlim(4, 5); ax[0, 0].set_ylabel('Fn − bias [N]'); ax[0, 0].set_xlabel('sim time in CONTACT [s], 1 s zoom')
    ax[0, 1].hist(np.clip(D['ipi'] * 1e3, 0, 200), bins=100); ax[0, 1].set_xlabel('pulse spacing [ms], sim time (clipped 200)'); ax[0, 1].set_ylabel('count')
    ax[1, 0].loglog(f[1:], P_x[1:], lw=0.8)
    Fr = S['frequencies']
    for lbl, v, c in (('AF', Fr['af']['f_hz'], 'C1'), ('uavpos', Fr['uavpos']['f_hz'], 'C2'), ('uavatt', Fr['uavatt']['f_hz'], 'C3'), ('opti', 100, 'C4'), ('physics Nyquist', 250, 'C5')):
        if v:
            ax[1, 0].axvline(v, color=c, ls='--', lw=0.8, label=f'{lbl} {v:.3g} Hz')
    ax[1, 0].set_xlabel('Hz'); ax[1, 0].set_ylabel('PSD [N²/Hz]'); ax[1, 0].legend(fontsize=7)
    t_op, sf = D['t_op'], D['sf_op']
    m = (t_op >= W['CONTACT'][0]) & (t_op <= W['CONTACT'][1])
    ax[1, 1].hist((2.0 - sf[m]) * 1e3, bins=60); ax[1, 1].axvline(0, color='k', lw=0.8); ax[1, 1].set_xlabel('tip gap to wall from opti [mm] (>0 = no contact)')
    g = D['grid']; e = D['eta']
    ax[2, 0].plot(g * 1e3, e['tip_vx_mm_s']['eta_curve']); ax[2, 0].set_xlabel('t − pulse onset [ms]'); ax[2, 0].set_ylabel('mean tip vx [mm/s]')
    ax[2, 1].plot(g * 1e3, e['surface_gap_mm']['eta_curve'], label='gap [mm]'); ax[2, 1].plot(g * 1e3, e['ex_uavpos_mm']['eta_curve'], label='uavpos e_x [mm]')
    ax[2, 1].set_xlabel('t − pulse onset [ms]'); ax[2, 1].legend(fontsize=8)
    fig.suptitle('t04 F/T impulses (post-hoc, not gated)')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'impulse_origin.png'), dpi=110)
    plt.close(fig)


def main(log_dir=a.LOG_DIR, out_dir=OUT):
    os.makedirs(out_dir, exist_ok=True)
    S, D = run(log_dir)
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(S, f, indent=1, default=float)
    plot(S, D, out_dir)
    print(json.dumps(S, indent=1, default=float))


if __name__ == '__main__':
    main(*sys.argv[1:3])
