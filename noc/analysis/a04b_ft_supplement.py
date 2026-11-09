"""
a04b_ft_supplement — POST-HOC, NOT GATED supplement to a04_ft_check (t04).

Gate B1 is decided only by a04_ft_check with noc/config/t04_ft_criteria.json. This script
was written after the flight, after a04 showed that:
  (1) the F/T contact signal is impulsive, so raw RMSE/xcorr on a 100 Hz interp grid measures
      the impulses rather than estimator error;
  (2) at RTF ~ 0.64 the committed centred 2 s envelope maps F/T samples ~0.5 s early.

It re-evaluates the numbers under a short (0.1 s) envelope, checks both time maps against
pom-derived contact onset/release, and compares band-limited signals. Filter widths and the
0.1 s envelope are post-hoc choices; results are reported for several widths.

Usage: .venv/bin/python -m noc.analysis.a04b_ft_supplement [log_dir] [out_json]
"""
import json
import os
import sys

import numpy as np

from noc.analysis import a04_ft_check as a

SEED = 0
SHORT_ENV_S = 0.1
GRID_HZ = 100.0
MA_WIDTHS_S = (0.2, 0.5, 1.0)
LAG_MAX_S = 1.0
BOOT_BLOCK_S = 0.5
BOOT_N = 2000
KPXY = 5.0                # af_on_baseline.json, as in a02/a03
KX = 10.0
D0_GIVEN = 0.184          # a02/a03 series model
X_WALL = 2.0
TIP_C = [0.6, 0.0, -0.125]
TIP_R = 0.04
ONSET_N = 1.0             # |Fn - bias| threshold for "F/T sees contact"


def block_boot_ci(x, n_block, rng, n_boot=BOOT_N):
    """95 % CI of the mean by circular block bootstrap."""
    x = np.asarray(x, float)
    n = x.size
    k = int(np.ceil(n / n_block))
    c = np.concatenate([x, x[:n_block]]).cumsum()
    c = np.concatenate([[0.0], c])
    means = np.empty(n_boot)
    for i in range(n_boot):
        s = rng.integers(0, n, k)
        means[i] = (c[s + n_block] - c[s]).sum() / (k * n_block)
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def bin_mean(t, y, grid, dt):
    """Mean of y in [grid_i - dt/2, grid_i + dt/2); empty bins are interpolated."""
    idx = np.floor((t - (grid[0] - dt / 2)) / dt).astype(int)
    ok = (idx >= 0) & (idx < grid.size)
    n = np.bincount(idx[ok], minlength=grid.size)
    s = np.bincount(idx[ok], weights=y[ok], minlength=grid.size)
    have = n > 0
    return np.interp(grid, grid[have], s[have] / n[have])


def moving_avg(y, width_s, hz):
    k = max(int(round(width_s * hz)) | 1, 1)      # odd: symmetric, zero phase
    return np.convolve(np.pad(y, k // 2, mode='edge'), np.ones(k) / k, mode='valid')


def lag_int(ref, est, inner, kmax):
    """ref(t) vs est(t + k); `inner` is a slice with room for +/- kmax in est."""
    r = ref[inner] - ref[inner].mean()
    best = (0, -np.inf)
    for k in range(-kmax, kmax + 1):
        e = est[inner.start + k:inner.stop + k]
        e = e - e.mean()
        c = float(np.dot(r, e) / (np.linalg.norm(r) * np.linalg.norm(e) + 1e-12))
        if c > best[1]:
            best = (k, c)
    return best


def contact_edges(L, t, Fn, bias, W, G):
    """Onset and release from pom tip surface vs from F/T impulses."""
    pom = L['pom']
    cg = [g for g in G if g[2] and abs(g[2][0] - 1.55) < 1e-6][0]
    rg = [g for g in G if g[0] > cg[0]][0]
    R = a.rpy_to_R(pom['roll'], pom['pitch'], pom['yaw'])
    surf = pom['x'] + R[:, 0, :] @ np.asarray(TIP_C) + TIP_R
    ts = pom['ts']
    level = float(np.median(surf[(ts >= W['CONTACT'][0]) & (ts <= W['CONTACT'][1])])) - 0.002
    after = np.flatnonzero((ts > cg[1]) & (surf >= level))
    t_on_pom = float(ts[after[0]])
    back = np.flatnonzero((ts > rg[1]) & (surf < level))
    t_off_pom = float(ts[back[0]])
    hit = np.flatnonzero((t > cg[1] - 1.0) & (np.abs(Fn - bias) > ONSET_N))
    t_on_ft = float(t[hit[0]])
    hit2 = np.flatnonzero((t < rg[1] + 8.0) & (t > rg[1] - 1.0) & (np.abs(Fn - bias) > ONSET_N))
    t_off_ft = float(t[hit2[-1]])
    return dict(surface_level_m=level, on_pom=t_on_pom, on_ft=t_on_ft, on_ft_minus_pom_s=t_on_ft - t_on_pom,
                off_pom=t_off_pom, off_ft=t_off_ft, off_ft_minus_pom_s=t_off_ft - t_off_pom)


def run(log_dir):
    rng = np.random.default_rng(SEED)
    d, meta, events, A, info, L = a.load(log_dir)
    W, G = a.window_walls(events), a.goto_walls(events)
    F = np.column_stack([A['fx'], A['fy'], A['fz']])
    off = A['host'] - A['stamp']
    o = np.argsort(A['stamp'], kind='stable')
    env = np.empty_like(off)
    env[o] = a.rolling_min(A['stamp'][o], off[o], SHORT_ENV_S / 2)
    maps = dict(committed_2s=a.ft_wall_time(A)[0], short_0p1s=A['stamp'] + env)
    cg = [g for g in G if g[2] and abs(g[2][0] - 1.55) < 1e-6][0]
    rg = [g for g in G if g[0] > cg[0]][0]

    pom, man, ph, up = L['pom'], L['maneuver'], L['phynt'], L['uavpos']
    cw = W['CONTACT']
    Ep = {'WO efx': (ph['ts'], ph['efx']), '-uavpos fx': (up['ts'], -up['fx'])}
    est_mean = {n: float(y[(t >= cw[0]) & (t <= cw[1])].mean()) for n, (t, y) in Ep.items()}

    tu = up['ts'][(up['ts'] >= cw[0]) & (up['ts'] <= cw[1])]
    d0m = np.interp(tu, man['ts'], man['x']) - np.interp(tu, pom['ts'], pom['x'])
    d0_meas = float(d0m.mean())
    series = lambda d0: d0 / (1.0 / KPXY + 1.0 / KX)
    S = dict(seed=SEED, short_env_s=SHORT_ENV_S, boot=dict(block_s=BOOT_BLOCK_S, n=BOOT_N),
             series=dict(Kpxy=KPXY, Kx=KX, d0_given=D0_GIVEN, d0_measured=d0_meas, d0_measured_std=float(d0m.std(ddof=1)),
                         F_d0_given=series(D0_GIVEN), F_d0_measured=series(d0_meas)),
             est_contact_mean=est_mean, maps={})

    for name, t in maps.items():
        Fn = a.body_to_world_x(L, t, F)
        win = lambda n: (t >= W[n][0]) & (t <= W[n][1])
        w1, wc, w2 = win('FREE1'), win('CONTACT'), win('FREE2')
        bias, fstd = float(Fn[w1].mean()), float(Fn[w1].std(ddof=1))
        x = Fn[wc] - bias
        nb = int(round(BOOT_BLOCK_S * x.size / (cw[1] - cw[0])))
        ci = block_boot_ci(x, nb, rng)
        mc = float(x.mean())
        dpre = float(Fn[wc].mean() - Fn[w1].mean())
        dpost = float(Fn[wc].mean() - Fn[w2].mean())
        sub = [float(x[i].mean()) for i in np.array_split(np.arange(x.size), 5)]
        M = dict(bias=bias, free_std=fstd, n_contact=int(x.size), contact_mean=mc, contact_ci95=ci,
                 contact_median=float(np.median(x)), contact_std=float(x.std(ddof=1)),
                 frac_within_0p05N=float(np.mean(np.abs(x) < 0.05)), frac_below_minus1N=float(np.mean(x < -1.0)),
                 contact_min=float(x.min()), subwindow_means_2s=sub,
                 C4_ratio=abs(mc) / fstd, C5_delta_pre=dpre, C5_delta_post=dpost, C5_ratio=dpre / dpost,
                 C4_C5_hold=bool(abs(mc) > 5 * fstd and np.sign(dpre) == np.sign(dpost) and 1 / 3 <= dpre / dpost <= 3),
                 edges=contact_edges(L, t, Fn, bias, W, G))
        M['series_error'] = {k: dict(model_N=v, err_N=v - abs(mc), err_pct_of_ft=100 * (v - abs(mc)) / abs(mc),
                                     ci95_ft_abs=[abs(ci[1]), abs(ci[0])],
                                     model_inside_ci=bool(abs(ci[1]) <= v <= abs(ci[0])))
                             for k, v in (('d0_given', series(D0_GIVEN)), ('d0_measured', series(d0_meas)))}
        M['est_bias_contact'] = {n: dict(est_mean=m, est_minus_ft=m - mc, ci95=[m - ci[1], m - ci[0]])
                                 for n, m in est_mean.items()}

        # band-limited comparison
        lo, hi = cg[1] - 2.0, rg[1] + 5.0
        kmax = int(round(LAG_MAX_S * GRID_HZ))
        pad = kmax + int(GRID_HZ * max(MA_WIDTHS_S)) + 2
        grid = np.arange(lo - pad / GRID_HZ, hi + pad / GRID_HZ, 1.0 / GRID_HZ)
        inner = slice(pad, grid.size - pad)
        ft0 = bin_mean(t, Fn - bias, grid, 1.0 / GRID_HZ)
        M['band_limited'] = {}
        for w in MA_WIDTHS_S:
            ref = moving_avg(ft0, w, GRID_HZ)
            mask_c = (grid[inner] >= cw[0]) & (grid[inner] <= cw[1])
            out = {}
            for n, (et, ey) in Ep.items():
                e = moving_avg(np.interp(grid, et, ey), w, GRID_HZ)
                k, c = lag_int(ref, e, inner, kmax)
                d0_ = e[inner] - ref[inner]
                dk = e[inner.start + k:inner.stop + k] - ref[inner]
                out[n] = dict(lag_s=k / GRID_HZ, corr=c, rmse_span_lag=float(np.sqrt(np.mean(dk ** 2))),
                              rmse_span_nolag=float(np.sqrt(np.mean(d0_ ** 2))),
                              rmse_contact_lag=float(np.sqrt(np.mean(dk[mask_c] ** 2))),
                              rmse_contact_nolag=float(np.sqrt(np.mean(d0_[mask_c] ** 2))),
                              mean_err_contact_nolag=float(np.mean(d0_[mask_c])))
            M['band_limited'][f'{w:g}'] = out
        S['maps'][name] = M
    return S


def main(log_dir=a.LOG_DIR, out=None):
    S = run(log_dir)
    txt = json.dumps(S, indent=1, default=float)
    if out:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            f.write(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main(*sys.argv[1:3])
