"""
t00_diag — diagnostic re-run of the 06a contact scenario with correct genomix I/O.

Reuses setup(), start(), stop(), waypoints and the components from
src/06a-physical-interaction/model_hexa_fa.py WITHOUT modifying that file.
Only the module attribute LOG_DIR is redirected so that stop() copies the
component logs to logs/noc/t00/ instead of overwriting logs/06a-.../gazebo/.

Records at 50 Hz target (06a timing):
  wall/monotonic time, loop lateness, read duration,
  pom frame (pos/att/vel/avel/acc), phynt reference, phynt desired,
  maneuver desired, phynt external_wrench, phynt wrench_measure
  (+ port timestamps and per-field valid flags; None -> NaN + valid=False).
Snapshots of phynt get_af_parameters / get_wo_parameters / get_body_parameters /
log_info (+ /tmp/phynt.log size) after start(), during steady contact and before stop().

Errors: per-sample port read failures are recorded explicitly (columns
'<src>.error' = exception text, values NaN, valid False; counts in meta.json) and
the run continues. Any other exception stops the rotors, saves partial data and
is re-raised.

Run inside the container (T2), after simulation.sh is up and Gazebo Play pressed:
  cd /shared-workspace && python3 -m noc.tests.t00_diag
Output: logs/noc/t00/t00_diag.npz, snapshots.json, events.json, preflight.json,
        component logs (*.log) copied by 06a stop().
"""

import importlib.util
import json
import os
import subprocess
import sys
import time
import traceback

import numpy as np

from noc.common import telekyb_io as io

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't00')
MODEL_06A = os.path.join(REPO, 'src', '06a-physical-interaction', 'model_hexa_fa.py')

TF = 80.0            # same as 06a simulation()
DT = 0.02            # 50 Hz target
T_WO_ZERO = 20.0     # same as 06a
SNAPSHOT_TIMES = {'contact_1': 28.0, 'contact_2': 45.0}  # steady contact windows in baseline run
PHYNT_TMP_LOG = '/tmp/phynt.log'


def load_06a():
    spec = importlib.util.spec_from_file_location('model_hexa_fa', MODEL_06A)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # connects genomix, loads components
    mod.LOG_DIR = OUT_DIR                 # stop() copies logs here (06a file untouched)
    return mod


def file_size(path):
    return os.path.getsize(path) if os.path.exists(path) else None


def git_rev():
    r = subprocess.run(['git', '-C', REPO, 'rev-parse', 'HEAD'], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else f'unavailable: {r.stderr.strip()}'


def jdump(obj, name):
    with open(os.path.join(OUT_DIR, name), 'w') as f:
        json.dump(obj, f, indent=1, default=str)


def snapshot(m, label, t_nom):
    s = io.phynt_snapshot(m.phynt)
    s['t_nominal'] = t_nom
    s['t_wall'] = time.time()
    s['tmp_phynt_log_bytes'] = file_size(PHYNT_TMP_LOG)
    print(f'[snapshot {label}] {json.dumps(s, default=str)}', flush=True)
    return s


SOURCE_COMPONENT = {'pom': 'pom', 'phynt_ref': 'phynt', 'phynt_des': 'phynt',
                    'man_des': 'maneuver', 'ext_wrench': 'phynt', 'wrench_meas': 'phynt'}


def read_all(m, err_counts):
    """Read every source. A read that raises is NOT hidden: its exception text is
    returned in ``errors[src]``, printed (first 5 per source), counted, and the
    sample is all-NaN with valid=False. The run continues so the failure pattern
    over time is recorded instead of aborting on the first error."""
    t0 = time.monotonic()
    row, errors = {}, {}
    for src, (reader, fields) in io.SOURCES.items():
        try:
            row[src] = reader(getattr(m, SOURCE_COMPONENT[src]))
            errors[src] = ''
        except Exception as e:  # recorded, not swallowed
            row[src] = io.invalid_sample(fields)
            errors[src] = f'{type(e).__name__}: {e}'
            err_counts[src] = err_counts.get(src, 0) + 1
            if err_counts[src] <= 5:
                print(f'[read error] {src}: {errors[src]}', flush=True)
    return row, errors, time.monotonic() - t0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    m = load_06a()
    rec = io.Recorder()
    events, snaps = [], {}
    meta = dict(git_rev=git_rev(), model_06a=MODEL_06A, tf=TF, dt=DT, t_wo_zero=T_WO_ZERO,
                waypoints=m.waypoints, snapshot_times=SNAPSHOT_TIMES, seed=None,
                seed_note='no RNG in this script; simulator has no recorded seed',
                started_unix=time.time())
    error = None
    err_counts = {}

    def ev(name, t_nom, w0, w1, extra=None):
        e = dict(name=name, t_nominal=t_nom, wall_start=w0, wall_end=w1, blocked_s=w1 - w0)
        if extra is not None:
            e['args'] = extra
        events.append(e)
        print(f'[t={t_nom:5.2f}] {name} blocked {w1 - w0:.3f}s', flush=True)

    try:
        w0 = time.time(); m.setup(); ev('setup', -1.0, w0, time.time())
        w0 = time.time(); m.start(); ev('start', -1.0, w0, time.time())

        # preflight: raw replies once, so the structure is on record
        jdump({
            'pom.frame(robot)': m.pom.frame('robot'),
            'phynt.reference': m.phynt.reference(),
            'phynt.desired': m.phynt.desired(),
            'maneuver.desired': m.maneuver.desired(),
            'phynt.external_wrench': m.phynt.external_wrench(),
            'phynt.wrench_measure': m.phynt.wrench_measure(),
        }, 'preflight.json')
        snaps['after_start'] = snapshot(m, 'after_start', 0.0)

        N = int(TF / DT)
        wo_done = False
        wp_idx = 0
        pending_snaps = dict(SNAPSHOT_TIMES)
        t0 = time.time()
        for i in range(N):
            ts = i * DT
            if not wo_done and ts >= T_WO_ZERO:
                w0 = time.time(); m.phynt.set_wo_zero(duration=2.0); ev('phynt.set_wo_zero', ts, w0, time.time())
                wo_done = True
            if wp_idx < len(m.waypoints) and ts >= m.waypoints[wp_idx][0]:
                args = m.waypoints[wp_idx][1]
                w0 = time.time(); m.maneuver.goto(*args); ev('maneuver.goto', ts, w0, time.time(), list(args))
                wp_idx += 1
            for lbl, t_snap in list(pending_snaps.items()):
                if ts >= t_snap:
                    snaps[lbl] = snapshot(m, lbl, ts)
                    del pending_snaps[lbl]

            t_wall = time.time()
            row, errors, read_s = read_all(m, err_counts)
            rec.add_scalar('i', i)
            rec.add_scalar('t_nominal', ts)
            rec.add_scalar('t_wall', t_wall)
            rec.add_scalar('t_mono', time.monotonic())
            rec.add_scalar('read_s', read_s)
            rec.add_scalar('late_s', (t_wall - t0) - ts)
            for src, sample in row.items():
                rec.add(src, sample)
                rec.add_scalar(f'{src}.error', errors[src])

            if i % 50 == 0:
                p = row['pom'][0]['pos']; f = row['ext_wrench'][0]['force']
                print(f't {ts:5.1f} pos [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] '
                      f'ext_f [{f[0]:.2f} {f[1]:.2f} {f[2]:.2f}] valid {row["ext_wrench"][1]}', flush=True)

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
            meta['tmp_phynt_log_bytes_after_stop'] = file_size(PHYNT_TMP_LOG)
            meta['out_phynt_log_bytes'] = file_size(os.path.join(OUT_DIR, 'phynt.log'))
            jdump(meta, 'meta.json')
            jdump(events, 'events.json')
            jdump(snaps, 'snapshots.json')
            arrays = rec.to_arrays() if rec.cols else {}
            np.savez(os.path.join(OUT_DIR, 't00_diag.npz'), **arrays,
                     meta_json=json.dumps(meta, default=str),
                     events_json=json.dumps(events, default=str),
                     snapshots_json=json.dumps(snaps, default=str))
            print(f'saved {len(arrays.get("i", []))} samples to {OUT_DIR}', flush=True)


if __name__ == '__main__':
    sys.exit(main())
