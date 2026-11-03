"""
t00_check — verify the t00_diag reads WITHOUT flying.

setup() + start() from 06a (rotors armed, no goto), then one preflight read of
every source and one phynt service snapshot, then stop(). Component logs are
copied by 06a stop() to logs/noc/t00_check/ (not the 06a log dir).

Run inside the container (T2), after simulation.sh is up and Gazebo Play pressed:
  cd /shared-workspace && python3 -m noc.tests.t00_check
Exit code 0 if every source read OK, 1 otherwise.
"""

import json
import os
import sys

from noc.common import telekyb_io as io
from noc.tests.t00_diag import REPO, components, load_06a, jdump, file_size, PHYNT_TMP_LOG

OUT_DIR = os.path.join(REPO, 'logs', 'noc', 't00_check')


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    m = load_06a(OUT_DIR)
    comps = components(m)
    sources = io.build_sources(m.phynt)
    result = {'sources': io.describe_sources(sources)}
    try:
        m.setup()
        m.start()
        result['preflight'] = io.preflight(comps, sources)
        snap = io.phynt_snapshot(m.phynt)
        snap['tmp_phynt_log_bytes'] = file_size(PHYNT_TMP_LOG)
        result['snapshot'] = snap
        print(f'[snapshot] {json.dumps(snap, default=str)}', flush=True)
    finally:
        try:
            m.stop()
        finally:
            jdump(result, 'check.json', OUT_DIR)
    ok = all(v['ok'] for v in result['preflight'].values())
    snap_err = [k for k, v in result['snapshot'].items() if isinstance(v, dict) and 'error' in v]
    print(f'[check] sources {"ALL OK" if ok else "FAILED"}; snapshot service errors: {snap_err or "none"}',
          flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
