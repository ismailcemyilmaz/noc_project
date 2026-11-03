"""
Parsers for telekyb3 / GenoM3 component logs (whitespace-separated, '#' comments,
first non-comment line = header).

Missing values written by components ('-', 'nan') become NaN. A file with only a
header is returned with zero-length arrays (``n == 0``) so callers can report it
as empty instead of crashing.
"""

import os
import numpy as np


class Log(dict):
    """dict column -> np.ndarray, plus metadata."""

    def __init__(self, path, header, data, n_bad):
        super().__init__({c: data[:, i] for i, c in enumerate(header)})
        self.path = path
        self.header = header
        self.n = data.shape[0]
        self.n_bad = n_bad  # rows dropped because column count mismatched


def _to_float(tok):
    try:
        return float(tok)
    except ValueError:
        return np.nan  # '-' and other placeholders


def read_log(path, text_cols=()):
    """Parse one component log. ``text_cols``: header names whose values are
    non-numeric labels (e.g. optitrack 'name'); they are dropped."""
    header = None
    rows = []
    n_bad = 0
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            parts = s.split()
            if header is None:
                header = parts
                continue
            if len(parts) != len(header):
                n_bad += 1
                continue
            rows.append([_to_float(p) for p, h in zip(parts, header) if h not in text_cols])
    if header is None:
        raise ValueError(f'{path}: no header line')
    keep = [h for h in header if h not in text_cols]
    data = np.array(rows, dtype=float).reshape(-1, len(keep))
    return Log(path, keep, data, n_bad)


def read_dir(log_dir, names=('pom', 'pom-measurements', 'uavpos', 'uavatt',
                             'maneuver', 'phynt', 'opti', 'rotorcraft')):
    """Read all known logs in ``log_dir``. Missing files map to None."""
    out = {}
    for n in names:
        p = os.path.join(log_dir, n + '.log')
        if not os.path.exists(p):
            out[n] = None
            continue
        out[n] = read_log(p, text_cols=('name',) if n == 'opti' else ())
    return out
