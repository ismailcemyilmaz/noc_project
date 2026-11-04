"""
stdin: `gz topic -e --json-output` stream; stdout: one line per message,
'<time.time()> TAB <compact JSON>', flushed per message.

A message may span several lines (pretty-printed JSON): lines are accumulated until
the braces balance. Text outside a JSON object (warnings) goes to stderr.
"""

import json
import sys
import time


def main(inp=sys.stdin, out=sys.stdout, err=sys.stderr, clock=time.time):
    buf, depth = [], 0
    for line in inp:
        s = line.strip()
        if not s:
            continue
        if depth == 0 and not s.startswith('{'):
            err.write(f'ft_stamp: ignored non-JSON line: {s[:200]}\n')
            continue
        t = clock() if depth == 0 else t
        buf.append(s)
        depth += s.count('{') - s.count('}')
        if depth <= 0:
            txt = ''.join(buf)
            buf, depth = [], 0
            try:
                txt = json.dumps(json.loads(txt), separators=(',', ':'))
            except ValueError:
                err.write(f'ft_stamp: unparsable message kept raw: {txt[:200]}\n')
            out.write(f'{t:.6f}\t{txt}\n')
            out.flush()


if __name__ == '__main__':
    main()
