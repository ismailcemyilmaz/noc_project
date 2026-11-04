"""In-process stand-in for t04's FtLogger (log_ft.sh): a thread writes ft.log lines in the
log_ft.sh format from a FakeWorld (contact force when the fake drone is at the wall)."""

import json
import threading
import time

import numpy as np

from noc.common import ft_log


def wrench_line(host, stamp, f, t=(0.0, 0.0, 0.0)):
    sec = int(stamp)
    msg = {'header': {'stamp': {'sec': str(sec), 'nsec': int(round((stamp - sec) * 1e9))}},
           'force': {k: v for k, v in zip('xyz', f) if v != 0.0},
           'torque': {k: v for k, v in zip('xyz', t) if v != 0.0}}
    return f'{host:.6f}\t{json.dumps(msg, separators=(",", ":"))}\n'


class FakeFtLogger:
    def __init__(self, path, world=None, rate=1000.0, silent=False, die_after=None,
                 contact_x=1.5, f_contact=-0.6, weight=-0.098, noise=0.002, seed=0):
        self.path, self.world, self.rate, self.silent, self.die_after = path, world, rate, silent, die_after
        self.contact_x, self.f_contact, self.weight, self.noise = contact_x, f_contact, weight, noise
        self.rng = np.random.default_rng(seed)
        self.tail = ft_log.FtTail(path)
        self.info = dict(cmd=['fake'], path=path)
        self._stop = threading.Event()
        self._th = None

    def _run(self):
        t0 = time.time()
        with open(self.path, 'w') as f:
            f.write('# fake log_ft.sh\n')
            f.flush()
            while not self._stop.is_set():
                now = time.time()
                if self.die_after is not None and now - t0 > self.die_after:
                    return
                if not self.silent:
                    fx = self.f_contact if (self.world is not None and self.world.pos[0] >= self.contact_x) else 0.0
                    fx += self.rng.normal(0, self.noise)
                    f.write(wrench_line(now, now - t0 + 100.0, (fx, 0.0, self.weight)))
                    f.flush()
                time.sleep(1.0 / self.rate)

    def start(self):
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        self.info.update(pid=None, started=time.time())

    def wait_stream(self, n_min, timeout):
        from noc.tests.t04_ft_check import FtLogger
        self.proc = None
        return FtLogger.wait_stream(self, n_min, timeout)

    def stop(self):
        self.info['alive_at_stop'] = bool(self._th and self._th.is_alive())
        self._stop.set()
        if self._th:
            self._th.join(2)
        self.info.update(returncode=-15, stopped=time.time(), stderr='')
        self.tail.poll()
        self.info.update(n_messages=self.tail.n, n_bad=self.tail.n_bad, bad_examples=self.tail.bad_examples)
