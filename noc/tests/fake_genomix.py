"""
Fake genomix component handles for dry runs of t00/t01 scripts on the host.

Reply structures copy the real ones (logs/t00_probe.txt, logs/noc/t00/preflight.json,
noc/docs/genom/*/README.adoc). Input ports raise the TypeError genomix raises
without a 'path' argument, so any accidental input-port read fails loudly.
"""

import copy
import types

import numpy as np

INPUT_PORT_ERR = "Handle.__partial_pub.<locals>.partial() missing 1 required positional argument: 'path'"


def _ts(t):
    return {'sec': int(t), 'nsec': int((t % 1) * 1e9)}


def _input_port(*a, **k):
    raise TypeError(INPUT_PORT_ERR)


class FakeWorld:
    """Minimal state: position follows the last goto target; external force set by test."""

    def __init__(self, t0=1.79e9):
        self.t = t0
        self.pos = np.array([0.0, 0.0, 0.21])
        self.f_ext = np.zeros(3)
        self.wo_running = True
        self.rotor_vel = [71.0] * 6
        self.thrust = np.array([0.0, 0.0, 26.7])
        self.gotos = []
        self.af = {'mass': 5.0, 'B': [2 * (5.0 * k) ** 0.5 for k in (10, 100, 100, 50, 50, 50)],
                   'K': [10.0, 100.0, 100.0, 50.0, 50.0, 50.0],
                   'J': [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05],
                   'force': {'x': 0, 'y': 0, 'z': 0}, 'torque': {'x': 0, 'y': 0, 'z': 0}}
        self.af_calls = []
        self.af_ignore_k = False   # test hook: set_af_parameters silently keeps the old K
        self.af_raise = None       # test hook: exception raised by set_af_parameters
        # t03 hooks (defaults keep the t01/t02 behaviour):
        self.des_valid_after_goto = False   # phynt.desired pos/att valid once a goto was sent
        self.ext_empty_until_goto = False   # phynt.external_wrench raises EMPTY_POSTER before the first goto

    def tick(self):
        self.t += 0.001
        return self.t


def make_module(world=None, rotor_measure_ok=True):
    w = world or FakeWorld()

    def frame(port):
        assert port == 'robot'
        return {'frame': {'ts': _ts(w.tick()), 'intrinsic': 0,
                          'pos': dict(zip('xyz', w.pos.tolist())),
                          'att': {'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
                          'vel': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
                          'avel': {'wx': 0.0, 'wy': 0.0, 'wz': 0.0},
                          'acc': {'ax': 0.0, 'ay': 0.0, 'az': 0.0}, 'aacc': None}}

    def rb_state(key, pos_none=True):
        return {key: {'ts': _ts(w.t), 'intrinsic': 0,
                      'pos': None if pos_none else dict(zip('xyz', w.pos.tolist())), 'att': None,
                      'vel': {'vx': 0, 'vy': 0, 'vz': 0}, 'avel': {'wx': 0, 'wy': 0, 'wz': 0},
                      'acc': {'ax': 0, 'ay': 0, 'az': 0}, 'aacc': None, 'jerk': None, 'snap': None}}

    def phynt_desired():
        if w.des_valid_after_goto and w.gotos:
            r = rb_state('desired', pos_none=False)
            r['desired']['att'] = {'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0}
            return r
        return rb_state('desired')

    def external_wrench():
        if w.ext_empty_until_goto and not w.gotos:
            raise RuntimeError("::genom::mwerr: {'what': 'phynt/external_wrench port: S_posterLib_EMPTY_POSTER'}")
        return wrench('external_wrench', w.wo_running)

    def wrench(key, valid):
        f = dict(zip('xyz', w.f_ext.tolist())) if valid else None
        return {key: {'ts': _ts(w.t) if valid else {'sec': 0, 'nsec': 0}, 'intrinsic': 0,
                      'force': f, 'force_cov': None,
                      'torque': {'x': 0, 'y': 0, 'z': 0} if valid else None, 'torque_cov': None}}

    def goto(x, y, z, yaw, dur, ack=False):
        w.gotos.append((x, y, z, yaw, dur, ack))
        w.pos = np.array([x, y, max(z, 0.21)], float)
        return 'ack-request' if ack else {}

    def set_af(mass, B, K, J):
        w.af_calls.append(dict(mass=mass, B=list(B), K=list(K), J=list(J)))
        if w.af_raise is not None:
            raise w.af_raise
        w.af.update(mass=float(mass), B=[float(x) for x in B], J=[float(x) for x in J])
        if not w.af_ignore_k:
            w.af['K'] = [float(x) for x in K]
        return {}

    phynt = types.SimpleNamespace(
        desired=phynt_desired, reference=_input_port, wrench_measure=_input_port,
        state=_input_port,
        get_reference=lambda: rb_state('reference'),
        external_wrench=external_wrench,
        get_af_parameters=lambda: {'af': copy.deepcopy(w.af)},
        set_af_parameters=set_af,
        get_wo_parameters=lambda: {'wo': {'bias': [0] * 6}},
        get_body_parameters=lambda: {'body': {'mass': 2.72}},
        log_info=lambda: {'miss': 0, 'total': 0},
        set_wo_zero=lambda duration=2.0: None)
    rc_attrs = dict(
        connect_port=lambda d: {})
    if rotor_measure_ok:
        rc_attrs['rotor_measure'] = lambda: {'rotor_measure': {'rotor': [
            {'ts': _ts(w.t), 'emerg': False, 'spinning': True, 'starting': False, 'disabled': False,
             'velocity': v, 'throttle': 0, 'consumption': 0, 'energy_level': 0} for v in w.rotor_vel]}}
    connections = []
    m = types.SimpleNamespace(
        world=w, connections=connections,
        pom=types.SimpleNamespace(frame=frame),
        phynt=phynt,
        maneuver=types.SimpleNamespace(desired=lambda: rb_state('desired'), goto=goto),
        uavatt=types.SimpleNamespace(
            wrench_measure=lambda: wrench('wrench_measure', True),
            connect_port=lambda d: connections.append(('uavatt', copy.deepcopy(d))) or {}),
        uavpos=types.SimpleNamespace(uav_input=lambda: {'uav_input': {
            'ts': _ts(w.t), 'intrinsic': 0, 'thrust': dict(zip('xyz', w.thrust.tolist())),
            'att': None, 'avel': None, 'aacc': None}}),
        rotorcraft=types.SimpleNamespace(**rc_attrs),
        waypoints=[(0.0, (0, 0, 1.0, 0, 5)), (0.1, (1.55, 0, 1.0, 0, 2))],
        BODY_X_CONTACT=1.55,
        setup=lambda: None, start=lambda: None, stop=lambda: None, LOG_DIR=None)
    return m
