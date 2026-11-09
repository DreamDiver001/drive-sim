"""Run a drive through a timed scenario and record everything for analysis."""
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .drive import Drive

Action = Callable[[Drive], None]
LoadProfile = Callable[[float], float]


@dataclass
class Trace:
    t: np.ndarray
    setpoint_rpm: np.ndarray
    ramp_rpm: np.ndarray
    speed_rpm: np.ndarray
    current_a: np.ndarray
    current_ref_a: np.ndarray
    voltage_v: np.ndarray
    load_nm: np.ndarray
    state: list
    events: list            # (time, label) of scheduled actions

    def index(self, t: float) -> int:
        return int(min(len(self.t) - 1, max(0, np.searchsorted(self.t, t))))

    def at(self, signal: str, t: float) -> float:
        return float(getattr(self, signal)[self.index(t)])

    def window(self, signal: str, t0: float, t1: float) -> np.ndarray:
        return getattr(self, signal)[self.index(t0):self.index(t1)]

    def first_time(self, mask: np.ndarray, after: float = 0.0) -> float | None:
        """First time at/after `after` where mask is True, else None."""
        idx = np.nonzero(mask & (self.t >= after))[0]
        return float(self.t[idx[0]]) if len(idx) else None

    def state_at(self, t: float):
        return self.state[self.index(t)]


class Simulation:
    def __init__(self, drive: Drive, dt: float = 1e-4):
        self.drive = drive
        self.dt = dt

    def run(self, duration: float, schedule: list[tuple[float, str, Action]] | None = None,
            load: LoadProfile | None = None) -> Trace:
        """schedule: list of (time, label, action). Actions fire once, at the first step >= time."""
        d, dt = self.drive, self.dt
        pending = sorted(schedule or [], key=lambda e: e[0])
        load = load or (lambda t: 0.0)
        n = int(round(duration / dt))
        cols = {k: np.empty(n) for k in
                ("t", "setpoint_rpm", "ramp_rpm", "speed_rpm", "current_a", "current_ref_a", "voltage_v", "load_nm")}
        states, fired = [], []
        for k in range(n):
            t = d.time
            while pending and pending[0][0] <= t + dt / 2:
                when, label, action = pending.pop(0)
                action(d)
                fired.append((t, label))
            torque = load(t)
            d.step(dt, torque)
            cols["t"][k] = d.time
            cols["setpoint_rpm"][k] = d.setpoint_rpm
            cols["ramp_rpm"][k] = d.ramp_output_rpm
            cols["speed_rpm"][k] = d.speed_rpm
            cols["current_a"][k] = d.current
            cols["current_ref_a"][k] = d.current_ref
            cols["voltage_v"][k] = d.voltage
            cols["load_nm"][k] = torque
            states.append(d.state)
        return Trace(**cols, state=states, events=fired)
