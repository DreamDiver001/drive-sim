"""Separately excited / permanent-magnet DC motor model.

Electrical:  L di/dt = V - R*i - Ke*w
Mechanical:  J dw/dt = Kt*i - B*w - T_load

Integrated with forward Euler; the time step must be well below the
electrical time constant L/R (3 ms with the default parameters).
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class MotorParams:
    R: float = 0.5        # armature resistance [ohm]
    L: float = 1.5e-3     # armature inductance [H]
    Kt: float = 0.1       # torque constant [N*m/A]
    Ke: float = 0.1       # back-EMF constant [V*s/rad]
    J: float = 1.0e-3     # rotor + load inertia [kg*m^2]
    B: float = 1.0e-4     # viscous friction [N*m*s/rad]

    @property
    def electrical_time_constant(self) -> float:
        return self.L / self.R


class DCMotor:
    def __init__(self, params: MotorParams | None = None):
        self.p = params or MotorParams()
        self.current = 0.0   # armature current [A]
        self.omega = 0.0     # shaft speed [rad/s]
        self.locked = False  # shaft mechanically blocked

    @property
    def torque(self) -> float:
        """Electromagnetic torque [N*m]."""
        return self.p.Kt * self.current

    @property
    def back_emf(self) -> float:
        return self.p.Ke * self.omega

    def step(self, voltage: float, load_torque: float, dt: float) -> None:
        p = self.p
        di = (voltage - p.R * self.current - p.Ke * self.omega) / p.L
        dw = (p.Kt * self.current - p.B * self.omega - load_torque) / p.J
        self.current += di * dt
        if self.locked:
            self.omega = 0.0
        else:
            self.omega += dw * dt

    def steady_state_speed(self, voltage: float, load_torque: float = 0.0) -> float:
        """Analytical steady-state speed for a constant voltage [rad/s]."""
        p = self.p
        return (p.Kt * voltage - p.R * load_torque) / (p.R * p.B + p.Kt * p.Ke)
