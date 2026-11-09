"""Ramp-function generator: limits how fast the speed setpoint may change.

Ramp times are defined like on industrial drives: the time to go from
0 to the reference (maximum) speed. A ramp time of 0 passes the setpoint
straight through.
"""


class RampFunctionGenerator:
    def __init__(self, reference_speed_rpm: float, ramp_up_s: float, ramp_down_s: float):
        if reference_speed_rpm <= 0:
            raise ValueError("reference speed must be positive")
        if ramp_up_s < 0 or ramp_down_s < 0:
            raise ValueError("ramp times cannot be negative")
        self.reference_speed = reference_speed_rpm
        self.ramp_up_s = ramp_up_s
        self.ramp_down_s = ramp_down_s
        self.output = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.output = value

    def _rate(self, ramp_time: float) -> float:
        return float("inf") if ramp_time == 0 else self.reference_speed / ramp_time

    def step(self, target: float, dt: float, ramp_down_override_s: float | None = None) -> float:
        diff = target - self.output
        if diff == 0:
            return self.output
        # "Up" means moving away from zero, "down" means moving towards zero
        accelerating = abs(target) > abs(self.output) and target * self.output >= 0
        ramp_time = self.ramp_up_s if accelerating else (
            ramp_down_override_s if ramp_down_override_s is not None else self.ramp_down_s)
        max_step = self._rate(ramp_time) * dt
        if abs(diff) <= max_step:
            self.output = target
        else:
            self.output += max_step if diff > 0 else -max_step
        return self.output
