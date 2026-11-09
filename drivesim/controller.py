"""PI controller with output limits and anti-windup (conditional integration)."""


class PIController:
    def __init__(self, kp: float, ki: float, out_min: float, out_max: float):
        if out_min >= out_max:
            raise ValueError("out_min must be smaller than out_max")
        self.kp = kp
        self.ki = ki
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.saturated = False

    def reset(self, integral: float = 0.0) -> None:
        self.integral = integral
        self.saturated = False

    def set_limits(self, out_min: float, out_max: float) -> None:
        self.out_min, self.out_max = out_min, out_max

    def step(self, error: float, dt: float) -> float:
        candidate_integral = self.integral + self.ki * error * dt
        unclamped = self.kp * error + candidate_integral
        output = min(self.out_max, max(self.out_min, unclamped))
        self.saturated = output != unclamped
        # Anti-windup: only accept the new integral if it doesn't push further into saturation
        pushing_up = unclamped > self.out_max and error > 0
        pushing_down = unclamped < self.out_min and error < 0
        if not (pushing_up or pushing_down):
            self.integral = candidate_integral
        return output
