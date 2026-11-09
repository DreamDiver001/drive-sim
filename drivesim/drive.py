"""Variable-speed drive: cascaded speed/current control, stop modes and protection.

Structure (like a real servo / DC drive):

    speed setpoint -> ramp-function generator -> speed PI -> current limit
                   -> current PI -> armature voltage (limited by DC link) -> motor

Stop modes follow the usual industrial-drive naming:
    OFF1  normal stop: ramp down with the ramp-down time, then pulses off
    OFF2  coast stop: pulses off immediately, motor coasts freely
    OFF3  fast stop:  ramp down with the (shorter) OFF3 ramp, then pulses off
"""
from dataclasses import dataclass, field
from enum import Enum

from .controller import PIController
from .motor import DCMotor, MotorParams
from .ramp import RampFunctionGenerator
from .units import rad_s_to_rpm, rpm_to_rad_s


class DriveState(Enum):
    READY = "ready"              # pulses off, waiting for ON
    RUNNING = "running"          # closed-loop speed control
    STOPPING_OFF1 = "off1"       # ramping down (normal stop)
    STOPPING_OFF3 = "off3"       # ramping down (fast stop)
    FAULT = "fault"              # tripped, pulses off, fault latched


class FaultCode(Enum):
    OVERCURRENT = "F01_OVERCURRENT"
    OVERSPEED = "F02_OVERSPEED"


@dataclass
class DriveConfig:
    dc_link_voltage: float = 48.0      # [V] maximum armature voltage
    max_speed_rpm: float = 3000.0      # setpoint limit and ramp reference speed
    ramp_up_s: float = 2.0             # 0 -> max speed
    ramp_down_s: float = 2.0           # max speed -> 0 (OFF1)
    off3_ramp_down_s: float = 0.5      # max speed -> 0 (OFF3 fast stop)
    current_limit_a: float = 15.0      # controlled current limit
    overcurrent_trip_a: float = 20.0   # hardware-style trip threshold
    overspeed_trip_rpm: float = 3300.0 # 110 % of max speed
    standstill_rpm: float = 10.0       # below this a stop is considered complete
    # Current loop tuned for ~500 Hz bandwidth (pole-zero cancellation of L/R)
    current_kp: float = 4.7
    current_ki: float = 1570.0
    # Speed loop: tuned so REQ-05 (load step) holds with margin
    speed_kp: float = 1.2              # [A per rad/s]
    speed_ki: float = 30.0
    # Acceleration precontrol: feed forward the current needed to follow the ramp
    # (J * dw/dt / Kt), so the speed PI only corrects errors. Added for FINDING-01.
    accel_precontrol: bool = True
    motor: MotorParams = field(default_factory=MotorParams)


class Drive:
    def __init__(self, config: DriveConfig | None = None):
        self.cfg = c = config or DriveConfig()
        self.motor = DCMotor(c.motor)
        self.ramp = RampFunctionGenerator(c.max_speed_rpm, c.ramp_up_s, c.ramp_down_s)
        self.speed_pi = PIController(c.speed_kp, c.speed_ki, -c.current_limit_a, c.current_limit_a)
        self.current_pi = PIController(c.current_kp, c.current_ki, -c.dc_link_voltage, c.dc_link_voltage)
        self.state = DriveState.READY
        self.fault: FaultCode | None = None
        self.setpoint_rpm = 0.0
        self.voltage = 0.0
        self.current_ref = 0.0
        self.time = 0.0
        self._previous_target_rpm = 0.0
        self.fault_time: float | None = None
        self._injected_faults: set[str] = set()

    # ------------------------------------------------------------------ status
    @property
    def speed_rpm(self) -> float:
        return rad_s_to_rpm(self.motor.omega)

    @property
    def current(self) -> float:
        return self.motor.current

    @property
    def pulses_enabled(self) -> bool:
        return self.state in (DriveState.RUNNING, DriveState.STOPPING_OFF1, DriveState.STOPPING_OFF3)

    @property
    def ramp_output_rpm(self) -> float:
        return self.ramp.output

    # ---------------------------------------------------------------- commands
    def set_speed(self, rpm: float) -> None:
        """Speed setpoint, clamped to +/- max speed."""
        limit = self.cfg.max_speed_rpm
        self.setpoint_rpm = max(-limit, min(limit, rpm))

    def on(self) -> bool:
        if self.state != DriveState.READY:
            return False
        # Start the ramp from the actual speed so a spinning motor is caught smoothly
        self.ramp.reset(self.speed_rpm)
        self._previous_target_rpm = self.speed_rpm
        self.speed_pi.reset()
        self.current_pi.reset()
        self.state = DriveState.RUNNING
        return True

    def off1(self) -> None:
        # OFF3 (fast stop) has priority, so OFF1 only acts on a running drive
        if self.state == DriveState.RUNNING:
            self.state = DriveState.STOPPING_OFF1

    def off3(self) -> None:
        if self.state in (DriveState.RUNNING, DriveState.STOPPING_OFF1):
            self.state = DriveState.STOPPING_OFF3

    def off2(self) -> None:
        if self.pulses_enabled:
            self.state = DriveState.READY

    def acknowledge(self) -> bool:
        """Clear a latched fault. Rejected while the cause is still present."""
        if self.state != DriveState.FAULT:
            return False
        if self.fault == FaultCode.OVERSPEED and abs(self.speed_rpm) >= self.cfg.overspeed_trip_rpm:
            return False
        if self.fault == FaultCode.OVERCURRENT and abs(self.current) >= self.cfg.overcurrent_trip_a:
            return False
        self.fault = None
        self.fault_time = None
        self.state = DriveState.READY
        return True

    # --------------------------------------------------------- fault injection
    def inject_fault(self, name: str) -> None:
        """Simulate a hardware/mechanical defect.

        'output_stage_short'  output transistor stuck on: armature voltage = +DC link
        'locked_rotor'        shaft mechanically blocked (e.g. jammed conveyor)
        """
        if name == "locked_rotor":
            self.motor.locked = True
        elif name == "output_stage_short":
            self._injected_faults.add(name)
        else:
            raise ValueError(f"unknown fault: {name}")

    def clear_injected_faults(self) -> None:
        self._injected_faults.clear()
        self.motor.locked = False

    # -------------------------------------------------------------- main cycle
    def step(self, dt: float, load_torque: float = 0.0) -> None:
        c = self.cfg
        if self.pulses_enabled:
            self._closed_loop(dt)
        else:
            self._pulses_off_voltage()
            self.ramp.reset(self.speed_rpm)
            self._previous_target_rpm = self.speed_rpm

        if "output_stage_short" in self._injected_faults and self.pulses_enabled:
            self.voltage = c.dc_link_voltage

        previous_current = self.motor.current
        self.motor.step(self.voltage, load_torque, dt)
        if not self.pulses_enabled and previous_current * self.motor.current <= 0:
            self.motor.current = 0.0     # diodes block reverse current; open circuit stays at 0 A
        self.time += dt
        self._protection()

    def _closed_loop(self, dt: float) -> None:
        c = self.cfg
        if self.state == DriveState.RUNNING:
            target = self.ramp.step(self.setpoint_rpm, dt)
        elif self.state == DriveState.STOPPING_OFF1:
            target = self.ramp.step(0.0, dt)
        else:
            target = self.ramp.step(0.0, dt, ramp_down_override_s=c.off3_ramp_down_s)

        if self.state != DriveState.RUNNING and target == 0.0 and abs(self.speed_rpm) < c.standstill_rpm:
            self.state = DriveState.READY
            self._pulses_off_voltage()
            return

        speed_error = rpm_to_rad_s(target) - self.motor.omega
        feedforward = 0.0
        if c.accel_precontrol:
            accel = rpm_to_rad_s(target - self._previous_target_rpm) / dt
            feedforward = c.motor.J * accel / c.motor.Kt
            feedforward = max(-c.current_limit_a, min(c.current_limit_a, feedforward))
        self._previous_target_rpm = target
        # The PI gets whatever current headroom the feed-forward leaves
        self.speed_pi.set_limits(-c.current_limit_a - feedforward, c.current_limit_a - feedforward)
        self.current_ref = feedforward + self.speed_pi.step(speed_error, dt)
        self.voltage = self.current_pi.step(self.current_ref - self.motor.current, dt)

    def _pulses_off_voltage(self) -> None:
        """With pulses off, any remaining current decays through the diodes against the DC link."""
        self.current_ref = 0.0
        self.speed_pi.reset()
        self.current_pi.reset()
        i = self.motor.current
        self.voltage = -self.cfg.dc_link_voltage if i > 0 else (self.cfg.dc_link_voltage if i < 0 else 0.0)

    def _protection(self) -> None:
        if self.state == DriveState.FAULT:
            return
        c = self.cfg
        if abs(self.motor.current) > c.overcurrent_trip_a:
            self._trip(FaultCode.OVERCURRENT)
        elif abs(self.speed_rpm) > c.overspeed_trip_rpm:
            self._trip(FaultCode.OVERSPEED)

    def _trip(self, code: FaultCode) -> None:
        self.state = DriveState.FAULT
        self.fault = code
        self.fault_time = self.time
