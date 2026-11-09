"""Component tests: the motor model is checked against textbook DC-motor equations."""
import math

import pytest

from drivesim import DCMotor, MotorParams

DT = 1e-5


def run(motor, voltage, seconds, load=0.0):
    for _ in range(int(seconds / DT)):
        motor.step(voltage, load, DT)


def test_no_load_speed_matches_analytical_steady_state():
    motor = DCMotor()
    run(motor, 24.0, 15.0)          # mechanical time constant is ~0.1 s; friction tail is slower
    assert motor.omega == pytest.approx(motor.steady_state_speed(24.0), rel=1e-3)


def test_loaded_speed_matches_analytical_steady_state():
    motor = DCMotor()
    run(motor, 24.0, 15.0, load=0.5)
    assert motor.omega == pytest.approx(motor.steady_state_speed(24.0, 0.5), rel=1e-3)


def test_locked_rotor_current_rises_with_electrical_time_constant():
    motor = DCMotor()
    motor.locked = True
    tau = motor.p.electrical_time_constant           # 3 ms
    run(motor, 10.0, tau)
    final = 10.0 / motor.p.R                          # 20 A
    assert motor.current == pytest.approx(final * (1 - math.exp(-1)), rel=0.01)
    assert motor.omega == 0.0


def test_torque_is_proportional_to_current():
    motor = DCMotor(MotorParams(Kt=0.2))
    motor.current = 7.5
    assert motor.torque == pytest.approx(1.5)


def test_load_larger_than_motor_torque_turns_the_shaft_backwards():
    motor = DCMotor()
    run(motor, 0.0, 0.1, load=0.5)
    assert motor.omega < 0
