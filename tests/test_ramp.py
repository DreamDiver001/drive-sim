"""Component tests for the ramp-function generator."""
import pytest

from drivesim import RampFunctionGenerator

DT = 0.001


def ramp_until(rfg, target, seconds):
    for _ in range(int(round(seconds / DT))):
        rfg.step(target, DT)
    return rfg.output


@pytest.mark.req("REQ-02")
def test_ramp_up_rate_is_reference_speed_over_ramp_time():
    rfg = RampFunctionGenerator(3000, ramp_up_s=2.0, ramp_down_s=1.0)
    assert ramp_until(rfg, 3000, 1.0) == pytest.approx(1500)
    assert ramp_until(rfg, 3000, 1.0) == pytest.approx(3000)


def test_partial_setpoint_takes_proportional_time():
    rfg = RampFunctionGenerator(3000, ramp_up_s=2.0, ramp_down_s=2.0)
    assert ramp_until(rfg, 600, 0.4) == pytest.approx(600)


def test_ramp_down_uses_ramp_down_time():
    rfg = RampFunctionGenerator(3000, ramp_up_s=2.0, ramp_down_s=1.0)
    rfg.reset(3000)
    assert ramp_until(rfg, 0, 0.5) == pytest.approx(1500)


def test_zero_ramp_time_passes_setpoint_through():
    rfg = RampFunctionGenerator(3000, ramp_up_s=0.0, ramp_down_s=0.0)
    assert rfg.step(2500, DT) == 2500


def test_reversal_ramps_down_through_zero_then_up():
    rfg = RampFunctionGenerator(3000, ramp_up_s=2.0, ramp_down_s=1.0)
    rfg.reset(1500)
    assert ramp_until(rfg, -1500, 0.5) == pytest.approx(0, abs=1e-6)       # down-ramp: 3000 rpm/s
    assert ramp_until(rfg, -1500, 1.0) == pytest.approx(-1500)              # up-ramp: 1500 rpm/s


def test_ramp_down_override_is_used_for_fast_stop():
    rfg = RampFunctionGenerator(3000, ramp_up_s=2.0, ramp_down_s=2.0)
    rfg.reset(3000)
    for _ in range(250):
        rfg.step(0, DT, ramp_down_override_s=0.5)
    assert rfg.output == pytest.approx(1500)


@pytest.mark.parametrize("args", [(0, 1, 1), (3000, -1, 1), (3000, 1, -0.5)])
def test_invalid_parameters_are_rejected(args):
    with pytest.raises(ValueError):
        RampFunctionGenerator(*args)
