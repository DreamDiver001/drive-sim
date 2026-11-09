"""Component tests for the PI controller."""
import pytest

from drivesim import PIController


def test_proportional_part_only_on_first_step_with_zero_ki():
    pi = PIController(kp=2.0, ki=0.0, out_min=-10, out_max=10)
    assert pi.step(3.0, 0.001) == pytest.approx(6.0)


def test_integral_accumulates_error_over_time():
    pi = PIController(kp=0.0, ki=100.0, out_min=-10, out_max=10)
    for _ in range(10):
        out = pi.step(1.0, 0.001)
    assert out == pytest.approx(1.0)


@pytest.mark.parametrize("error, expected", [(100.0, 10.0), (-100.0, -10.0)])
def test_output_is_clamped_to_limits(error, expected):
    pi = PIController(kp=1.0, ki=0.0, out_min=-10, out_max=10)
    assert pi.step(error, 0.001) == expected
    assert pi.saturated


def test_anti_windup_lets_output_leave_saturation_immediately():
    pi = PIController(kp=1.0, ki=1000.0, out_min=-10, out_max=10)
    for _ in range(10_000):                       # 10 s stuck in saturation
        pi.step(50.0, 0.001)
    assert abs(pi.integral) <= 10.0               # integral did not wind up
    assert pi.step(-5.0, 0.001) < 10.0            # reacts at once when error reverses


def test_reset_clears_integral():
    pi = PIController(kp=1.0, ki=10.0, out_min=-10, out_max=10)
    pi.step(1.0, 0.1)
    pi.reset()
    assert pi.integral == 0.0


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        PIController(1.0, 1.0, out_min=5, out_max=5)
