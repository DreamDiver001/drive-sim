"""System tests: every test verifies a requirement from requirements.toml.

Each test is written like a test case on a drive test bench:
precondition (configuration) -> stimulus (scheduled commands / load) -> check against the requirement limits.
"""
import numpy as np
import pytest

from drivesim import DriveState, FaultCode
from helpers import start, stop_time

MAX_RPM = 3000.0
RATED_LOAD = 1.0          # N*m (= 10 A with Kt = 0.1)


# ----------------------------------------------------------------- functional
@pytest.mark.req("REQ-01")
@pytest.mark.parametrize("requested, expected", [(5000, 3000), (-5000, -3000), (3000, 3000), (1234.5, 1234.5)])
def test_setpoint_is_limited_to_max_speed(make_drive, requested, expected):
    drive, _ = make_drive()
    drive.set_speed(requested)
    assert drive.setpoint_rpm == expected


@pytest.mark.req("REQ-01")
def test_motor_never_exceeds_max_speed_with_too_high_setpoint(make_drive):
    drive, sim = make_drive()
    trace = sim.run(3.5, [(0.0, "ON 5000 rpm", start(5000))])
    assert trace.speed_rpm.max() <= MAX_RPM * 1.01      # within overshoot allowance of REQ-03


@pytest.mark.req("REQ-02")
@pytest.mark.parametrize("setpoint, ramp_up_s", [(3000, 2.0), (1500, 2.0), (3000, 1.0), (750, 4.0)])
def test_acceleration_follows_ramp_time(make_drive, setpoint, ramp_up_s):
    drive, sim = make_drive(ramp_up_s=ramp_up_s)
    expected = setpoint / MAX_RPM * ramp_up_s
    trace = sim.run(expected + 1.0, [(0.0, "ON", start(setpoint))])

    reached = trace.first_time(trace.speed_rpm >= 0.99 * setpoint)
    assert reached is not None, "setpoint never reached"
    assert 0.95 * expected <= reached <= expected + 0.2


@pytest.mark.req("REQ-03")
@pytest.mark.parametrize("setpoint", [500, 1500, 3000, -2000])
def test_overshoot_is_below_one_percent(make_drive, setpoint):
    drive, sim = make_drive()
    trace = sim.run(abs(setpoint) / MAX_RPM * 2.0 + 1.5, [(0.0, "ON", start(setpoint))])
    peak = np.max(np.sign(setpoint) * trace.speed_rpm)
    assert peak <= abs(setpoint) * 1.01


@pytest.mark.req("REQ-04")
@pytest.mark.parametrize("setpoint", [300, 1500, 3000, -1500])
@pytest.mark.parametrize("load", [0.0, RATED_LOAD], ids=["no-load", "rated-load"])
def test_steady_state_speed_error(make_drive, setpoint, load):
    drive, sim = make_drive()
    load_sign = np.sign(setpoint)                  # load always opposes the rotation
    trace = sim.run(4.0, [(0.0, "ON", start(setpoint))], load=lambda t: load * load_sign)
    settled = trace.window("speed_rpm", 3.5, 4.0)
    assert np.max(np.abs(settled - setpoint)) <= 0.001 * MAX_RPM


@pytest.mark.req("REQ-05")
def test_rated_load_step_dip_and_recovery(make_drive):
    drive, sim = make_drive()
    t_step = 2.0
    trace = sim.run(3.0, [(0.0, "ON 1500 rpm", start(1500))],
                    load=lambda t: RATED_LOAD if t >= t_step else 0.0)

    after = trace.window("speed_rpm", t_step, 3.0)
    assert 1500 - after.min() <= 100, "speed dip too large"

    outside = np.nonzero((np.abs(trace.speed_rpm - 1500) > 10) & (trace.t >= t_step))[0]
    recovery = trace.t[outside[-1]] - t_step if len(outside) else 0.0
    assert recovery <= 0.2, f"recovered after {recovery:.3f} s"


@pytest.mark.req("REQ-07")
def test_reversal_completes_in_time_without_fault(make_drive):
    drive, sim = make_drive()
    t_rev = 1.5
    trace = sim.run(5.0, [(0.0, "ON +1500", start(1500)),
                          (t_rev, "setpoint -1500", lambda d: d.set_speed(-1500))])
    reached = trace.first_time(trace.speed_rpm <= -0.99 * 1500, after=t_rev)
    assert reached is not None and reached - t_rev <= 2.0 + 0.3
    assert DriveState.FAULT not in trace.state


# ------------------------------------------------------------------ stop modes
@pytest.mark.req("REQ-08")
@pytest.mark.parametrize("speed", [3000, 1500])
def test_off1_ramps_down_then_switches_pulses_off(make_drive, speed):
    drive, sim = make_drive()
    t_off = 2.5
    trace = sim.run(t_off + 3.0, [(0.0, "ON", start(speed)), (t_off, "OFF1", lambda d: d.off1())])

    stopped = stop_time(trace, after=t_off)
    assert stopped is not None, "drive never reached READY"
    assert stopped - t_off <= speed / MAX_RPM * 2.0 + 0.3
    # Speed must follow the ramp, not drop faster (no coasting / no hard braking)
    halfway = t_off + speed / MAX_RPM * 2.0 / 2
    assert trace.at("speed_rpm", halfway) == pytest.approx(speed / 2, abs=0.03 * speed)
    assert trace.at("voltage_v", stopped + 0.01) == 0.0


@pytest.mark.req("REQ-09")
def test_off3_stops_faster_than_off1(make_drive):
    results = {}
    for mode in ("off1", "off3"):
        drive, sim = make_drive()
        trace = sim.run(5.5, [(0.0, "ON", start(3000)), (2.5, mode, lambda d, m=mode: getattr(d, m)())])
        results[mode] = stop_time(trace, after=2.5) - 2.5
    assert results["off3"] <= 0.5 + 0.3
    assert results["off3"] < results["off1"]


@pytest.mark.req("REQ-09")
def test_off3_overrides_active_off1(make_drive):
    drive, sim = make_drive()
    trace = sim.run(4.0, [(0.0, "ON", start(3000)), (2.5, "OFF1", lambda d: d.off1()),
                          (2.6, "OFF3", lambda d: d.off3()), (2.65, "OFF1 again", lambda d: d.off1())])
    assert trace.state_at(2.7) == DriveState.STOPPING_OFF3
    remaining = trace.at("speed_rpm", 2.6) / MAX_RPM * 0.5
    assert stop_time(trace, after=2.6) - 2.6 <= remaining + 0.3


@pytest.mark.req("REQ-10")
def test_off2_removes_voltage_immediately_and_motor_coasts(make_drive):
    drive, sim = make_drive()
    t_off = 2.0
    # Load while running, so there is current to switch off. (A constant load torque
    # during coasting would turn the shaft backwards, so it is removed at OFF2.)
    trace = sim.run(3.0, [(0.0, "ON", start(1500)), (t_off, "OFF2", lambda d: d.off2())],
                    load=lambda t: 0.5 if t < t_off else 0.0)
    assert trace.at("current_a", t_off - 0.001) > 4.0

    first_cycle = trace.index(t_off)
    assert trace.state[first_cycle] == DriveState.READY
    assert trace.current_a[trace.index(t_off + 0.005)] == 0.0
    assert np.all(trace.window("current_a", t_off + 0.005, 3.0) == 0.0)   # no braking torque
    assert 0 < trace.at("speed_rpm", 3.0) < trace.at("speed_rpm", t_off)  # still turning, slowing down


# ------------------------------------------------------------------ protection
@pytest.mark.req("REQ-06")
def test_current_limit_holds_for_setpoint_step_without_ramp(make_drive):
    drive, sim = make_drive(ramp_up_s=0.0, ramp_down_s=0.0)
    trace = sim.run(2.0, [(0.0, "ON 3000 (no ramp)", start(3000)),
                          (1.0, "step to -3000", lambda d: d.set_speed(-3000))])
    assert np.max(np.abs(trace.current_a)) <= 15.0 * 1.02
    assert DriveState.FAULT not in trace.state


@pytest.mark.req("REQ-06")
def test_locked_rotor_runs_at_current_limit_without_trip(make_drive):
    drive, sim = make_drive()
    trace = sim.run(2.0, [(0.0, "jam shaft + ON", lambda d: (d.inject_fault("locked_rotor"), start(3000)(d)))])
    assert np.max(np.abs(trace.current_a)) <= 15.0 * 1.02
    assert trace.at("current_a", 2.0) == pytest.approx(15.0, abs=0.1)   # producing max torque
    assert drive.fault is None


@pytest.mark.req("REQ-11")
def test_overcurrent_trips_within_1ms_and_current_decays(make_drive):
    drive, sim = make_drive()
    t_fault = 1.0
    trace = sim.run(1.2, [(0.0, "ON", start(1000)),
                          (t_fault, "inject output stage short", lambda d: d.inject_fault("output_stage_short"))])

    exceeded = trace.first_time(np.abs(trace.current_a) > 20.0, after=t_fault)
    assert exceeded is not None, "fault injection did not cause overcurrent"
    assert drive.fault == FaultCode.OVERCURRENT
    assert drive.fault_time - exceeded <= 1e-3
    assert trace.at("voltage_v", drive.fault_time + 1e-4) <= 0          # pulses off (diodes only)
    assert trace.at("current_a", drive.fault_time + 5e-3) == 0.0


@pytest.mark.req("REQ-12")
def test_overhauling_load_trips_overspeed(make_drive):
    drive, sim = make_drive()
    # An overhauling load (e.g. lowering a crane load) pushes the motor forwards.
    # The mechanical holding brake engages once the drive has tripped.
    load = lambda t: -3.0 if (t >= 2.5 and drive.state != DriveState.FAULT) else 0.0
    trace = sim.run(3.5, [(0.0, "ON 3000", start(3000))], load=load)

    assert drive.fault == FaultCode.OVERSPEED
    assert trace.at("speed_rpm", drive.fault_time) == pytest.approx(3300, abs=15)
    assert not drive.pulses_enabled


# ---------------------------------------------------------- fault handling
def _tripped_by_overspeed(make_drive):
    drive, sim = make_drive()
    load = lambda t: -3.0 if t >= 2.5 and drive.state != DriveState.FAULT else 0.0
    sim.run(2.6, [(0.0, "ON", start(3000))], load=load)
    assert drive.state == DriveState.FAULT
    return drive, sim


@pytest.mark.req("REQ-13")
def test_on_is_rejected_while_fault_is_latched(make_drive):
    drive, sim = _tripped_by_overspeed(make_drive)
    assert drive.on() is False
    sim.run(0.5)
    assert drive.state == DriveState.FAULT


@pytest.mark.req("REQ-13")
def test_acknowledge_rejected_while_cause_present_then_accepted(make_drive):
    drive, sim = make_drive()
    brake_engaged = False
    load = lambda t: 0.0 if brake_engaged or t < 2.5 else -3.0     # overhauling load, no brake yet
    sim.run(2.53, [(0.0, "ON", start(3000))], load=load)   # trips at ~2.519 s
    assert drive.fault == FaultCode.OVERSPEED
    assert abs(drive.speed_rpm) > 3300              # load is still driving the motor
    assert drive.acknowledge() is False
    assert drive.state == DriveState.FAULT

    brake_engaged = True                            # load removed; motor slows down by friction
    sim.run(2.0, load=load)
    assert abs(drive.speed_rpm) < 3300
    assert drive.acknowledge() is True
    assert drive.state == DriveState.READY and drive.fault is None


@pytest.mark.req("REQ-13", "REQ-14")
def test_overcurrent_fault_needs_acknowledge_and_new_on(make_drive):
    drive, sim = make_drive()
    sim.run(1.1, [(0.0, "ON", start(1000)), (1.0, "short", lambda d: d.inject_fault("output_stage_short"))])
    assert drive.fault == FaultCode.OVERCURRENT
    drive.clear_injected_faults()                      # "repair" the output stage
    assert drive.acknowledge() is True
    sim.run(0.5)
    assert drive.state == DriveState.READY             # no automatic restart after acknowledge
    assert drive.on() is True
    trace = sim.run(1.5)
    assert trace.at("speed_rpm", trace.t[-1]) == pytest.approx(1000, abs=3)


@pytest.mark.req("REQ-14")
@pytest.mark.parametrize("stop", ["off1", "off2", "off3"])
def test_no_automatic_restart_after_stop(make_drive, stop):
    drive, sim = make_drive()
    sim.run(5.0, [(0.0, "ON", start(1500)), (1.5, stop.upper(), lambda d: getattr(d, stop)())])
    assert drive.setpoint_rpm == 1500                  # setpoint still present ...
    assert drive.state == DriveState.READY             # ... but the drive stays off
    assert drive.voltage == 0.0
