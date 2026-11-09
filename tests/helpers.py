"""Small helpers for building test scenarios."""
import numpy as np

from drivesim import DriveState


def start(rpm):
    """Scheduled action: set the speed setpoint and switch on."""
    def _action(d):
        d.set_speed(rpm)
        d.on()
    return _action


def stop_time(trace, after):
    """Time when the drive reached READY (pulses off) after `after`, or None."""
    mask = np.array([s == DriveState.READY for s in trace.state])
    return trace.first_time(mask, after=after)
