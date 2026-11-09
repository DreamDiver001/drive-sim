"""Unit helpers. Users work in rpm; the physics works in rad/s."""
import math

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm / RPM_PER_RAD_S


def rad_s_to_rpm(omega: float) -> float:
    return omega * RPM_PER_RAD_S
