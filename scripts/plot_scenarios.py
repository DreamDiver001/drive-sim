import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from drivesim import Drive, DriveConfig, DriveState, Simulation  

OUT = ROOT / "docs" / "img"

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
CRITICAL = "#d03b3b"
INK, INK_2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": MUTED, "ytick.color": MUTED, "font.size": 10,
    "lines.linewidth": 2, "legend.frameon": False, "legend.labelcolor": INK_2,
})


def start(rpm):
    def _a(d):
        d.set_speed(rpm)
        d.on()
    return _a


def event_line(ax, t, label, y_frac=0.97):
    ax.axvline(t, color=MUTED, lw=1, ls=":")
    ax.text(t + 0.04, y_frac, label, transform=ax.get_xaxis_transform(), color=INK_2,
            fontsize=9, va="top")


def overview():
    d = Drive()
    tr = Simulation(d).run(7.0, [(0.5, "ON 3000 rpm", start(3000)), (5.0, "OFF1", lambda d: d.off1())],
                           load=lambda t: 1.0 if 3.0 <= t < 4.3 else 0.0)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    a1.plot(tr.t, tr.setpoint_rpm, color=MUTED, lw=1.5, ls="--", label="Setpoint")
    a1.plot(tr.t, tr.ramp_rpm, color=ORANGE, label="Ramp output")
    a1.plot(tr.t, tr.speed_rpm, color=BLUE, label="Actual speed")
    a1.set_ylabel("Speed [rpm]")
    a1.set_title("Ramp-up, rated-load step and OFF1 stop")
    a1.legend(loc="center left", bbox_to_anchor=(0.62, 0.45))
    a1.set_ylim(-150, 3500)
    for t, lbl in [(0.5, "ON"), (3.0, "rated load on"), (4.3, "load off"), (5.0, "OFF1")]:
        event_line(a1, t, lbl)
    a2.plot(tr.t, tr.current_a, color=BLUE, label="Armature current")
    a2.axhline(15, color=CRITICAL, lw=1, ls="--")
    a2.text(0.05, 15.4, "current limit 15 A", color=CRITICAL, fontsize=9)
    a2.set_ylabel("Current [A]")
    a2.set_xlabel("Time [s]")
    a2.set_ylim(-4, 18)
    fig.tight_layout()
    fig.savefig(OUT / "overview.png", dpi=150)
    plt.close(fig)


def precontrol_finding():
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for enabled, color, label in [(False, ORANGE, "Before fix (PI only)"), (True, BLUE, "After fix (with acceleration precontrol)")]:
        d = Drive(DriveConfig(accel_precontrol=enabled))
        tr = Simulation(d).run(1.5, [(0.0, "ON 500", start(500))])
        ax.plot(tr.t, tr.speed_rpm, color=color, label=label)
        peak = tr.speed_rpm.max()
        ax.annotate(f"peak {peak:.1f} rpm", (tr.t[tr.speed_rpm.argmax()], peak),
                    xytext=(14, 4) if not enabled else (30, -22),
                    textcoords="offset points", color=INK_2, fontsize=9)
    ax.axhline(505, color=CRITICAL, lw=1, ls="--")
    ax.text(0.86, 505.4, "REQ-03 limit: +1 % (505 rpm)", color=CRITICAL, fontsize=9)
    ax.set_xlim(0.2, 1.0)
    ax.set_ylim(470, 512)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [rpm]")
    ax.set_title("FINDING-01: overshoot at 500 rpm, before and after the fix")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(OUT / "finding_precontrol.png", dpi=150)
    plt.close(fig)


def stop_modes():
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for mode, color in [("off1", BLUE), ("off2", ORANGE), ("off3", AQUA)]:
        d = Drive()
        tr = Simulation(d).run(5.0, [(0.0, "ON", start(3000)), (2.5, mode, lambda d, m=mode: getattr(d, m)())])
        ax.plot(tr.t - 2.5, tr.speed_rpm, color=color, label=mode.upper())
    ax.set_xlim(-0.3, 2.5)
    ax.set_xlabel("Time after stop command [s]")
    ax.set_ylabel("Speed [rpm]")
    ax.set_title("Stop modes from 3000 rpm")
    ax.text(0.52, 300, "OFF3 fast stop\n0.5 s ramp", color=INK_2, fontsize=9)
    ax.text(2.02, 300, "OFF1\n2.0 s ramp", color=INK_2, fontsize=9)
    ax.text(1.6, 2650, "OFF2 coasts (no torque)", color=INK_2, fontsize=9)
    ax.legend(loc="center right")
    fig.tight_layout()
    fig.savefig(OUT / "stop_modes.png", dpi=150)
    plt.close(fig)


def protection():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    d = Drive()
    tr = Simulation(d).run(1.01, [(0.0, "ON", start(1000)), (1.0, "short", lambda d: d.inject_fault("output_stage_short"))])
    ms = (tr.t - 1.0) * 1000
    a1.plot(ms, tr.current_a, color=BLUE)
    a1.axhline(20, color=CRITICAL, lw=1, ls="--")
    a1.text(-0.9, 20.6, "trip 20 A", color=CRITICAL, fontsize=9)
    a1.axvline((d.fault_time - 1.0) * 1000, color=MUTED, lw=1, ls=":")
    a1.text((d.fault_time - 1.0) * 1000 + 0.75, 6, f"F01 trip {(d.fault_time - 1.0) * 1000:.1f} ms\nafter injection,\npulses off", color=INK_2, fontsize=9)
    a1.set_xlim(-1, 4)
    a1.set_ylim(-1, 24)
    a1.set_xlabel("Time after fault injection [ms]")
    a1.set_ylabel("Current [A]")
    a1.set_title("Overcurrent: output stage short")

    d = Drive()
    load = lambda t: -3.0 if t >= 2.5 and d.state != DriveState.FAULT else 0.0
    tr = Simulation(d).run(2.6, [(0.0, "ON", start(3000))], load=load)
    ms = (tr.t - 2.5) * 1000
    a2.plot(ms, tr.speed_rpm, color=BLUE)
    a2.axhline(3300, color=CRITICAL, lw=1, ls="--")
    a2.text(-18, 3310, "trip 3300 rpm", color=CRITICAL, fontsize=9)
    a2.axvline((d.fault_time - 2.5) * 1000, color=MUTED, lw=1, ls=":")
    a2.text((d.fault_time - 2.5) * 1000 + 2, 3050, "F02 trip,\nbrake engages", color=INK_2, fontsize=9)
    a2.set_xlim(-20, 60)
    a2.set_ylim(2950, 3400)
    a2.set_xlabel("Time after overhauling load applied [ms]")
    a2.set_ylabel("Speed [rpm]")
    a2.set_title("Overspeed: overhauling load")
    fig.tight_layout()
    fig.savefig(OUT / "protection.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    overview()
    precontrol_finding()
    stop_modes()
    protection()
    print(f"figures written to {OUT}")
