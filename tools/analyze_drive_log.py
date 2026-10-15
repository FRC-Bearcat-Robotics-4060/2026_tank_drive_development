#!/usr/bin/env python3
"""
Analyze a WPILib .wpilog from the tank-drive robot (simulation or real robot).

Checks the drivetrain software against its spec and measures the (simulated) dynamics:
  1. Followers output exactly what their leaders output (PWM 8 == PWM 3, PWM 5 == PWM 4)
  2. |output| never exceeds kMaxOutput
  3. Stick -> output mapping matches deadband + squaring + max output
  4. Outputs are 0 whenever the robot is disabled
  5. Right side is inverted on the PWM pin (right forward -> negative PWM)
  6. Step-response segments: top speed, time to 63% / 90%, yaw rate, turn direction
  7. Battery sag and current draw (simulation)

Usage:
  pip install robotpy-wpiutil numpy matplotlib
  python analyze_drive_log.py path/to/FRC_xxx.wpilog   (or a folder: newest .wpilog is used)
  python analyze_drive_log.py LOG --out report_dir --max-output 0.5 --deadband 0.05

Outputs report.md plus PNG plots in the --out folder (default: next to the log).
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    from wpiutil.log import DataLogReader
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency: pip install robotpy-wpiutil")

LOOP_S = 0.02

# Log entry names (NetworkTables entries are logged as "NT:" + topic path).
SD = "NT:/SmartDashboard/Drive/"
E = {
    "enabled": "DS:enabled",
    "auto": "DS:autonomous",
    "test": "DS:test",
    "axes": "DS:joystick0/axes",
    "left_cmd": SD + "Left Output",
    "right_cmd": SD + "Right Output",
    "pwm_ll": SD + "PWM/Left Leader",
    "pwm_lf": SD + "PWM/Left Follower",
    "pwm_rl": SD + "PWM/Right Leader",
    "pwm_rf": SD + "PWM/Right Follower",
    "batt": SD + "Battery Volts",
    "x": SD + "Sim/X m",
    "y": SD + "Sim/Y m",
    "heading": SD + "Sim/Heading deg",
    "vl": SD + "Sim/Left Velocity mps",
    "vr": SD + "Sim/Right Velocity mps",
    "amps": SD + "Sim/Current A",
}


# --------------------------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------------------------
@dataclass
class Series:
    name: str
    type: str
    t: list = field(default_factory=list)
    v: list = field(default_factory=list)


def read_log(path: Path) -> dict[str, Series]:
    reader = DataLogReader(str(path))
    if not reader.isValid():
        sys.exit(f"Not a valid .wpilog: {path}")
    entries: dict[int, Series] = {}
    out: dict[str, Series] = {}
    for rec in reader:
        if rec.isStart():
            d = rec.getStartData()
            s = Series(d.name, d.type)
            entries[d.entry] = s
            out.setdefault(d.name, s)
            continue
        if rec.isControl():
            continue
        s = entries.get(rec.getEntry())
        if s is None:
            continue
        t = rec.getTimestamp() / 1e6
        try:
            if s.type == "double":
                val = rec.getDouble()
            elif s.type == "float":
                val = rec.getFloat()
            elif s.type == "boolean":
                val = float(rec.getBoolean())
            elif s.type == "int64":
                val = float(rec.getInteger())
            elif s.type == "float[]":
                val = list(rec.getFloatArray())
            elif s.type == "double[]":
                val = list(rec.getDoubleArray())
            elif s.type == "struct:Pose2d":
                raw = bytes(rec.getRaw())
                val = list(struct.unpack("<3d", raw[:24])) if len(raw) >= 24 else None
            else:
                continue
        except Exception:
            continue
        if val is None:
            continue
        out[s.name].t.append(t)
        out[s.name].v.append(val)
    return out


def hold(series: Series | None, grid: np.ndarray, default=np.nan) -> np.ndarray:
    """Sample-and-hold a change-only series onto a uniform time grid."""
    if series is None or not series.t:
        return np.full(grid.shape, default, dtype=float)
    t = np.asarray(series.t)
    order = np.argsort(t, kind="stable")
    t = t[order]
    v = np.asarray(series.v, dtype=float)[order]
    idx = np.searchsorted(t, grid, side="right") - 1
    res = np.where(idx >= 0, v[np.clip(idx, 0, None)], default)
    return res.astype(float)


def hold_axis(series: Series | None, grid: np.ndarray, axis: int) -> tuple[np.ndarray, bool]:
    """Sample-and-hold one joystick axis. Returns (values, axis_present)."""
    if series is None or not series.t:
        return np.zeros(grid.shape), False
    present = any(len(a) > axis for a in series.v)
    vals = [a[axis] if len(a) > axis else 0.0 for a in series.v]
    return hold(Series(series.name, "double", series.t, vals), grid, 0.0), present


# --------------------------------------------------------------------------------------------
# Expected behavior (mirrors DifferentialDrive.tankDrive with squared inputs)
# --------------------------------------------------------------------------------------------
def expected_output(stick: np.ndarray, deadband: float, max_output: float, square: bool) -> np.ndarray:
    x = np.clip(stick, -1, 1)
    mag = np.abs(x)
    db = np.where(mag > deadband, np.sign(x) * (mag - deadband) / (1 - deadband), 0.0)
    if square:
        db = np.sign(db) * db * db
    return db * max_output


# --------------------------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------------------------
def segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """Index ranges [start, end) where mask is True."""
    segs, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            segs.append((start, i))
            start = None
    if start is not None:
        segs.append((start, len(mask)))
    return segs


def rise_time(t: np.ndarray, y: np.ndarray, frac: float) -> float:
    final = np.mean(y[-max(3, len(y) // 6):])
    if abs(final) < 1e-6:
        return math.nan
    hit = np.nonzero(np.abs(y) >= frac * abs(final))[0]
    return float(t[hit[0]] - t[0]) if len(hit) else math.nan


def fmt(x, nd=3):
    return "—" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def analyze(log: Path, out_dir: Path, max_output: float, deadband: float, square: bool,
            left_axis: int, right_axis: int) -> Path:
    data = read_log(log)
    out_dir.mkdir(parents=True, exist_ok=True)
    g = lambda k: data.get(E[k])
    lines: list[str] = [f"# Drive log report\n", f"Log: `{log.name}`\n"]

    missing = [k for k in ("enabled", "pwm_ll", "pwm_lf", "pwm_rl", "pwm_rf", "left_cmd")
               if g(k) is None]
    if missing:
        lines.append(f"**Missing entries:** {', '.join(E[m] for m in missing)} — "
                     "was this log made with the tank-drive branch?\n")

    # Uniform grid over the whole log
    all_t = [t for s in data.values() for t in s.t]
    if not all_t:
        sys.exit("Log has no data")
    t0, t1 = min(all_t), max(all_t)
    grid = np.arange(t0, t1, LOOP_S)
    tt = grid - t0

    en = hold(g("enabled"), grid, 0.0) > 0.5
    auto = hold(g("auto"), grid, 0.0) > 0.5
    test = hold(g("test"), grid, 0.0) > 0.5
    teleop = en & ~auto & ~test
    ax_l, has_l = hold_axis(g("axes"), grid, left_axis)
    ax_r, has_r = hold_axis(g("axes"), grid, right_axis)
    cmd_l, cmd_r = hold(g("left_cmd"), grid), hold(g("right_cmd"), grid)
    p_ll, p_lf = hold(g("pwm_ll"), grid), hold(g("pwm_lf"), grid)
    p_rl, p_rf = hold(g("pwm_rl"), grid), hold(g("pwm_rf"), grid)
    batt, amps = hold(g("batt"), grid), hold(g("amps"), grid)
    x, y = hold(g("x"), grid), hold(g("y"), grid)
    hdg = np.unwrap(np.radians(hold(g("heading"), grid, 0.0)))
    vl, vr = hold(g("vl"), grid), hold(g("vr"), grid)
    is_sim = g("x") is not None

    lines.append(f"Duration {t1 - t0:.1f} s · enabled {en.sum() * LOOP_S:.1f} s · "
                 f"teleop {teleop.sum() * LOOP_S:.1f} s · {'simulation' if is_sim else 'real robot'}\n")

    # Settle mask: skip the loop right after any input/mode change (log ordering jitter).
    change = np.zeros_like(en)
    for arr in (ax_l, ax_r, en.astype(float)):
        d = np.abs(np.diff(arr, prepend=arr[0])) > 1e-9
        change |= d | np.roll(d, 1) | np.roll(d, 2)
    steady = ~change

    checks: list[tuple[str, bool | None, str]] = []

    # 1. followers
    ok_mask = ~np.isnan(p_ll) & ~np.isnan(p_lf) & ~np.isnan(p_rl) & ~np.isnan(p_rf)
    if ok_mask.any():
        dl = np.max(np.abs(p_ll - p_lf)[ok_mask & steady], initial=0)
        dr = np.max(np.abs(p_rl - p_rf)[ok_mask & steady], initial=0)
        checks.append(("Followers match leaders (PWM8=PWM3, PWM5=PWM4)", dl < 1e-6 and dr < 1e-6,
                       f"max |Δleft| = {dl:.2e}, max |Δright| = {dr:.2e}"))
        peak = np.nanmax(np.abs(np.r_[p_ll, p_lf, p_rl, p_rf]))
        checks.append((f"|PWM| ≤ max output {max_output}", peak <= max_output + 1e-6,
                       f"peak |PWM| = {peak:.3f}"))

    # 3. mapping
    m = teleop & steady & ~np.isnan(cmd_l) & ~np.isnan(cmd_r)
    if m.any():
        exp_l = expected_output(-ax_l, deadband, max_output, square)
        exp_r = expected_output(-ax_r, deadband, max_output, square)
        el, er = np.abs(cmd_l - exp_l)[m], np.abs(cmd_r - exp_r)[m]
        frac = np.mean((el < 1e-3) & (er < 1e-3))
        checks.append(("Stick → output mapping (deadband, squaring, max output)", frac > 0.98,
                       f"{frac * 100:.1f}% of {m.sum()} steady teleop samples match "
                       f"(max err L {el.max():.3f}, R {er.max():.3f})"))
        if not has_r:
            checks.append((f"Right stick axis {right_axis} present", False,
                           f"joystick 0 reports fewer than {right_axis + 1} axes → right side always 0 "
                           "(configure Keyboard 0 Axis 5 or use an Xbox controller)"))
        active = m & ((np.abs(exp_l) > 0) | (np.abs(exp_r) > 0))
        lines.append(f"Samples with stick input during teleop: {active.sum()}\n")

    # 4. disabled => 0
    dis = ~en & steady & ~np.isnan(p_ll)
    if dis.any():
        mx = np.nanmax(np.abs(np.c_[p_ll, p_lf, p_rl, p_rf][dis]))
        checks.append(("All outputs 0 while disabled", mx < 1e-6, f"max |PWM| disabled = {mx:.3f}"))

    # 5. inversion sign
    fwd_r = teleop & steady & (cmd_r > 0.05)
    if fwd_r.any():
        ok = np.all(p_rl[fwd_r] < 0)
        checks.append(("Right side inverted on PWM (right forward → PWM4 < 0)", bool(ok),
                       f"{fwd_r.sum()} samples"))
    fwd_l = teleop & steady & (cmd_l > 0.05)
    if fwd_l.any():
        ok = np.all(p_ll[fwd_l] > 0)
        checks.append(("Left side not inverted (left forward → PWM3 > 0)", bool(ok),
                       f"{fwd_l.sum()} samples"))

    lines.append("\n## Checks\n\n| Check | Result | Detail |\n|---|---|---|")
    for name, ok, detail in checks:
        ok = None if ok is None else bool(ok)
        res = "✅ pass" if ok else ("❌ FAIL" if ok is False else "—")
        lines.append(f"| {name} | {res} | {detail} |")

    # 6. step segments
    seg_rows = []
    if is_sim:
        yaw_rate = np.degrees(np.gradient(hdg, LOOP_S))
        lin = (np.abs(vl) + np.abs(vr)) / 2  # mean wheel speed: meaningful for straight, turn and spin
        q_l = np.round(cmd_l, 2)
        q_r = np.round(cmd_r, 2)
        const = teleop & ~np.isnan(q_l)
        key = np.where(const, q_l * 1000 + q_r, np.nan)
        segs = []
        start = 0
        for i in range(1, len(key) + 1):
            if i == len(key) or not (key[i] == key[start]):
                if not np.isnan(key[start]) and (i - start) * LOOP_S >= 0.8 and (q_l[start] or q_r[start]):
                    segs.append((start, i))
                start = i
        for a, b in segs:
            l, r = q_l[a], q_r[a]
            kind = ("straight forward" if l > 0 and abs(l - r) < 1e-6 else
                    "straight backward" if l < 0 and abs(l - r) < 1e-6 else
                    "spin in place" if abs(l + r) < 1e-6 else
                    "left only" if r == 0 else "right only" if l == 0 else "arc")
            ts = tt[a:b]
            yr = float(np.mean(yaw_rate[b - max(3, (b - a) // 6):b]))
            expected_dir = None
            if kind in ("left only",) or (kind == "spin in place" and l > 0) or (kind == "arc" and l > r):
                expected_dir = "CW"
            elif kind in ("right only",) or (kind == "spin in place" and l < 0) or (kind == "arc" and r > l):
                expected_dir = "CCW"
            actual_dir = "CW" if yr < -1 else "CCW" if yr > 1 else "straight"
            dir_ok = (expected_dir is None and actual_dir == "straight") or expected_dir == actual_dir
            seg_rows.append(
                f"| {ts[0]:.1f}–{ts[-1]:.1f} | {kind} | {l:+.2f} / {r:+.2f} | "
                f"{fmt(float(np.mean(lin[b - max(3, (b - a) // 6):b])), 2)} | "
                f"{fmt(rise_time(ts, lin[a:b], 0.63), 2)} | {fmt(rise_time(ts, lin[a:b], 0.9), 2)} | "
                f"{yr:+.0f} | {actual_dir} {'✅' if dir_ok else '❌'} |")
        lines.append("\n## Step segments (simulation physics)\n")
        if seg_rows:
            lines.append("| t (s) | Maneuver | L / R output | Mean wheel speed (m/s) | t63 (s) | t90 (s) | "
                         "Yaw rate (°/s) | Turn dir |\n|---|---|---|---|---|---|---|---|")
            lines += seg_rows
            lines.append("\nCW = clockwise (right turn), CCW = counter-clockwise (left turn). "
                         "Physics uses the KOP kitbot preset, so absolute speeds are placeholders.")
        else:
            lines.append("No constant-input segments ≥ 0.8 s found. Hold each input steady for ~3 s.")

        lines.append("\n## Battery / current\n")
        lines.append(f"Min battery {fmt(float(np.nanmin(batt)), 2)} V · "
                     f"peak current {fmt(float(np.nanmax(amps)), 1)} A · "
                     f"distance driven {fmt(float(np.nansum(np.hypot(np.diff(x), np.diff(y)))), 2)} m")

    # Plots
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        n = 4 if is_sim else 2
        fig, axs = plt.subplots(n, 1, figsize=(11, 2.4 * n), sharex=True)
        shade = lambda a: [a.axvspan(tt[s], tt[e - 1], color="0.92", lw=0) for s, e in segments(en)]
        shade(axs[0])
        axs[0].plot(tt, -ax_l, label=f"left stick (−axis {left_axis})")
        axs[0].plot(tt, -ax_r, label=f"right stick (−axis {right_axis})")
        axs[0].set_ylabel("stick")
        axs[0].legend(loc="upper right", fontsize=8)
        shade(axs[1])
        for arr, lab, st in ((p_ll, "PWM3 L lead", "-"), (p_lf, "PWM8 L follow", ":"),
                             (p_rl, "PWM4 R lead", "-"), (p_rf, "PWM5 R follow", ":")):
            axs[1].plot(tt, arr, st, label=lab)
        axs[1].axhline(max_output, color="0.5", lw=0.6, ls="--")
        axs[1].axhline(-max_output, color="0.5", lw=0.6, ls="--")
        axs[1].set_ylabel("PWM")
        axs[1].legend(loc="upper right", fontsize=8, ncol=2)
        if is_sim:
            shade(axs[2])
            axs[2].plot(tt, vl, label="left m/s")
            axs[2].plot(tt, vr, label="right m/s")
            ax2b = axs[2].twinx()
            ax2b.plot(tt, np.degrees(np.gradient(hdg, LOOP_S)), color="0.4", lw=0.8, label="yaw °/s")
            ax2b.set_ylabel("yaw °/s")
            axs[2].set_ylabel("wheel m/s")
            axs[2].legend(loc="upper left", fontsize=8)
            shade(axs[3])
            axs[3].plot(tt, batt, label="battery V")
            ax3b = axs[3].twinx()
            ax3b.plot(tt, amps, color="0.4", lw=0.8)
            ax3b.set_ylabel("current A")
            axs[3].set_ylabel("battery V")
        axs[-1].set_xlabel("time (s)  — grey = enabled")
        fig.tight_layout()
        fig.savefig(out_dir / "timeseries.png", dpi=120)
        plt.close(fig)
        lines.append("\n![timeseries](timeseries.png)")

        if is_sim:
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.set_xlim(0, 16.54)
            ax.set_ylim(0, 8.07)
            ax.set_aspect("equal")
            sc = ax.scatter(x, y, c=tt, s=3, cmap="viridis")
            ax.plot(x[0], y[0], "go", label="start")
            ax.plot(x[-1], y[-1], "rs", label="end")
            fig.colorbar(sc, ax=ax, label="time (s)")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title("Simulated path (field 16.54 × 8.07 m)")
            ax.legend(fontsize=8)
            fig.tight_layout()
            fig.savefig(out_dir / "path.png", dpi=120)
            plt.close(fig)
            lines.append("\n![path](path.png)")
    except ImportError:
        lines.append("\n(matplotlib not installed — no plots)")

    report = out_dir / "report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", type=Path, help=".wpilog file or a folder containing them (newest is used)")
    p.add_argument("--out", type=Path, default=None, help="output folder (default: <log>_report)")
    p.add_argument("--max-output", type=float, default=0.5)
    p.add_argument("--deadband", type=float, default=0.05)
    p.add_argument("--no-square", action="store_true")
    p.add_argument("--left-axis", type=int, default=1)
    p.add_argument("--right-axis", type=int, default=5)
    a = p.parse_args()
    log = a.log
    if log.is_dir():
        logs = sorted(log.glob("*.wpilog"), key=lambda f: f.stat().st_mtime)
        if not logs:
            sys.exit(f"No .wpilog in {log}")
        log = logs[-1]
    out = a.out or log.with_name(log.stem + "_report")
    rep = analyze(log, out, a.max_output, a.deadband, not a.no_square, a.left_axis, a.right_axis)
    print(f"Report: {rep}")


if __name__ == "__main__":
    main()
