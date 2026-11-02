"""
Compare a real robot recording against the simulated robot, joint by joint.

It gives the simulated robot (BAM servo model on every joint) the exact same
commands the real robot got, then compares where each joint actually went.

Inputs:
  --robot      the robot's MuJoCo file (.xml). Each servo must be a <motor>
               with the SAME name as its joint.
  --recording  a .csv with a "time" column (seconds) and, for every joint,
               "<joint>_goal" (commanded angle) and "<joint>_actual" (measured angle)
  --models     servo models to try. Defaults to BAM's ready-made STS3215
               m1 (MuJoCo's usual simple friction) and m6 (BAM's detailed one).
               Add your own fitted file from bam_pipeline.py, e.g. results/fitted_m1.json

Run from the BAM folder:
    uv run python robot_compare.py --robot fake_robot.xml --recording fake_robot_recording.csv
    uv run python robot_compare.py --robot robot.xml --recording run1.csv --models bundled:m6 results/fitted_m1.json

Everything goes into robot_results/ (start with report.md).
"""

import argparse
import csv
import json
import os
import sys

import mujoco
import numpy as np

from bam.model import load_model
from bam.mujoco import MujocoController

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--robot", required=True, help="robot MuJoCo .xml")
p.add_argument("--recording", required=True, help="recording .csv")
p.add_argument("--models", nargs="+", default=["bundled:m1", "bundled:m6"],
               help="servo models: bundled:m1 ... bundled:m6, or a params .json file")
p.add_argument("--bundled", default="feetech_sts3215_7_4V", help="which bundled servo")
p.add_argument("--kp", type=int, default=32, help="servo P-gain used on the robot")
p.add_argument("--vin", type=float, default=7.4, help="supply voltage on the robot [V]")
p.add_argument("--units", choices=["deg", "rad"], default="deg", help="angle units in the recording")
p.add_argument("--offset", action="append", default=[], metavar="JOINT=DEG",
               help="zero-position correction for a joint, subtracted from its readings, e.g. --offset elbow=2.5")
p.add_argument("--out", default="robot_results")
args = p.parse_args()
os.makedirs(args.out, exist_ok=True)

# ---------------------------------------------------------------------------
# 1. Read the recording
# ---------------------------------------------------------------------------
with open(args.recording) as f:
    rows = list(csv.DictReader(f))
if not rows or "time" not in rows[0]:
    sys.exit("The recording needs a 'time' column.")
cols = rows[0].keys()
rec_joints = [c[:-5] for c in cols if c.endswith("_goal") and f"{c[:-5]}_actual" in cols]
if not rec_joints:
    sys.exit("No '<joint>_goal' + '<joint>_actual' column pairs found in the recording.")

to_rad = np.radians if args.units == "deg" else (lambda x: np.asarray(x, dtype=float))
t_rec = np.array([float(r["time"]) for r in rows])
t_rec = t_rec - t_rec[0]
goal = {j: to_rad(np.array([float(r[f"{j}_goal"]) for r in rows])) for j in rec_joints}
real = {j: to_rad(np.array([float(r[f"{j}_actual"]) for r in rows])) for j in rec_joints}

offsets = {}
for item in args.offset:
    name, val = item.split("=")
    offsets[name] = np.radians(float(val))
    real[name] = real[name] - offsets[name]

# ---------------------------------------------------------------------------
# 2. Match joints between the recording and the robot file
# ---------------------------------------------------------------------------
probe = mujoco.MjModel.from_xml_path(args.robot)
motors = {probe.actuator(i).name for i in range(probe.nu)}
joints = [j for j in rec_joints if j in motors]
warnings = []
for j in rec_joints:
    if j not in motors:
        warnings.append(f"Joint '{j}' is in the recording but has no <motor> with that name in the robot file - skipped.")
for mname in sorted(motors - set(rec_joints)):
    warnings.append(f"Motor '{mname}' is in the robot file but not in the recording - it gets no commands.")
if not joints:
    sys.exit("No joint names match between the recording and the robot file.\n" + "\n".join(warnings))

# Unit sanity check: degrees recordings almost always move more than ~3 units
span = max(np.ptp(goal[j]) for j in joints)
if args.units == "deg" and np.degrees(span) < 3.5:
    warnings.append("The commands barely move (under 3.5 deg). Is the recording in radians? Try --units rad.")
if args.units == "rad" and span > 2 * np.pi:
    warnings.append("The commands move more than a full turn. Is the recording in degrees? Try --units deg.")

print(f"Recording: {len(rows)} readings over {t_rec[-1]:.1f} s, joints: {', '.join(joints)}")


# ---------------------------------------------------------------------------
# 3. Replay the commands in the simulation, once per servo model
# ---------------------------------------------------------------------------
def load_servo(spec):
    if spec.startswith("bundled:"):
        variant = spec.split(":")[1]
        return f"bundled {variant}", load_model(motor_name=args.bundled, model=variant)
    return os.path.splitext(os.path.basename(spec))[0], load_model(spec)


def simulate(servo):
    m = mujoco.MjModel.from_xml_path(args.robot)
    d = mujoco.MjData(m)
    servo.actuator.kp, servo.actuator.vin = args.kp, args.vin
    ctrl = MujocoController(servo, joints, m, d)
    for j in joints:  # start where the real robot started
        d.qpos[m.jnt_qposadr[m.joint(j).id]] = real[j][0]
    mujoco.mj_forward(m, d)
    ctrl.reset()
    qadr = [m.jnt_qposadr[m.joint(j).id] for j in joints]
    ts, qs = [], []
    while d.time < t_rec[-1]:
        for j in joints:
            ctrl.set_q_target(j, np.interp(d.time, t_rec, goal[j]))
        ctrl.update()
        mujoco.mj_step(m, d)
        ts.append(d.time)
        qs.append(d.qpos[qadr].copy())
    ts, qs = np.array(ts), np.array(qs)
    return {j: np.interp(t_rec, ts, qs[:, k]) for k, j in enumerate(joints)}


sims, errors = {}, {}  # model -> joint -> sim angles / error stats
for spec in args.models:
    name, servo = load_servo(spec)
    print(f"Simulating with {name} ...")
    sims[name] = simulate(servo)
    errors[name] = {}
    for j in joints:
        diff = np.degrees(sims[name][j] - real[j])
        errors[name][j] = {"avg": float(np.mean(np.abs(diff))), "max": float(np.max(np.abs(diff))),
                           "bias": float(np.mean(diff))}

names = list(sims)
overall = {n: float(np.mean([errors[n][j]["avg"] for j in joints])) for n in names}
best = min(names, key=overall.get)

# ---------------------------------------------------------------------------
# 4. Charts, spreadsheet, report
# ---------------------------------------------------------------------------
with open(os.path.join(args.out, "errors_per_joint.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["joint"] + [f"{n} {k} (deg)" for n in names for k in ("avg error", "max error", "bias")])
    for j in joints:
        w.writerow([j] + [round(errors[n][j][k], 3) for n in names for k in ("avg", "max", "bias")])

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK, INK2, GRID, REAL = "#0b0b0b", "#52514e", "#e4e3df", "#c4c2b9"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # fixed order, one per model
plt.rcParams.update({"axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.titlecolor": INK, "font.size": 10})


def tidy(ax):
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


# Chart 1: every joint, real vs simulated
fig, axes = plt.subplots(len(joints), 1, figsize=(9, 2.3 * len(joints) + 0.6), sharex=True)
axes = np.atleast_1d(axes)
for ax, j in zip(axes, joints):
    ax.plot(t_rec, np.degrees(real[j]), color=REAL, lw=6, solid_capstyle="round", label="real robot")
    ax.plot(t_rec, np.degrees(goal[j]), color=INK2, lw=1, ls=":", label="command")
    for k, n in enumerate(names):
        ax.plot(t_rec, np.degrees(sims[n][j]), color=SERIES[k % 4], lw=1.6, label=f"sim: {n}")
    ax.set_title(f"{j}", loc="left")
    ax.set_ylabel("angle (deg)")
    tidy(ax)
axes[-1].set_xlabel("time (s)")
h, l = axes[0].get_legend_handles_labels()
fig.legend(h, l, loc="upper center", ncol=len(l), frameon=False)
fig.tight_layout(rect=(0, 0, 1, 1 - 0.45 / (2.3 * len(joints) + 0.6)))
fig.savefig(os.path.join(args.out, "joints_real_vs_sim.png"), dpi=150)
plt.close(fig)

# Chart 2: average error per joint, per model
fig, ax = plt.subplots(figsize=(8, 3.6))
width = 0.8 / len(names)
x = np.arange(len(joints))
for k, n in enumerate(names):
    vals = [errors[n][j]["avg"] for j in joints]
    bars = ax.bar(x + (k - (len(names) - 1) / 2) * width, vals, width * 0.92,
                  color=SERIES[k % 4], label=n)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.2f}°", ha="center", va="bottom", fontsize=9, color=INK)
ax.set_xticks(x, joints)
ax.set_ylabel("average error (deg)")
ax.set_title("How far the simulation is from the real robot, per joint (lower is better)", loc="left")
ax.legend(frameon=False)
tidy(ax)
ax.grid(axis="x", visible=False)
fig.tight_layout()
fig.savefig(os.path.join(args.out, "error_per_joint.png"), dpi=150)
plt.close(fig)

# Report
L = ["# Robot vs. simulation report", "",
     f"- Recording: `{args.recording}` ({len(rows)} readings, {t_rec[-1]:.1f} s)",
     f"- Robot file: `{args.robot}`",
     f"- Joints compared: {', '.join(joints)}",
     f"- Servo settings: kp={args.kp}, {args.vin} V", ""]
if offsets:
    L.append("- Zero corrections applied: " + ", ".join(f"{k} {np.degrees(v):+.1f}°" for k, v in offsets.items()))
    L.append("")
L += ["## Average error per joint (degrees)", "",
      "| Joint | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
for j in joints:
    L.append(f"| {j} | " + " | ".join(f"{errors[n][j]['avg']:.2f}" for n in names) + " |")
L.append("| **all joints** | " + " | ".join(f"**{overall[n]:.2f}**" for n in names) + " |")
L += ["", "## Verdict", "",
      f"- **{best}** matches the real robot best (average {overall[best]:.2f}° across joints)."]
worst = max(joints, key=lambda j: errors[best][j]["avg"])
L.append(f"- The hardest joint to match is **{worst}** ({errors[best][worst]['avg']:.2f}° average, "
         f"{errors[best][worst]['max']:.2f}° at worst).")
for j in joints:
    e = errors[best][j]
    if abs(e["bias"]) > 1.5 and abs(e["bias"]) > 0.6 * e["avg"]:
        L.append(f"- **{j}** is consistently off by about {e['bias']:+.1f}° in the same direction. That usually "
                 f"means its zero position differs between the robot and the simulation. Try "
                 f"`--offset {j}={-e['bias']:.1f}` and run again.")
if warnings:
    L += ["", "## Warnings", ""] + [f"- {w_}" for w_ in warnings]
L += ["", "## Files", "",
      "- `joints_real_vs_sim.png` - each joint: real robot vs. simulation",
      "- `error_per_joint.png` - average error for each joint and model",
      "- `errors_per_joint.csv` - the numbers (opens in Excel/Sheets)"]
with open(os.path.join(args.out, "report.md"), "w") as f:
    f.write("\n".join(L) + "\n")

print("\n".join(L[L.index("## Average error per joint (degrees)"):L.index("## Files")]))
print(f"\nEverything saved in {args.out}/  (start with report.md)")
