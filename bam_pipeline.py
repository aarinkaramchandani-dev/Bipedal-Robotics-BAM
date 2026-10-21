"""
WEEK 6 - One-command pipeline: recordings in, answer out.

Give it a folder of servo recordings (fake ones from make_synthetic_data.py,
the BAM authors' real STS3215 recordings, or our own from the test rig) and it:

  1. Cleans them up          (BAM's processing step: steady 5 ms timing)
  2. Checks BAM's bundled STS3215 model against them  -> "does the ready-made model match?"
  3. Fits a new model to them (BAM's fitting tool)    -> "what are OUR servo's numbers?"
  4. Checks the fitted model, including on recordings it never trained on
  5. Saves charts, a spreadsheet of errors, and a plain-English report

Run from the BAM folder:
    uv run python bam_pipeline.py --raw data_raw_synthetic --truth true_params.json
    uv run python bam_pipeline.py --raw data_raw_real

Everything goes into the --out folder (default: results/).
"""

import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import time
from copy import deepcopy

import numpy as np

from bam.logs import Logs
from bam.model import load_model
from bam.simulate import Simulator

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--raw", required=True, help="folder of raw recordings (.json)")
p.add_argument("--out", default="results", help="where to save everything")
p.add_argument("--actuator", default="sts3215", help="BAM actuator name used for fitting")
p.add_argument("--bundled", default="feetech_sts3215_7_4V", help="BAM's bundled model folder to compare against")
p.add_argument("--model", default="m1", help="friction model to fit: m1 (simple, fast) ... m6 (detailed, slow)")
p.add_argument("--trials", type=int, default=3000, help="fitting attempts (more = better but slower; ~15/s)")
p.add_argument("--validation-kp", type=int, default=None,
               help="P-gain whose recordings are held back to test the fit (default: picked automatically)")
p.add_argument("--truth", default=None, help="answer key (true_params.json) - only for fake data")
p.add_argument("--skip-fit", action="store_true", help="only check the bundled model, don't fit")
args = p.parse_args()

OUT = args.out
PROCESSED = os.path.join(OUT, "data_processed")
FITTED = os.path.join(OUT, f"fitted_{args.model}.json")
os.makedirs(PROCESSED, exist_ok=True)


def step(n, text):
    print(f"\n=== Step {n}: {text} ===")


# ---------------------------------------------------------------------------
step(1, "clean up the recordings (BAM process)")
raw_files = glob.glob(os.path.join(args.raw, "*.json"))
if not raw_files:
    sys.exit(f"No .json recordings found in {args.raw}/")
for f in glob.glob(os.path.join(PROCESSED, "*.json")):
    os.remove(f)  # start fresh so old runs don't mix in
subprocess.run([sys.executable, "-m", "bam.process", "--raw", args.raw,
                "--logdir", PROCESSED, "--dt", "0.005"],
               check=True, stdout=subprocess.DEVNULL)
logs = Logs(PROCESSED)
print(f"{len(logs.logs)} recordings ready")
kps = sorted({log["kp"] for log in logs.logs})
print(f"P-gains found: {kps}")


# ---------------------------------------------------------------------------
def errors_deg(model_path, logs_obj):
    """Run the simulator on every recording at once; return per-recording
    average error in degrees, plus the simulated angles."""
    model = load_model(model_path)
    batch = logs_obj.make_batch()
    sim_q, _, _ = Simulator(model).rollout_log(batch, simulate_control=True)
    sim_q = np.array(sim_q, dtype=float)                       # (steps, recordings)
    real_q = np.array([e["position"] for e in batch["entries"]], dtype=float)
    err = np.degrees(np.mean(np.abs(sim_q - real_q), axis=0))
    return np.atleast_1d(err), sim_q, real_q


def bundled_path(variant):
    import bam
    return os.path.join(os.path.dirname(bam.__file__), "params", args.bundled, f"{variant}.json")


step(2, f"check BAM's bundled model ({args.bundled}) against the recordings")
results = {}  # name -> per-recording errors
for variant in sorted({"m1", "m6", args.model}):
    path = bundled_path(variant)
    if os.path.exists(path):
        err, _, _ = errors_deg(path, logs)
        results[f"bundled {variant}"] = err
        print(f"bundled {variant}: average error {err.mean():.2f} deg (worst recording {err.max():.2f} deg)")


# ---------------------------------------------------------------------------
val_kp = None
if not args.skip_fit:
    val_kp = args.validation_kp
    if val_kp is None and len(kps) >= 3:
        val_kp = kps[len(kps) // 2]       # hold back a middle gain
    step(3, f"fit a new {args.model} model ({args.trials} attempts"
            + (f", holding back kp={val_kp} to test it)" if val_kp else ")"))
    cmd = [sys.executable, "-m", "bam.fit", "--logdir", PROCESSED, "--actuator", args.actuator,
           "--model", args.model, "--trials", str(args.trials), "--output", FITTED]
    if val_kp:
        cmd += ["--validation_kp", str(val_kp)]
    t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    last = 0
    for line in proc.stdout:
        if "Best score" in line and time.time() - last > 5:
            last = time.time()
            print("  " + line.strip().replace("\x1b[1m", "").replace("\x1b[0m", ""), flush=True)
    if proc.wait() != 0:
        sys.exit("Fitting failed - scroll up for the error")
    print(f"Fit finished in {(time.time() - t0) / 60:.1f} min -> {FITTED}")

    step(4, "check the fitted model")
    err, _, _ = errors_deg(FITTED, logs)
    results[f"fitted {args.model}"] = err
    print(f"fitted {args.model}: average error {err.mean():.2f} deg (worst recording {err.max():.2f} deg)")


# ---------------------------------------------------------------------------
step(5, "save charts, spreadsheet and report")
names = list(results)
held = np.array([log["kp"] == val_kp for log in logs.logs]) if val_kp else np.zeros(len(logs.logs), bool)

# Spreadsheet: one row per recording
with open(os.path.join(OUT, "errors_per_recording.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["recording", "trajectory", "arm_length_m", "mass_kg", "kp", "held_back_for_testing"]
               + [f"{n} error (deg)" for n in names])
    for i, log in enumerate(logs.logs):
        w.writerow([os.path.basename(log["filename"]), log.get("trajectory", ""), log["length"],
                    log["mass"], log["kp"], bool(held[i])] + [round(results[n][i], 3) for n in names])

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
COLORS = {"recorded": "#c4c2b9", "bundled": "#eb6834", "fitted": "#2a78d6"}
plt.rcParams.update({"axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.titlecolor": INK, "font.size": 10})


def color_for(name):
    return COLORS["fitted"] if name.startswith("fitted") else COLORS["bundled"]


# Chart 1: average error per model
fig, ax = plt.subplots(figsize=(7, 0.6 * len(names) + 1.4))
means = [results[n].mean() for n in names]
y = np.arange(len(names))
ax.barh(y, means, height=0.5, color=[color_for(n) for n in names])
for yi, m in zip(y, means):
    ax.text(m, yi, f"  {m:.2f}°", va="center", color=INK)
ax.set_yticks(y, names)
ax.invert_yaxis()
ax.set_xlabel("average error between simulation and recordings (degrees, lower is better)")
ax.set_xlim(0, max(means) * 1.25)
ax.grid(axis="x", color=GRID, linewidth=0.8)
ax.set_axisbelow(True)
for s in ["top", "right"]:
    ax.spines[s].set_visible(False)
ax.set_title("How closely each model matches the recordings", loc="left")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "error_comparison.png"), dpi=150)
plt.close(fig)

# Chart 2: example recordings with the simulations drawn on top
compare = [n for n in names if n.startswith("fitted")] + [f"bundled {args.model}"]
compare = [n for n in compare if n in results]
sims = {n: errors_deg(FITTED if n.startswith("fitted") else bundled_path(n.split()[1]), logs)[1]
        for n in compare}
real_q = np.array([e["position"] for e in logs.make_batch()["entries"]], dtype=float)
picks = []
for traj in ["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down"]:
    idx = [i for i, l in enumerate(logs.logs) if l.get("trajectory") == traj]
    if idx:  # heaviest setup for that movement
        picks.append(max(idx, key=lambda i: logs.logs[i]["mass"] * logs.logs[i]["length"]))
picks = picks or list(range(min(4, len(logs.logs))))
dt = logs.logs[0]["dt"]
fig, axes = plt.subplots(len(picks), 1, figsize=(9, 2.3 * len(picks)), sharex=True)
axes = np.atleast_1d(axes)
for ax, i in zip(axes, picks):
    log = logs.logs[i]
    t = np.arange(real_q.shape[0]) * dt
    ax.plot(t, np.degrees(real_q[:, i]), color=COLORS["recorded"], lw=6, solid_capstyle="round", label="recorded data")
    for n in compare:
        ax.plot(t, np.degrees(sims[n][:, i]), color=color_for(n), lw=1.6,
                ls="--" if n.startswith("bundled") else "-", label=f"{n} sim")
    ax.set_title(f"{log.get('trajectory', '')}  |  {log['mass']*1000:.0f} g on {log['length']*100:.0f} cm arm, "
                 f"kp={log['kp']}", loc="left", fontsize=10)
    ax.set_ylabel("angle (deg)")
    ax.grid(color=GRID, linewidth=0.8)
    for s in ["top", "right"]:
        ax.spines[s].set_visible(False)
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False)
axes[-1].set_xlabel("time (s)")
fig.tight_layout(rect=(0, 0, 1, 1 - 0.35 / len(picks)))
fig.savefig(os.path.join(OUT, "example_recordings.png"), dpi=150)
plt.close(fig)

# Report
lines = ["# BAM pipeline report", "",
         f"- Recordings: **{len(logs.logs)}** from `{args.raw}/`",
         f"- P-gains: {kps}", ""]
lines += ["## How well each model matches", "", "| Model | Average error | Worst recording |", "|---|---|---|"]
for n in names:
    lines.append(f"| {n} | {results[n].mean():.2f}° | {results[n].max():.2f}° |")
lines.append("")

b = results.get(f"bundled {args.model}")
f_err = results.get(f"fitted {args.model}")
lines.append("## Verdict")
lines.append("")
best = min(names, key=lambda n: results[n].mean())
best_bundled = min([n for n in names if n.startswith("bundled")], key=lambda n: results[n].mean(), default=None)
if best_bundled is not None:
    bb = results[best_bundled].mean()
    lines.append(f"- BAM's ready-made model ({best_bundled}) is off by {bb:.2f}° on average: "
                 + ("close, so these servos behave like the BAM authors' servos." if bb < 1.0 else
                    "these servos behave noticeably differently from the BAM authors' servos."))
if f_err is not None:
    lines.append(f"- The fitted {args.model} model is off by {f_err.mean():.2f}° on average.")
if best.startswith("fitted"):
    lines.append(f"- **Use the fitted model:** load `{FITTED}` in the simulation.")
else:
    lines.append(f"- **Use BAM's ready-made {best.split()[1]} model**: it matches best here, so you can keep "
                 f"`load_model(motor_name=\"{args.bundled}\", model=\"{best.split()[1]}\")`.")
    if f_err is not None and best.split()[1] != args.model:
        lines.append(f"  - A more detailed model beat the fitted {args.model}. To fit that one, rerun with "
                     f"`--model {best.split()[1]} --trials 20000` (slower).")
if f_err is not None:
    if val_kp:
        tr, te = f_err[~held].mean(), f_err[held].mean()
        lines.append(f"- On recordings it trained on: {tr:.2f}°. On recordings it never saw (kp={val_kp}): {te:.2f}°.")
        lines.append("  - These are close, so the model learned the servo, not just the recordings."
                     if te < 1.5 * tr + 0.2 else
                     "  - It does noticeably worse on unseen recordings, so it may be overfitting. "
                     "Try more recordings or a simpler model (m1).")
lines.append("")

if args.truth and f_err is not None:
    truth = json.load(open(args.truth))
    fit = json.load(open(FITTED))
    lines += ["## Did the fit find the true settings? (fake data only)", "",
              "| Setting | True value | Fitted value | Off by |", "|---|---|---|---|"]
    for k, v in truth.items():
        if isinstance(v, (int, float)) and k in fit:
            off = abs(fit[k] - v) / abs(v) * 100 if v else float("nan")
            lines.append(f"| {k} | {v:.4f} | {fit[k]:.4f} | {off:.0f}% |")
    lines += ["", "Settings that are off by a lot barely change how the servo moves in these tests, "
              "so the recordings can't pin them down. That's normal; what matters is the error above.", ""]

lines += ["## Files", "",
          "- `error_comparison.png` - average error for each model",
          "- `example_recordings.png` - recorded motion vs. simulation for a few recordings",
          "- `errors_per_recording.csv` - error for every recording (opens in Excel/Sheets)"]
if f_err is not None:
    lines += [f"- `{os.path.basename(FITTED)}` - the fitted servo model. Use it with "
              f"`load_model(\"{FITTED}\")` instead of `load_model(motor_name=..., model=...)`."]
with open(os.path.join(OUT, "report.md"), "w") as fh:
    fh.write("\n".join(lines) + "\n")

print("\n".join(lines[lines.index("## Verdict"):lines.index("## Files")]))
print(f"Everything saved in {OUT}/  (start with report.md)")
