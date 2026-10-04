"""
Test rig sizing for the STS3215 using the BAM simulation.

For every combination of arm length and tip weight, the simulated STS3215
is told to lift the arm from hanging (0 deg) to flat (90 deg) and hold it.
We measure how well it does, to help choose arm lengths and weights for
the real pendulum test rig.

Run from the BAM folder:
    uv run python rig_sweep_sts3215.py

Outputs:
    rig_sweep_results.csv   one row per combination
    rig_sweep_heatmap.png   sag at 90 deg for every combination
"""

import csv
import numpy as np
import mujoco
from bam.model import load_model
from bam.mujoco import MujocoController

MOTOR = "feetech_sts3215_7_4V"
FRICTION_MODEL = "m6"
ARM_LENGTHS_CM = [5, 10, 15, 20, 25]
TIP_MASSES_G = [50, 100, 200, 300, 500, 750, 1000]
ARM_MASS_G = 20          # the arm itself (3D printed / wood)
TARGET_DEG = 90          # flat: the hardest pose, gravity pulls the most
SIM_TIME = 3.0           # seconds

# Ratings for the summary
GOOD_SAG_DEG = 3         # holds within 3 deg of target
OK_SAG_DEG = 10          # noticeable sag but still usable


def make_xml(length_m, tip_kg, arm_kg):
    return f"""
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="servo" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -{length_m}" size="0.005" mass="{arm_kg}"/>
      <geom type="sphere" pos="0 0 -{length_m}" size="0.01" mass="{tip_kg}"/>
    </body>
  </worldbody>
  <actuator><motor name="servo" joint="servo" gear="1"/></actuator>
</mujoco>"""


def run_one(model, length_cm, mass_g):
    length = length_cm / 100
    mj_model = mujoco.MjModel.from_xml_string(make_xml(length, mass_g / 1000, ARM_MASS_G / 1000))
    mj_data = mujoco.MjData(mj_model)
    ctrl = MujocoController(model, "servo", mj_model, mj_data)

    target = np.radians(TARGET_DEG)
    times, angles = [], []
    while mj_data.time < SIM_TIME:
        ctrl.set_q_target("servo", target)
        ctrl.update()
        mujoco.mj_step(mj_model, mj_data)
        times.append(mj_data.time)
        angles.append(np.degrees(mj_data.qpos[0]))
    times, angles = np.array(times), np.array(angles)

    final = angles[times > SIM_TIME - 0.5].mean()
    sag = TARGET_DEG - final
    peak = angles.max()
    overshoot = max(0.0, peak - TARGET_DEG)
    # time to first reach 90% of the target angle
    reached = np.where(angles >= 0.9 * TARGET_DEG)[0]
    rise = times[reached[0]] if len(reached) else float("nan")

    # gravity torque at flat: tip weight + arm weight (arm acts at its middle)
    torque_nm = 9.81 * (mass_g / 1000 * length + ARM_MASS_G / 1000 * length / 2)

    if np.isnan(rise) or sag > OK_SAG_DEG:
        rating = "too heavy"
    elif sag > GOOD_SAG_DEG:
        rating = "ok"
    else:
        rating = "good"

    return dict(
        arm_length_cm=length_cm, tip_mass_g=mass_g,
        torque_at_flat_Nm=round(torque_nm, 3),
        torque_at_flat_kgcm=round(torque_nm / 9.81 * 100, 1),
        rise_time_s=round(rise, 3) if not np.isnan(rise) else "never",
        overshoot_deg=round(overshoot, 2),
        final_angle_deg=round(final, 2),
        sag_deg=round(sag, 2),
        rating=rating,
    )


def main():
    model = load_model(motor_name=MOTOR, model=FRICTION_MODEL)
    rows = []
    for L in ARM_LENGTHS_CM:
        for m in TIP_MASSES_G:
            r = run_one(model, L, m)
            rows.append(r)
            print(f"{L:3d} cm, {m:5d} g | torque {r['torque_at_flat_kgcm']:5.1f} kg*cm | "
                  f"rise {r['rise_time_s']!s:>6} s | overshoot {r['overshoot_deg']:5.2f} | "
                  f"sag {r['sag_deg']:6.2f} deg | {r['rating']}")

    with open("rig_sweep_results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    print("Saved rig_sweep_results.csv")

    try:
        import matplotlib.pyplot as plt
        sag = np.array([[next(r["sag_deg"] for r in rows
                              if r["arm_length_cm"] == L and r["tip_mass_g"] == m)
                         for m in TIP_MASSES_G] for L in ARM_LENGTHS_CM])
        fig, ax = plt.subplots(figsize=(8, 4.5))
        im = ax.imshow(np.clip(sag, 0, 10), cmap="YlOrRd", vmin=0, vmax=10, aspect="auto")
        ax.set_xticks(range(len(TIP_MASSES_G)), [f"{m} g" for m in TIP_MASSES_G])
        ax.set_yticks(range(len(ARM_LENGTHS_CM)), [f"{L} cm" for L in ARM_LENGTHS_CM])
        for i in range(len(ARM_LENGTHS_CM)):
            for j in range(len(TIP_MASSES_G)):
                ax.text(j, i, f"{sag[i, j]:.1f}", ha="center", va="center", fontsize=9,
                        color="white" if sag[i, j] > 6 else "black")
        ax.set_xlabel("Weight on end of arm")
        ax.set_ylabel("Arm length")
        ax.set_title("STS3215 (sim): sag below target when holding arm flat [deg]")
        fig.colorbar(im, ax=ax, label="sag [deg] (capped at 10)")
        fig.tight_layout()
        fig.savefig("rig_sweep_heatmap.png", dpi=130)
        print("Saved rig_sweep_heatmap.png")
    except ImportError:
        print("matplotlib not installed, skipping the heatmap")


if __name__ == "__main__":
    main()
