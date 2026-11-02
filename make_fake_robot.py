"""
Make a fake robot + a fake "real robot" recording, to practice comparing a
whole robot against its simulation before the real robot exists.

Creates:
  fake_robot.xml            a 3-joint arm (shoulder, elbow, wrist), one STS3215 per joint
  fake_robot_recording.csv  what a real robot recording would look like:
                            time, and for each joint the commanded and actual angle (degrees)
  fake_robot_truth.json     the servo settings the "real" robot secretly used (answer key)

Run from the BAM folder:
    uv run python make_fake_robot.py
Then:
    uv run python robot_compare.py --robot fake_robot.xml --recording fake_robot_recording.csv
"""

import csv
import json

import mujoco
import numpy as np

from bam.model import load_model, load_model_from_dict
from bam.mujoco import MujocoController

JOINTS = ["shoulder", "elbow", "wrist"]

# A small arm hanging from a fixed base. Each joint has an STS3215 (55 g) on it.
XML = """
<mujoco model="fake_arm">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 -1 1.5" dir="0 1 -1"/>
    <geom type="box" pos="0 0 0.62" size="0.04 0.04 0.02" rgba="0.2 0.2 0.2 1"/>
    <body name="upper_arm" pos="0 0 0.6">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.12" size="0.012" mass="0.06" rgba="0.2 0.4 0.9 1"/>
      <geom type="box" pos="0 0 -0.12" size="0.012 0.02 0.018" mass="0.055" rgba="0.1 0.1 0.1 1"/>
      <body name="forearm" pos="0 0 -0.12">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.10" size="0.010" mass="0.05" rgba="0.2 0.4 0.9 1"/>
        <geom type="box" pos="0 0 -0.10" size="0.012 0.02 0.018" mass="0.055" rgba="0.1 0.1 0.1 1"/>
        <body name="hand" pos="0 0 -0.10">
          <joint name="wrist" type="hinge" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.06" size="0.008" mass="0.03" rgba="0.2 0.4 0.9 1"/>
          <geom type="sphere" pos="0 0 -0.07" size="0.02" mass="0.08" rgba="0.9 0.3 0.2 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder" joint="shoulder" gear="1"/>
    <motor name="elbow" joint="elbow" gear="1"/>
    <motor name="wrist" joint="wrist" gear="1"/>
  </actuator>
</mujoco>
"""


def commands(t):
    """A 10 s routine: lift the arm out, wave, then come back down (radians)."""
    ramp = np.clip(t / 2.0, 0, 1)                    # ease in over 2 s
    ramp = ramp * ramp * (3 - 2 * ramp)
    back = np.clip((10.0 - t) / 2.0, 0, 1)           # ease out in the last 2 s
    back = back * back * (3 - 2 * back)
    s = ramp * back
    shoulder = s * (np.radians(60) + np.radians(20) * np.sin(2 * np.pi * 0.4 * t))
    elbow = s * (np.radians(45) + np.radians(30) * np.sin(2 * np.pi * 0.7 * t + 1.0))
    wrist = s * np.radians(35) * np.sin(2 * np.pi * 1.1 * t)
    return np.array([shoulder, elbow, wrist])


def main():
    rng = np.random.default_rng(1)

    # The "real" robot's servos: BAM's STS3215 model, but a bit different
    base = load_model(motor_name="feetech_sts3215_7_4V", model="m6")
    truth = {k: p.value for k, p in base.get_parameters().items()}
    for k in truth:
        if "friction" in k:
            truth[k] *= 1.4          # 40% more friction (wear, tighter gears ...)
    truth["kt"] *= 0.92              # slightly weaker motors
    truth["model"], truth["actuator"] = "m6", "sts3215"
    model = load_model_from_dict(dict(truth))
    model.actuator.kp, model.actuator.vin = 32, 7.4

    with open("fake_robot.xml", "w") as f:
        f.write(XML)
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    ctrl = MujocoController(model, JOINTS, m, d)

    duration = 10.0
    ts, qs = [], []
    while d.time < duration:
        for name, q in zip(JOINTS, commands(d.time)):
            ctrl.set_q_target(name, q)
        ctrl.update()
        mujoco.mj_step(m, d)
        ts.append(d.time)
        qs.append(d.qpos[:3].copy())
    ts, qs = np.array(ts), np.array(qs)

    # "Read the servos" every ~10 ms (3 servos on one bus), with noise and encoder steps
    step = 2 * np.pi / 4096
    rows, t = [], 0.0
    while t < duration - 0.01:
        actual = [np.interp(t, ts, qs[:, j]) + rng.normal(0, 0.002) for j in range(3)]
        actual = np.round(np.array(actual) / step) * step
        goal = commands(t)
        row = {"time": round(t, 5)}
        for j, name in enumerate(JOINTS):
            row[f"{name}_goal"] = round(float(np.degrees(goal[j])), 4)
            row[f"{name}_actual"] = round(float(np.degrees(actual[j])), 4)
        rows.append(row)
        t += 0.010 + rng.uniform(-0.002, 0.002)

    with open("fake_robot_recording.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    with open("fake_robot_truth.json", "w") as f:
        json.dump(truth, f, indent=2)

    print("Made fake_robot.xml (3-joint arm)")
    print(f"Made fake_robot_recording.csv ({len(rows)} readings over {duration:.0f} s)")
    print("Saved the answer key to fake_robot_truth.json")


if __name__ == "__main__":
    main()
