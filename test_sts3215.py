"""
Test the BAM STS3215 model in MuJoCo with a simple pendulum.

One STS3215 holds a 15 cm arm with a 100 g weight at the tip. The servo is
asked to move to a few target angles, and we compare the default MuJoCo-style
friction model (m1) against BAM's richest model (m6).

Install first (the GitHub version has all six STS3215 models; the PyPI
release may only include m1):
    pip install "better-actuator-models[mujoco] @ git+https://github.com/Rhoban/bam.git" matplotlib
"""

import numpy as np
import mujoco
import matplotlib.pyplot as plt
from bam.model import load_model
from bam.mujoco import MujocoController

MOTOR = "feetech_sts3215_7_4V"
ARM_LENGTH = 0.15  # m
TIP_MASS = 0.1     # kg
SIM_TIME = 4.0     # s

# Angle 0 = arm hanging straight down. Pendulum swings around the y axis.
XML = f"""
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="arm" pos="0 0 0.5">
      <joint name="servo" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -{ARM_LENGTH}" size="0.008" mass="0.02"/>
      <geom type="sphere" pos="0 0 -{ARM_LENGTH}" size="0.02" mass="{TIP_MASS}"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="servo" joint="servo" gear="1"/>
  </actuator>
</mujoco>
"""

def target_at(t):
    """Step through a few target angles (radians)."""
    if t < 1.0:
        return 0.0
    if t < 2.0:
        return np.pi / 2   # lift to horizontal
    if t < 3.0:
        return np.pi / 4
    return 0.0

def run(model_name):
    model = load_model(motor_name=MOTOR, model=model_name)
    mj_model = mujoco.MjModel.from_xml_string(XML)
    mj_data = mujoco.MjData(mj_model)
    controller = MujocoController(model, "servo", mj_model, mj_data)

    times, targets, angles = [], [], []
    while mj_data.time < SIM_TIME:
        target = target_at(mj_data.time)
        controller.set_q_target("servo", target)
        controller.update()
        mujoco.mj_step(mj_model, mj_data)
        times.append(mj_data.time)
        targets.append(target)
        angles.append(mj_data.qpos[0])
    return np.array(times), np.array(targets), np.array(angles)

if __name__ == "__main__":
    plt.figure(figsize=(9, 5))
    for name in ["m1", "m6"]:
        t, target, q = run(name)
        err = np.degrees(np.abs(target - q))[t > 0.9][-50:].mean()
        print(f"{name}: final angle {np.degrees(q[-1]):6.2f} deg, "
              f"steady-state error at end ~{err:.2f} deg")
        plt.plot(t, np.degrees(q), label=f"BAM {name}")
    plt.plot(t, np.degrees(target), "k--", label="target")
    plt.xlabel("time [s]")
    plt.ylabel("angle [deg]")
    plt.title(f"STS3215 pendulum ({TIP_MASS*1000:.0f} g at {ARM_LENGTH*100:.0f} cm)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("sts3215_test.png", dpi=120)
    print("Saved plot to sts3215_test.png")
