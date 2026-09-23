"""
Watch a simulated STS3215 servo move a weighted arm, live in the MuJoCo viewer.

Solid arm   = the simulated servo (using BAM's STS3215 model)
Ghost arm   = the target the servo is trying to reach

Run from the BAM folder:
    Windows / Linux:  uv run python test_sts3215_live.py
    Mac:              uv run mjpython test_sts3215_live.py

Close the viewer window to stop.
"""

import sys
import time
import numpy as np
import mujoco
from bam.model import load_model
from bam.mujoco import MujocoController

# ---- Things you can change -------------------------------------------------
MOTOR = "feetech_sts3215_7_4V"
FRICTION_MODEL = "m6"   # try "m1" to compare with a basic friction model
ARM_LENGTH = 0.15       # meters
TIP_MASS = 0.5         # kg (0.1 = 100 g)
# Target angles in degrees (0 = hanging down, 90 = flat). Each is held 1.5 s.
TARGETS_DEG = [0, 90, 45, -60, 0]
HOLD_TIME = 1.5         # seconds per target
# ---------------------------------------------------------------------------

XML = f"""
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual><global azimuth="90" elevation="-10"/></visual>
  <worldbody>
    <light pos="0 -1 1.5" dir="0 1 -1"/>
    <geom type="plane" size="0.5 0.5 0.01" rgba="0.9 0.9 0.9 1"/>
    <!-- servo body (just for looks) -->
    <geom type="box" pos="0 0.03 0.5" size="0.02 0.02 0.012" rgba="0.1 0.1 0.1 1"/>

    <!-- the real (simulated) arm -->
    <body name="arm" pos="0 0 0.5">
      <joint name="servo" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -{ARM_LENGTH}" size="0.008"
            mass="0.02" rgba="0.2 0.4 0.9 1"/>
      <geom type="sphere" pos="0 0 -{ARM_LENGTH}" size="0.02"
            mass="{TIP_MASS}" rgba="0.9 0.3 0.2 1"/>
    </body>

    <!-- ghost arm showing the target (no physics) -->
    <body name="target" mocap="true" pos="0 0 0.5">
      <geom type="capsule" fromto="0 0 0 0 0 -{ARM_LENGTH}" size="0.009"
            contype="0" conaffinity="0" rgba="0.2 0.9 0.3 0.3"/>
      <geom type="sphere" pos="0 0 -{ARM_LENGTH}" size="0.021"
            contype="0" conaffinity="0" rgba="0.2 0.9 0.3 0.3"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="servo" joint="servo" gear="1"/>
  </actuator>
</mujoco>
"""


def target_at(t):
    """Cycle through TARGETS_DEG forever. Returns radians."""
    i = int(t // HOLD_TIME) % len(TARGETS_DEG)
    return np.radians(TARGETS_DEG[i])


def setup():
    model = load_model(motor_name=MOTOR, model=FRICTION_MODEL)
    mj_model = mujoco.MjModel.from_xml_string(XML)
    mj_data = mujoco.MjData(mj_model)
    controller = MujocoController(model, "servo", mj_model, mj_data)
    return mj_model, mj_data, controller


def step(mj_model, mj_data, controller):
    target = target_at(mj_data.time)
    controller.set_q_target("servo", target)
    controller.update()
    mujoco.mj_step(mj_model, mj_data)
    # rotate the ghost arm to the target angle (rotation about y axis)
    mj_data.mocap_quat[0] = [np.cos(target / 2), 0, np.sin(target / 2), 0]
    return target


def main():
    import mujoco.viewer

    mj_model, mj_data, controller = setup()
    last_print = -1.0

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        start = time.time()
        while viewer.is_running():
            # step the physics until it catches up with real time
            while mj_data.time < time.time() - start:
                target = step(mj_model, mj_data, controller)
            viewer.sync()

            if mj_data.time - last_print > 0.25:
                last_print = mj_data.time
                print(f"t={mj_data.time:5.2f}s  target={np.degrees(target):6.1f} deg  "
                      f"actual={np.degrees(mj_data.qpos[0]):6.1f} deg")
            time.sleep(0.005)


if __name__ == "__main__":
    if "--headless" in sys.argv:  # quick check without a window
        m, d, c = setup()
        while d.time < 3.0:
            step(m, d, c)
        print(f"OK: arm at {np.degrees(d.qpos[0]):.1f} deg after 3 s")
    else:
        main()
