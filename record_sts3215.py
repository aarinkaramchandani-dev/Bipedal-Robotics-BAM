"""
Record a Feetech STS3215 on the pendulum test rig, in BAM's format.

This replaces BAM's own bam/feetech/record.py for our setup:
  - finds the servo's USB port on a Mac automatically
  - uses our 7.4 V servos (BAM's script defaulted to 15 V)
  - records the arm's own mass (BAM's Feetech script left it out)
  - measures where "hanging straight down" is, so 0 deg is correct
  - safety stops: angle limit, temperature limit, low voltage, Ctrl+C
  - runs every test movement at every gain in one go
  - PRACTICE MODE: uses the simulation instead of a real servo, so the
    whole process can be tested before the rig is built

Run from the BAM folder.

  Practice (no servo needed):
    uv run python record_sts3215.py --practice --mass 0.2 --length 0.15 --arm-mass 0.02

  Real servo, first time on a new rig (finds "straight down"):
    uv run python record_sts3215.py --calibrate

  Real servo, record one rig setup:
    uv run python record_sts3215.py --mass 0.2 --length 0.15 --arm-mass 0.02

Repeat the record command for each arm length / weight combo (change the
numbers to match what's on the rig). All recordings land in data_raw/,
ready for:  uv run python bam_pipeline.py --raw data_raw
"""

import argparse
import datetime
import glob
import json
import os
import sys
import time

import numpy as np

from bam.trajectory import trajectories

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--mass", type=float, help="weight on the end of the arm [kg], e.g. 0.2 for 200 g")
p.add_argument("--length", type=float, help="arm length from servo shaft to weight [m], e.g. 0.15")
p.add_argument("--arm-mass", type=float, help="mass of the arm itself [kg], e.g. 0.02")
p.add_argument("--kps", type=int, nargs="+", default=[8, 16, 32], help="firmware P-gains to record")
p.add_argument("--trajectories", nargs="+",
               default=["sin_time_square", "sin_sin", "lift_and_drop", "up_and_down"])
p.add_argument("--logdir", default="data_raw", help="where recordings are saved")
p.add_argument("--vin", type=float, default=7.4, help="supply voltage [V]")
p.add_argument("--port", default="auto", help="serial port (auto = find it)")
p.add_argument("--id", type=int, default=1, help="servo ID")
p.add_argument("--yes", action="store_true", help="don't pause between recordings")
p.add_argument("--calibrate", action="store_true", help="measure the 'hanging straight down' position")
p.add_argument("--practice", action="store_true", help="use the simulation instead of a real servo")
p.add_argument("--practice-params", default=None,
               help="servo model the practice servo uses (default: BAM's bundled STS3215 m6)")
args = p.parse_args()

# ---- Safety limits -------------------------------------------------------
MAX_ANGLE = np.radians(125)   # sin_sin asks for up to 101 deg; stop if the arm goes past 125
MAX_TEMP = 60                 # deg C: stop recording above this
COOL_TO = 45                  # deg C: wait until it cools to this before continuing
MIN_VOLTS = 0.85 * args.vin   # stop if the supply sags more than 15%
RETURN_SPEED = 1.0            # rad/s when gently returning the arm to 0
CALIBRATION_FILE = "sts3215_calibration.json"


class SafetyStop(Exception):
    pass


# =========================================================================
# The real servo
# =========================================================================
class RealServo:
    def __init__(self, port, servo_id):
        import rustypot
        if port == "auto":
            candidates = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
                                + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if not candidates:
                sys.exit("No servo board found. Is the USB cable plugged in? (Or pass --port yourself.)")
            port = candidates[0]
        print(f"Connecting to servo {servo_id} on {port} ...")
        self.c = rustypot.Sts3215PyController(serial_port=port, baudrate=1_000_000, timeout=0.1)
        self.id = servo_id
        if not self.c.ping(servo_id):
            sys.exit(f"No servo answered on ID {servo_id}. Check power, wiring and the ID.")
        self.offset = 0.0
        if os.path.exists(CALIBRATION_FILE):
            self.offset = json.load(open(CALIBRATION_FILE))["offset_rad"]

    @staticmethod
    def _one(value):
        # older rustypot versions return a list even for a single servo
        return float(value[0] if isinstance(value, (list, tuple)) else value)

    def setup(self, kp):
        c, i = self.c, self.id
        c.write_torque_enable(i, False)
        c.write_mode(i, 0)              # position mode
        c.write_p_coefficient(i, kp)
        c.write_i_coefficient(i, 0)
        c.write_d_coefficient(i, 0)     # BAM's model assumes a pure P controller
        c.write_acceleration(i, 0)      # no built-in smoothing ...
        c.write_goal_speed(i, 0.0)      # ... and no speed cap (0 = max)

    def torque(self, on):
        self.c.write_torque_enable(self.id, bool(on))

    def goal(self, q):
        self.c.write_goal_position(self.id, float(q + self.offset))

    def read(self):
        return {
            "position": self._one(self.c.read_present_position(self.id)) - self.offset,
            "speed": self._one(self.c.read_present_speed(self.id)),
            "load": 0.0,
            "input_volts": self._one(self.c.read_present_voltage(self.id)) * 0.1,
            "temp": self._one(self.c.read_present_temperature(self.id)),
        }

    def raw_position(self):
        return self._one(self.c.read_present_position(self.id))

    def now(self):
        return time.time()

    def wait(self, seconds):
        time.sleep(seconds)

    def close(self):
        try:
            self.torque(False)
        finally:
            self.c.close()


# =========================================================================
# The practice servo: same interface, but it's the BAM simulation
# =========================================================================
class PracticeServo:
    SUB_DT = 0.001     # simulate in 1 ms steps
    READ_TIME = 0.004  # pretend each sensor read takes ~4 ms, like the real bus

    def __init__(self, params_path):
        from bam.model import load_model
        from bam.simulate import Simulator
        self.model = load_model(params_path) if params_path else \
            load_model(motor_name="feetech_sts3215_7_4V", model="m6")
        self.sim = Simulator(self.model)
        self.rng = np.random.default_rng()
        self.t = 0.0
        self.temp = 30.0
        self.setup(args.kps[0])

    def setup(self, kp):
        log = {"mass": args.mass, "arm_mass": args.arm_mass, "length": args.length,
               "kp": kp, "vin": args.vin, "entries": []}
        self.model.actuator.load_log(log)
        self.sim.reset(0.0, 0.0)
        self.enabled, self.target = False, 0.0

    def torque(self, on):
        self.enabled = bool(on)

    def goal(self, q):
        self.target = float(q)

    def _advance(self, dt):
        steps = max(1, int(round(dt / self.SUB_DT)))
        for _ in range(steps):
            control = self.model.actuator.compute_control(self.target, self.sim.q, self.sim.dq, self.SUB_DT)
            self.sim.step(control, self.enabled, self.SUB_DT)
        self.t += steps * self.SUB_DT
        # pretend temperature: warms up while holding torque, cools toward room temp
        seconds = steps * self.SUB_DT
        self.temp += seconds * ((0.4 if self.enabled else 0.0) - (self.temp - 30.0) / 60.0)

    def read(self):
        self._advance(self.READ_TIME + self.rng.uniform(-0.0015, 0.0015))
        step = 2 * np.pi / 4096  # encoder resolution
        q = float(np.round((self.sim.q + self.rng.normal(0, 0.002)) / step) * step)
        return {"position": q, "speed": float(self.sim.dq + self.rng.normal(0, 0.05)), "load": 0.0,
                "input_volts": float(args.vin + self.rng.normal(0, 0.05)), "temp": self.temp}

    def raw_position(self):
        return float(self.sim.q)

    def now(self):
        return self.t

    def wait(self, seconds):
        self._advance(seconds)

    def close(self):
        pass


# =========================================================================
def check_safety(entry):
    if abs(entry["position"]) > MAX_ANGLE:
        raise SafetyStop(f"arm went to {np.degrees(entry['position']):.0f} deg (limit {np.degrees(MAX_ANGLE):.0f})")
    if entry["temp"] > MAX_TEMP:
        raise SafetyStop(f"servo is {entry['temp']:.0f} C (limit {MAX_TEMP} C)")
    if entry["input_volts"] < MIN_VOLTS:
        raise SafetyStop(f"supply dropped to {entry['input_volts']:.1f} V (need at least {MIN_VOLTS:.1f} V)")


def return_to_zero(servo):
    """Gently bring the arm back to hanging straight down, then relax."""
    q = servo.read()["position"]
    servo.torque(True)
    while abs(q) > 1e-3:
        q -= np.sign(q) * min(abs(q), RETURN_SPEED * 0.01)
        servo.goal(q)
        servo.wait(0.01)
    servo.goal(0.0)
    servo.wait(1.0)
    servo.torque(False)
    servo.wait(0.5)


def wait_until_cool(servo):
    temp = servo.read()["temp"]
    if temp > COOL_TO + 5:
        print(f"  servo is {temp:.0f} C, letting it cool to {COOL_TO} C ...")
        while servo.read()["temp"] > COOL_TO:
            servo.wait(5.0)


def record_one(servo, kp, traj_name):
    traj = trajectories[traj_name]
    servo.setup(kp)
    goal, enable = traj(0.0)
    servo.goal(goal)
    servo.torque(enable)
    servo.wait(0.5)  # settle at the start position

    entries = []
    start = servo.now()
    while servo.now() - start < traj.duration:
        t = servo.now() - start
        goal, new_enable = traj(t)
        if new_enable != enable:
            servo.torque(new_enable)
            enable = new_enable
        if enable:
            servo.goal(goal)
        t0 = servo.now() - start
        entry = servo.read()
        t1 = servo.now() - start
        entry["timestamp"] = (t0 + t1) / 2.0
        entry["goal_position"] = float(goal)
        entry["torque_enable"] = bool(enable)
        entries.append(entry)
        check_safety(entry)
    return entries


def calibrate(servo):
    print("\nCALIBRATION: find where 'hanging straight down' is.")
    servo.torque(False)
    input("Let the arm hang completely still (no weight swinging), then press Enter ...")
    samples = []
    for _ in range(50):
        samples.append(servo.raw_position())
        servo.wait(0.02)
    offset = float(np.median(samples))
    json.dump({"offset_rad": offset, "offset_deg": float(np.degrees(offset)),
               "date": datetime.datetime.now().isoformat()}, open(CALIBRATION_FILE, "w"), indent=2)
    print(f"Saved: straight down = {np.degrees(offset):.2f} deg on the servo -> {CALIBRATION_FILE}")
    print("Redo this whenever the arm is re-mounted on the servo horn.")


def main():
    servo = PracticeServo(args.practice_params) if args.practice else RealServo(args.port, args.id)
    try:
        if args.calibrate:
            if args.practice:
                sys.exit("Calibration is only for a real servo.")
            calibrate(servo)
            return

        missing = [n for n in ("mass", "length", "arm_mass") if getattr(args, n) is None]
        if missing:
            sys.exit("Please give " + ", ".join("--" + m.replace("_", "-") for m in missing))
        if not args.practice and not os.path.exists(CALIBRATION_FILE):
            sys.exit("Run with --calibrate first so 0 deg means 'hanging straight down'.")
        for name in args.trajectories:
            if name not in trajectories:
                sys.exit(f"Unknown movement '{name}'. Options: {', '.join(trajectories)}")

        os.makedirs(args.logdir, exist_ok=True)
        jobs = [(kp, tr) for kp in args.kps for tr in args.trajectories]
        mode = "PRACTICE (simulated servo)" if args.practice else "REAL servo"
        print(f"\n{mode}: {args.mass*1000:.0f} g on a {args.length*100:.0f} cm arm "
              f"({args.arm_mass*1000:.0f} g arm), {len(jobs)} recordings of 6 s each")

        for n, (kp, traj_name) in enumerate(jobs, 1):
            wait_until_cool(servo)
            label = f"[{n}/{len(jobs)}] kp={kp}, {traj_name}"
            if not args.yes and not args.practice:
                input(f"\n{label}: make sure the arm can swing freely, then press Enter ...")
            else:
                print(f"{label}")
            entries = record_one(servo, kp, traj_name)
            return_to_zero(servo)

            data = {"mass": args.mass, "arm_mass": args.arm_mass, "length": args.length,
                    "kp": kp, "vin": args.vin, "motor": "sts3215", "trajectory": traj_name,
                    "practice": args.practice, "entries": entries}
            stamp = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
            name = f"{stamp}_L{round(args.length*100)}cm_m{round(args.mass*1000)}g_kp{kp}_{traj_name}.json"
            with open(os.path.join(args.logdir, name), "w") as f:
                json.dump(data, f)
            peak = max(abs(e["position"]) for e in entries)
            print(f"    saved {name}  ({len(entries)} readings, peak {np.degrees(peak):.0f} deg)")

        print(f"\nDone. Recordings are in {args.logdir}/")
        print(f"Next: uv run python bam_pipeline.py --raw {args.logdir}")
    except SafetyStop as e:
        print(f"\nSAFETY STOP: {e}. Torque turned off. Nothing from this recording was saved.")
    except KeyboardInterrupt:
        print("\nStopped by you. Torque turned off.")
    finally:
        servo.close()


if __name__ == "__main__":
    main()
