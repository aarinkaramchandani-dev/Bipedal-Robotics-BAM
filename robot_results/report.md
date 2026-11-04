# Robot vs. simulation report

- Recording: `fake_robot_recording.csv` (998 readings, 10.0 s)
- Robot file: `fake_robot.xml`
- Joints compared: shoulder, elbow, wrist
- Servo settings: kp=32, 7.4 V

## Average error per joint (degrees)

| Joint | bundled m1 | bundled m6 | fake_robot_truth |
|---|---|---|---|
| shoulder | 0.40 | 0.28 | 0.10 |
| elbow | 0.24 | 0.13 | 0.09 |
| wrist | 0.64 | 0.20 | 0.09 |
| **all joints** | **0.43** | **0.21** | **0.09** |

## Verdict

- **fake_robot_truth** matches the real robot best (average 0.09° across joints).
- The hardest joint to match is **shoulder** (0.10° average, 0.42° at worst).

## Files

- `joints_real_vs_sim.png` - each joint: real robot vs. simulation
- `error_per_joint.png` - average error for each joint and model
- `errors_per_joint.csv` - the numbers (opens in Excel/Sheets)
