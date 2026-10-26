# BAM pipeline report

- Recordings: **12** from `data_raw/`
- P-gains: [8, 16, 32]

## How well each model matches

| Model | Average error | Worst recording |
|---|---|---|
| bundled m1 | 1.70° | 4.18° |
| bundled m6 | 0.42° | 0.77° |
| fitted m1 | 0.64° | 2.79° |

## Verdict

- BAM's ready-made model (bundled m6) is off by 0.42° on average: close, so these servos behave like the BAM authors' servos.
- The fitted m1 model is off by 0.64° on average.
- **Use BAM's ready-made m6 model**: it matches best here, so you can keep `load_model(motor_name="feetech_sts3215_7_4V", model="m6")`.
  - A more detailed model beat the fitted m1. To fit that one, rerun with `--model m6 --trials 20000` (slower).
- On recordings it trained on: 0.72°. On recordings it never saw (kp=16): 0.47°.
  - These are close, so the model learned the servo, not just the recordings.

## Files

- `error_comparison.png` - average error for each model
- `example_recordings.png` - recorded motion vs. simulation for a few recordings
- `errors_per_recording.csv` - error for every recording (opens in Excel/Sheets)
- `fitted_m1.json` - the fitted servo model. Use it with `load_model("results/fitted_m1.json")` instead of `load_model(motor_name=..., model=...)`.
