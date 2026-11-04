# Progress log

A week-by-week record of the project (Georgia Tech VIP, Fall 2026).

## Week 1: Getting started (Sep 8)

- Read the BAM friction-model paper ([arXiv:2410.08650](https://arxiv.org/abs/2410.08650)).
- Read through the [BAM code](https://github.com/Rhoban/bam) to see how it is put together.
- Next: move from reading to building.

## Week 2: Learning how BAM works (Sep 14)

- Learned how BAM is organized: saved servo models, one folder per motor brand, and the link to the MuJoCo simulator.
- Confirmed BAM already has a model of our servo, the Feetech STS3215 (7.4 V).
- Learned BAM's three steps for a new motor: build a test rig, record the motor, fit a model.
- Learned the six friction models, from simplest (m1) to most detailed (m6).

## Week 3: First simulation (Sep 21 and 23)

- Set up BAM and ran a simulation: one STS3215 lifting a 100 g weight on a 15 cm arm, moving between target angles.
- Added a live viewer to watch it move. The servo acted like a real one: it took time to reach each target, overshot a little, and sagged under the weight.

## Week 4: Planning the test rig (Oct 4 and 5)

- Looked at how the BAM authors built their test rig (arm lengths, weights, mounting).
- Wrote a script that tries many arm-length and weight combinations on the simulated STS3215.
- Problems: the pip version of BAM was missing the STS3215 models (had to install from GitHub), BAM needs a specific Python version, and the first chart was hard to read.

## Week 5: Test-rig results (Oct 11)

- Fixed the chart so good and bad setups are easy to tell apart.
- Tested 35 combinations: arms 5 to 25 cm, weights 50 g to 1 kg.
- The STS3215 handled almost everything. It only struggled with 1 kg on a 25 cm arm (9.6° of sag).
- Recommended rig: 10 to 20 cm arms with 100 to 750 g weights.

## Week 6: One-command pipeline (Oct 20)

- Built `bam_pipeline.py`: one command checks BAM's model against recordings, fits a new model, and makes charts and a report.
- Tested it on fake recordings where the true settings were known. The fitted model cut the error from 1.9° to 0.1° and found the hidden settings within 1 to 2%.

## Week 7: Recording script (Oct 24 and 25)

- Wrote `record_sts3215.py` to replace BAM's Feetech script. BAM's version didn't work on a Mac, used the wrong voltage (15 V instead of 7.4 V), and left out the arm's weight.
- Added safety stops (swinging too far, overheating, low battery) and a calibration step so 0° means "hanging straight down".
- Added a practice mode that uses the simulation instead of a real servo.
- Made 12 practice recordings and ran them through the pipeline. It picked the right model (m6, 0.42° error).

## Week 8: Whole-robot comparison (Nov 2 and 4)

- Built `robot_compare.py`: it gives the simulated robot (BAM on every joint) the same commands the real robot got, then shows how far off each joint is.
- Made a fake 3-joint arm (shoulder, elbow, wrist) to test it, since our robot isn't built yet.
- BAM's detailed model (m6) matched about twice as well as the simple one (0.21° vs. 0.43°).
- Added checks for common setup mistakes (wrong units, mismatched joint names, wrong zero positions). When the elbow was set 4° off on purpose, it caught it.
