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
