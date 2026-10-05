# MagPilot — Week 1 decisions and project plan

Meeting: 5 October 2026. Time zone: Europe/Berlin. Branch: `paper`.

**Meeting outcome:** The sensor board is repaired and three smaller magnets are
available for one-, two- and three-magnet comparisons. Start the virtual
robot-arm target-reaching task immediately in **Week 1**: this is the main
implementation risk. Develop and pilot completion-time/error measurement while
hardware comparisons and character collection proceed in parallel.

Review: [Week 1 update](reports/MagPilot_Lab_Update_2026-10-05.md) ·
[Weekly reports](reports/README.md) · [Virtual task pilot](VIRTUAL_TASK_PILOT.md).

**3–5 minute launcher demo**

Launch `python3 colmag_launcher.py`, open **Data**, and rehearse the sequence once. Use a fictitious participant and a clearly named experiment such as `Lab meeting mouse demo`.

| Time | Show | Explain |
|---|---|---|
| 0:00–0:45 | Participant name with stable P ID; experiment name; digits/letters and repetition target. Save the form. | Protocol: digits 0–9 and letters A–J, ten takes each. Blank/still controls are optional and outside that target. |
| 0:45–1:15 | Select **Demo (mouse)** and start the participant session. | This exercises the recording software with synthetic input; it supplies no evidence of magnetic tracking accuracy. Keep demo counts separate from sensor counts. |
| 1:15–2:15 | In the recording window, select **A** or **3**, position the cursor, press **Enter**, move the mouse to draw one character, then press **Enter** again. | The experimenter selects the prompt and presses Enter; the participant only draws. Enter starts/stops both capture and mouse ink. Stop before repositioning; the saved take uses a frozen sample snapshot. |
| 2:15–3:00 | Show the character preview and updated demo count; open the matching CSV/PNG/JSON and manifest. | The raw rows, derived image, participant/session IDs, experiment, source and protocol travel together. The prompt defines the intended class; sample review will check that it was written correctly. |
| 3:00–4:00 | Press Enter for another take, draw briefly, press **Escape**; then record a successful repeat. | Cancellation adds no completed take. A successful repeat gets its own repetition and files. |
| 4:00–5:00 | Show the next-milestone list below. | Virtual task development starts now; check the repaired board and compare the three magnet configurations in parallel. |

The Data table refreshes periodically; use its refresh control if the newly saved demo take has not appeared yet. Rehearse the actual GUI before presenting it; automated recording tests do not verify desktop interaction.

**Current software versus the next milestones**

- Implemented: participant names/IDs, experiment metadata, shared character protocol, explicit capture boundaries, cancel/repeat, matched recording artifacts, frozen rows and save retry. Board diagnostics and a ground-truth tracking recorder also exist.
- Immediate priority: develop/pilot virtual robot-arm target reaching; log target and actual end-effector positions, completion time, endpoint error and unsuccessful trials. Use consistent targets and starting poses across control conditions.
- Parallel work: check the repaired board; compare one-, two- and three-magnet pen/cover configurations; record a small real-sensor character pilot; inspect timing, raw channel response, segmentation and file consistency.
- Before formal collection: freeze the pen/cover and protocol; add documented quality review and accepted/rejected takes, recoverable progress, reproducible preprocessing and participant-separated dataset splits.
- Later milestones: recognition-model comparison, measured tracking-error maps and formal participant evaluation of the already-piloted virtual task.

## Held-out evaluation

- The drawing prompt records ground truth; the classifier receives sensor/pose/image features. Keep the label, label-bearing filenames and label-dependent metadata out of those features.
- For recognition on new writers, reserve complete participants for the protected test set: every session, height and repetition from each reserved participant stays together. Do not split repetitions from one person across training and test. [scikit-learn: grouped cross-validation](https://scikit-learn.org/stable/modules/cross_validation.html#cross-validation-iterators-for-grouped-data).
- Use training data to fit learned preprocessing and models; use validation participants to choose models and thresholds. Freeze the complete pipeline before the final test. Test data must not influence those choices. [scikit-learn: data leakage](https://scikit-learn.org/stable/common_pitfalls.html#data-leakage).
- Freeze pen/board conditions, prompts and quality rules before the protected collection. Keep mouse demos and development-pilot participants outside the final test; do not repeat or reject test takes because of a model error.
- If human blinding is needed, hide the expected label from the evaluator until predictions are recorded. The participant still needs the drawing prompt.

Participant assignments and protected evaluation are planned; the launcher does not enforce a train/validation/test split yet. Agree on participant numbers before choosing split sizes.

**Compare the three smaller pen magnets**

The existing **11 mm magnet** makes the pen bulky. Three smaller magnets are now
available. Compare **one magnet**, **two magnets** and **three magnets** using
documented arrangements. Record dimensions/shape, material/grade, magnetization
direction, stack spacing/orientation and part identifiers where available.
Confirm what the existing 11 mm measurement refers to before comparing geometries.

At equal remanence/material grade, the idealized magnetic moment is `m = Br × V / μ0`: less magnetic volume gives a lower dipole moment. In the far field, a weaker moment reduces the signal at a fixed distance; bringing the magnet closer can compensate, but near the magnet its shape matters. This is the reason to compare the complete pen, cover gap and intended working range. [K&J Magnetics: magnetic dipole moment](https://www.kjmagnetics.com/blog/magnetic-dipole-moment).

Treat calculations as screening estimates. Field geometry depends on magnetization and the measurement position; K&J's calculator assumes axial magnetization for discs/cylinders/rings and recommends measurement in the actual configuration. [K&J Magnetics: field calculator and assumptions](https://www.kjmagnetics.com/magnetic-field-calculator.asp).

No smaller candidate is yet proven sufficient. The saved September probe, reanalyzed with probe v1.2, suggests channel flat tops during one sweep; this is a setup-specific observation, not a universal saturation threshold or evidence about an untested smaller pen.

**With the repaired board: hardware and character pilot**

1. Verify startup, all 16 sensors and stable valid packets after the repair. Close other sensor readers before running the board check.
2. Record a magnet-away baseline, then check controlled weak/strong exposure and return to baseline. Distinguish wiring/startup failures from offset or saturation observations.
3. Compare the existing magnet and the one-/two-/three-magnet configurations using the same writing surface and marked positions. Record magnet/pen ID, cover/spacer, magnet-center offset, stack orientation/spacing, raw channel response, noise, timing gaps and pose repeatability/error against known placements.
4. Separately test the larger volume needed for teleoperation. A comfortable writing pen may have insufficient range there; choose the shared or separate configuration from measurements.
5. Pilot a few characters with Simon, then one or two colleagues. Inspect the full artifacts and filtered image before freezing hardware and inviting the formal cohort.

**Ten-week workstreams and order of priority**

| Planned weeks | Priority and deliverable |
|---|---|
| **1–3** | **Virtual robot-arm target-reaching implementation and early pilot. Start in Week 1; establish targets, start pose, logging, completion rule and configurable control conditions.** |
| 1–2 | Hardware requirements/CAD/print iterations alongside reliable launcher recording; verify the repaired board and compare one-/two-/three-magnet configurations. |
| 3 | Real-sensor pilot; fix remaining acquisition issues; freeze pen/cover; finish the tracking ground-truth procedure. |
| 4–6 | Formal participant collection with fixed conditions; preprocessing and participant-based splits. Collect tracking references and begin baseline networks in parallel. |
| 5–7 | Compare models and tracking errors; refine the already-started virtual task in parallel. |
| 9 | Teleoperation participant evaluation and preliminary analysis. |
| 10 | Final recognition/tracking/teleoperation figures, documentation and paper/report; buffer for delays. |

These are relative project weeks. The virtual task now starts in **Week 1**, in
parallel with pen/recording work; it is no longer deferred until weeks 6–8.
Before formal evaluation, agree on participant numbers, comparison controllers,
target difficulty/tolerance, starting poses, trial order and timeout handling.
The initial task recorder is a development tool; no participant completion-time
results or controller-speed advantage have been established yet.
