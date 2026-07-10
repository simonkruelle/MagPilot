# COLMAG Pipeline Task List

Week 4 goal from the slides: make the air-writing and virtual-switch pipeline robust enough to behave like an interface, not just a visualization demo.

## Sequential Implementation Plan

- [x] Capture the week 4 requirements in a shared task list.
- [x] Start modularizing app-level behavior.
  - `magnetometer_reader.py` still owns serial input, recording, plotting, and OCR wiring.
  - `colmag/interaction.py` owns virtual joystick geometry, dwell detection, app modes, and command events.
  - Future ROS code should consume command events instead of depending on plotting or OCR internals.
- [x] Keep OCR from blocking the live interface.
  - EasyOCR runs in a background worker.
  - The Matplotlib app submits the latest OCR image after a writing pause by default.
  - Continuous OCR remains available with `--classifier-mode continuous`.
  - Joystick/cursor rendering stays on the UI path.
- [x] Add writing/hover filtering before OCR image generation.
  - Use XY velocity to reject stationary magnet samples so a resting magnet does not become a dot.
  - Use an optional maximum velocity so fast repositioning motions can become "pen up" during air-writing.
  - Keep Z closeness as an optional board/contact-mode gate, disabled by default for air-writing.
  - Show the number of active ink samples in the live visualization.
- [ ] Tune and document air-writing segmentation.
  - Record typical controlled-writing velocity ranges.
  - Record typical fast-reposition velocity ranges for multi-stroke letters such as A.
  - Decide when to use dwell/pause gestures versus fast repositioning for stroke breaks.
- [ ] Define the command vocabulary.
  - Test only a small subset of robust symbols instead of all 26 letters.
  - Prefer letters/numbers that work as single continuous air-written strokes.
  - Keep EasyOCR for OCR-like symbols and add a separate template recognizer for shapes such as stars.
- [ ] Add synthetic data save/replay.
  - Save raw sessions, filtered sessions, and generated OCR images with metadata.
  - Add a replay mode that feeds recorded/synthetic pose data through the same pipeline without serial hardware.
- [x] Implement the first virtual joystick/menu layer.
  - Define U/D/L/R/A/B/C/X regions in projected XY space.
  - Detect a switch press from dwell time, e.g. magnet inside a region for 2 seconds.
  - Holding L switches to letter detection; holding R switches to number detection.
  - A/B/C/X and U/D currently emit robot-command placeholders for the later ROS adapter.
- [ ] Refine the virtual joystick/menu behavior.
  - Decide the final button-to-command mapping.
  - Add confirm/cancel menus for important robot commands.
  - Decide whether writing starts after a mode press for a timed window or until another switch is pressed.
- [ ] Build a graphical product interface.
  - Replace the research plot layout with a main UI that shows cursor, virtual switches, mode, confidence, and last command.
  - Keep a diagnostics panel for raw sensor plots and trajectory debugging.
- [ ] Prepare ROS integration for the real robot.
  - Keep recognition output as clean command events.
  - Define a ROS-ready adapter boundary, but defer actual robot integration to next week.

## Current Priorities

1. Make the OCR image contain only intentional writing strokes.
2. Make multi-stroke air-written letters work without requiring a board-distance gesture.
3. Shortlist a small command alphabet that is robust in air-writing.
4. Keep every decision visible in the UI so threshold tuning is fast during lab testing.
