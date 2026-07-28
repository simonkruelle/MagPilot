# MagPilot - three-minute keynote

An eight-part presentation delivered as ten remote-controlled frames. It is
built around one vivid problem: robot arms can assist people, but their usual
controllers can exclude them.

MagPilot explores whether a passive magnet, mounted wherever reliable movement
remains, could provide direct access to a robot arm. The sensing and real-robot
control are working today. The assistive mount is a design direction that still
needs to be co-designed and validated with users.

## Files

| File | Purpose |
|---|---|
| `MagPilot_Keynote.pptx` | 10-frame, 16:9 deck with a concise speaker script and two auto-playing GIFs |
| `build_keynote.py` | Generates the editable presentation base from repository assets |
| `assets/assistive_concept.png` | AI-generated, clearly labeled concept visualization |
| `assets/demo_classify_slide.gif` | 39-second classified-command proof |
| `assets/demo_teleop_slide.gif` | 28-second MagPilot package-pick proof |
| `../docs/action_mapping.png` | Verified configurable action editor revealed on slide 7 |

Rebuild the deck from the repository root:

```bash
python3 presentation/build_keynote.py
```

The checked-in PowerPoint is the final hand-tuned presentation. Rebuilding it
regenerates the scripted base and does not preserve later manual layout tweaks.

## Running order

The complete talk is exactly 3 minutes, including 67 seconds of auto-playing
demonstrations. Presenter View contains a short script paced to each slide.

| Time | Slide | Point |
|---|---|---|
| 0:00 | MagPilot | Introduce the product and hold up the magnet |
| 0:08 | Problem | Robot arms can assist while controllers exclude |
| 0:20 | Magnet reveal | Placement, position, tilt, and zero-power benefits |
| 0:38 | Mode 1 | Classified commands |
| 0:50 | Mode 2 | MagPilot Teleoperation |
| 1:02 | Proof 1 | Auto-play 39 seconds of letter/digit classification |
| 1:41 | Proof 2 | Auto-play 28 seconds of MagPilot Teleoperation |
| 2:09 | Customer journey | Try in simulation, add sensing, then deploy |
| 2:29 | Click reveal | The customization screen drops into the same platform |
| 2:44 | Where this could go | One user, one task, one proof, then repeat |

Both demonstrations are embedded as animated GIFs. They begin automatically
when their slide appears, without a play-button click. Let each clip run and
avoid speaking over it unless the room needs a brief explanation.

## Logitech remote flow

Only the remote's forward and back buttons are needed. The visual builds use
consecutive PowerPoint frames rather than fragile object animations:

1. The first click on the concept scene reveals where the magnet can be placed
   and the four things it does not need.
2. The next pair of clicks highlights classified commands and then MagPilot
   Teleoperation.
3. After the customer journey, one click drops the customizable action screen
   into the same interface.

The two demonstrations start automatically when their frame opens. Avoid a
double-click when entering either demo.

## Demo media

The slide GIFs are cut from the full masked 1080p demonstrations. They use
monochrome 960x540 output at 20 fps with a 64-tone optimized palette. This keeps
the motion smooth and the robot readable while keeping the PowerPoint below
GitHub's 100 MB per-file limit.

```bash
ffmpeg -ss 33 -i docs/demo_classify.mp4 -t 39 \
  -filter_complex \
  "fps=20,scale=960:540:flags=lanczos,hue=s=0,split[base][palette_input];\
  [palette_input]palettegen=max_colors=64:stats_mode=diff[palette];\
  [base][palette]paletteuse=dither=sierra2_4a:diff_mode=rectangle" \
  -loop 0 presentation/assets/demo_classify_slide.gif

ffmpeg -ss 38 -i docs/demo_teleop.mp4 -t 28 \
  -filter_complex \
  "fps=20,scale=960:540:flags=lanczos,hue=s=0,split[base][palette_input];\
  [palette_input]palettegen=max_colors=64:stats_mode=diff[palette];\
  [base][palette]paletteuse=dither=sierra2_4a:diff_mode=rectangle" \
  -loop 0 presentation/assets/demo_teleop_slide.gif
```

The full recordings remain linked from the project README for anyone who wants
to inspect the complete demonstrations.

## Delivery

Open the deck in PowerPoint and test both animations before presenting. Animated
GIFs start when a slide opens and restart when that slide is entered again.
Keynote and LibreOffice Impress can import the file, but playback should still
be checked on the exact presentation computer.

Keep the assistive claim precise: MagPilot currently demonstrates the sensing,
configurable action mapping, and real-robot control mechanism. It has not yet
been tested as an assistive device. The assistive frames label their
AI-generated image as a concept visualization. The customization reveal states
that new domain actions still require engineering and validation. The final
slide pitches the next step without pretending the seminar prototype is
already a company or product.
