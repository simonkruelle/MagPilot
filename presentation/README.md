# MagPilot - three-minute keynote

An eight-slide presentation built around one vivid problem: robot arms can
assist people, but their usual controllers can exclude them.

MagPilot explores whether a passive magnet, mounted wherever reliable movement
remains, could provide direct access to a robot arm. The sensing and real-robot
control are working today. The assistive mount is a design direction that still
needs to be co-designed and validated with users.

## Files

| File | Purpose |
|---|---|
| `MagPilot_Keynote.pptx` | 8-slide, 16:9 deck with a concise speaker script and two auto-playing GIFs |
| `build_keynote.py` | Rebuilds the presentation from the repository assets |
| `assets/assistive_concept.png` | AI-generated, clearly labeled concept visualization |
| `assets/demo_classify_slide.gif` | 39-second classified-command proof |
| `assets/demo_teleop_slide.gif` | 28-second MagPilot package-pick proof |
| `../docs/action_mapping.png` | Verified configurable action editor revealed on slide 7 |

Rebuild the deck from the repository root:

```bash
python3 presentation/build_keynote.py
```

## Running order

The complete talk is exactly 3 minutes, including 67 seconds of auto-playing
demonstrations. Presenter View contains a short script paced to each slide.

| Time | Slide | Point |
|---|---|---|
| 0:00 | MagPilot | One passive magnet can become the interface |
| 0:14 | Problem + direction | Put the input where reliable movement remains |
| 0:38 | Two operating modes | Classified tasks and MagPilot Teleoperation |
| 1:02 | Proof 1 | Auto-play 39 seconds of letter/digit classification |
| 1:41 | Proof 2 | Auto-play 28 seconds of MagPilot Teleoperation |
| 2:09 | Reusable platform | Trackpad, Gazebo, and real FR3 share one workflow |
| 2:29 | Click reveal | The customization screen drops into the same platform |
| 2:44 | Where this could go | One user, one task, one proof, then repeat |

Both demonstrations are embedded as animated GIFs. They begin automatically
when their slide appears, without a play-button click. Let each clip run and
avoid speaking over it unless the room needs a brief explanation.

Slides 6 and 7 form one click-built scene. Advance once after explaining the
verification pipeline; the customizable action screen arrives with a downward
wipe.

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
been tested as an assistive device. Slide 2 labels its AI-generated image as a
concept visualization. The customization reveal states that new domain actions
still require engineering and validation. The final slide pitches the next
step without pretending the seminar prototype is already a company or product.
