# MagPilot - short keynote

An eight-slide presentation about a concrete problem: robot arms can assist
people, but their usual controllers can exclude people with an upper-limb
difference or limited hand function.

MagPilot explores whether a passive magnet, mounted wherever reliable movement
remains, could provide direct access to a robot arm. The sensing and real-robot
control are working today. The assistive mount is a design direction that still
needs to be co-designed and validated with users.

## Files

| File | Purpose |
|---|---|
| `MagPilot_Keynote.pptx` | 8-slide, 16:9 deck with speaker notes and two embedded videos |
| `build_keynote.py` | Rebuilds the presentation from the repository assets |
| `assets/demo_command.mp4` | 20-second air-written command proof |
| `assets/demo_pick.mp4` | 48-second package-pick proof |
| `assets/poster_command.jpg` | Poster frame for the command video |
| `assets/poster_pick.jpg` | Poster frame for the pick video |

Rebuild the deck from the repository root:

```bash
python3 presentation/build_keynote.py
```

## Running order

The complete talk is about 3 minutes 25 seconds, including 68 seconds of video.
The exact script and timestamps are in the speaker notes.

| Time | Slide | Point |
|---|---|---|
| 0:00 | MagPilot | One passive magnet can become the interface |
| 0:18 | Accessibility gap | The robot can help while its controller excludes |
| 0:40 | Assistive concept | Put the input where reliable movement remains |
| 1:00 | Control language | Move, raise, tilt, and lift away to pause |
| 1:20 | Proof 1 | Play the 20-second air-written command clip |
| 1:45 | Proof 2 | Play the 48-second package-pick clip |
| 2:38 | Working system | Show the real FR3, sensor array, loop, and workspace |
| 2:58 | Next step | Mount, co-design, and validate with users |

Both videos are embedded in the PowerPoint file. Click each blue play prompt,
let the clip run, and continue when it finishes. Avoid speaking over the clips
unless the room needs a brief explanation.

## Demo media

The short presentation clips are cut from the full masked 1080p demonstrations:

```bash
ffmpeg -ss 4 -i docs/demo_classify.mp4 -t 20 \
  -c:v libx264 -preset slow -crf 20 -c:a aac -movflags +faststart \
  presentation/assets/demo_command.mp4

ffmpeg -ss 18 -i docs/demo_teleop.mp4 -t 48 \
  -c:v libx264 -preset slow -crf 20 -c:a aac -movflags +faststart \
  presentation/assets/demo_pick.mp4
```

The full recordings remain linked from the project README for anyone who wants
to inspect the complete demonstrations.

## Delivery

Open the deck in PowerPoint and test both videos before presenting. Keynote and
LibreOffice Impress can import the file, but media playback should be checked on
the exact presentation computer.

Keep the assistive claim precise: MagPilot currently demonstrates the sensing,
mapping, and real-robot control mechanism. It has not yet been tested as an
assistive device. The final slide makes that next research step explicit.
