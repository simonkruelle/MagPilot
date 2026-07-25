# MagPilot — pitch keynote

A startup-keynote-style deck for MagPilot: *pilot a robot arm with nothing but a
magnet.* Built to be presented in ~8–10 minutes with two live demo videos.

## Files

| File | What it is |
|---|---|
| `MagPilot_Keynote.pptx` | The deck — 16 slides, 16:9, **speaker notes on every slide**. Open in PowerPoint, Keynote, or LibreOffice Impress. |
| `build_keynote.py` | Regenerates the `.pptx` from scratch (`python3 presentation/build_keynote.py`). Edit copy/design here, not by hand, so it stays reproducible. |
| `assets/` | All images the deck embeds: the sensor-board close-up, wide setup shots, video poster frames, and UI screenshots. |

## The two demo videos (not in git — they're gigabytes)

The deck has a **poster frame** on each demo slide with a `▶ PLAY:` cue. Drop the
matching video onto that slide in your presentation app and set it to play:

| Slide | Insert this video | Shows |
|---|---|---|
| 6 · *Air-writing* | `ABD.MP4` | Writing A/B/D on the board; the arm reads and performs each. |
| 7 · *Teleoperation* | `Package_Lift_&_B.MP4` | Flying the arm, picking up a package, then a handwritten B. |

**Cleaned versions.** `build`-adjacent ffmpeg produced `*_clean.mp4` copies next to
the originals (in `~/Downloads/COLMAG VIDEOS/`): downscaled to 1080p with a
"spotlight" treatment — sharp centre, softly blurred + darkened edges — so the
people walking in the background recede. Prefer these for the talk.

To clean any other clip the same way:

```bash
# one-time: make the feathered radial mask
convert -size 1920x1080 radial-gradient:white-black -level 12%,72% spotlight_mask.png

# then per video (sharp centre, blurred/darkened surround, 1080p)
ffmpeg -i INPUT.MP4 -i spotlight_mask.png -filter_complex \
 "[0:v]scale=1920:1080,setsar=1[s];[s]split[a][b];\
  [b]boxblur=26:2,eq=brightness=-0.06[bl];\
  [1:v]format=gray,scale=1920:1080[m];\
  [bl][a][m]maskedmerge[out]" \
 -map "[out]" -map 0:a? -c:v libx264 -preset veryfast -crf 23 -c:a aac OUTPUT_clean.mp4
```

Tighter core = less blur: raise the `-level` low value (e.g. `18%,78%`).

## Running order (≈9 min)

1. **Title** — hold up the magnet. "I'll fly this arm and have it read handwriting
   using only *this*."
2. **Problem** — teleop is expensive, expert-only.
3. **Insight** — a magnet costs cents and carries five signals through the air.
4. **Product** — the five channels + air-writing.
5. **Five channels** — the control vocabulary.
6. **Demo 1** — play `ABD.MP4` (classification).
7. **Demo 2** — play `Package_Lift_&_B.MP4` (teleop + pick).
8. **How it works** — sensing / recognition / motion.
9. **Hardware** — the 4×4 magnetometer board.
10. **Setup** — the whole workcell.
11. **Sim → real** — the staged, de-risked path.
12. **Product UX** — one-window control center.
13. **Market** — where it goes.
14. **Why now** — cheap sensors, the software is the moat.
15. **Vision** — "every magnet is a robot controller."
16. **Close** — offer a hands-on: let someone fly the arm.

## Still to add (optional)

- A **Gazebo screen-capture** on slide 11 (simulation). Record the sim following
  the trackpad cursor, drop it in.
- Trim the demo clips to their best ~20 s if the full takes run long.
