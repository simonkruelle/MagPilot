# MagPilot — pitch keynote

A startup-keynote-style deck for MagPilot: *pilot a robot arm with nothing but a
magnet.* Built to be presented in ~8–10 minutes with two live demo videos.

## Files

| File | What it is |
|---|---|
| `MagPilot_Keynote.pptx` | The deck — 16 slides, 16:9, **speaker notes on every slide**. Open in PowerPoint, Keynote, or LibreOffice Impress. |
| `build_keynote.py` | Regenerates the `.pptx` from scratch (`python3 presentation/build_keynote.py`). Edit copy/design here, not by hand, so it stays reproducible. |
| `assets/` | All images the deck embeds: the sensor-board close-up, wide setup shots, video poster frames, and UI screenshots. |

## The two demo videos

The deck has a **poster frame** on each demo slide with a `▶ PLAY:` cue. Drop the
matching video onto that slide in your presentation app and set it to play:

| Slide | Full 4K source | Repository-ready copy | Shows |
|---|---|---|---|
| 6 · *Air-writing* | `ABD.MP4` | `docs/demo_classify.mp4` | Writing A/B/D on the board; the arm reads and performs each. |
| 7 · *Teleoperation* | `Package_Lift_&_B.MP4` | `docs/demo_teleop.mp4` | Flying the arm, picking up a package, then a handwritten B. |

The original 4K recordings remain outside git in
`~/Downloads/COLMAG VIDEOS/`. The repository copies are H.264 1080p with the
busy upper-left background replaced by a solid backdrop-colored mask. Nothing
is blurred, so the robot stays sharp through its complete range of motion.
`assets/demo_mask_overlay.png` stores the exact opaque polygon mask.

To regenerate either repository copy:

```bash
ffmpeg -i INPUT.MP4 -i presentation/assets/demo_mask_overlay.png \
  -filter_complex \
  "[0:v]scale=1920:1080:flags=lanczos:out_color_matrix=bt709,\
   format=yuv420p[s];\
   [1:v]format=yuva420p[mask];\
   [s][mask]overlay=0:0:format=yuv420,format=yuv420p,\
   setparams=range=limited:color_primaries=bt709:\
   color_trc=bt709:colorspace=bt709[out]" \
  -map "[out]" -map 0:a? -c:v libx264 -preset slow -crf 21 \
  -x264-params "colorprim=bt709:transfer=bt709:colormatrix=bt709" \
  -c:a copy -movflags +faststart OUTPUT.mp4
```

The README uses eight-second animated GIF previews at the source-native 25 fps
because GitHub does not render HTML video controls consistently. Clicking a GIF
opens its full MP4 with audio.

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
