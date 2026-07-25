#!/usr/bin/env python3
"""build_keynote.py — generate the MagPilot investor/keynote pitch deck (.pptx).

Regenerate with:
    python3 presentation/build_keynote.py

Produces presentation/MagPilot_Keynote.pptx (16:9), designed as a startup
keynote: one idea per slide, big type, the MagPilot sky palette, and full
speaker notes on every slide. Demo slides carry a poster frame from the real
robot videos plus a note telling you which .mp4 to drop in (the raw videos are
gigabytes and are NOT committed — see presentation/README.md).
"""

import os

from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, 'assets')

# ── MagPilot palette ────────────────────────────────────────────────────────
INK      = RGBColor(0x10, 0x2A, 0x43)   # deep navy text
SKY      = RGBColor(0xEA, 0xF3, 0xFB)   # light card background
CARD     = RGBColor(0xFF, 0xFF, 0xFF)
BLUE     = RGBColor(0x0A, 0x84, 0xFF)   # accent
BLUE_DK  = RGBColor(0x06, 0x3A, 0x6E)   # deep brand blue (dark slides)
NAVY     = RGBColor(0x0A, 0x1E, 0x33)   # near-black navy (title/section bg)
SUBTLE   = RGBColor(0x5C, 0x74, 0x89)
MIST     = RGBColor(0xB9, 0xD3, 0xE8)
WHITE    = RGBColor(0xFF, 0xFF, 0xFF)
GREEN    = RGBColor(0x2E, 0xC4, 0x6B)

FONT = 'Arial'          # widely available on Linux/Mac/PowerPoint
EMU_IN = 914400

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
BLANK = prs.slide_layouts[6]
SW, SH = prs.slide_width, prs.slide_height


# ── helpers ─────────────────────────────────────────────────────────────────
def slide(bg=SKY):
    s = prs.slides.add_slide(BLANK)
    s.background.fill.solid()
    s.background.fill.fore_color.rgb = bg
    return s


def notes(s, text):
    s.notes_slide.notes_text_frame.text = text.strip()


def rect(s, l, t, w, h, color, line=None):
    shp = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, l, t, w, h)
    shp.fill.solid(); shp.fill.fore_color.rgb = color
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line; shp.line.width = Pt(1)
    shp.shadow.inherit = False
    return shp


def rounded(s, l, t, w, h, color):
    shp = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, l, t, w, h)
    shp.adjustments[0] = 0.08
    shp.fill.solid(); shp.fill.fore_color.rgb = color
    shp.line.fill.background(); shp.shadow.inherit = False
    return shp


def text(s, l, t, w, h, runs, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP,
         space_after=6, line_spacing=1.0):
    """runs: list of paragraphs; each paragraph is a list of (str, size, bold,
    color, [tracking]) run tuples, OR a single tuple for a one-run paragraph."""
    tb = s.shapes.add_textbox(l, t, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = 0; tf.margin_right = 0
    tf.margin_top = 0; tf.margin_bottom = 0
    for i, para in enumerate(runs):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.space_after = Pt(space_after)
        p.space_before = Pt(0)
        p.line_spacing = line_spacing
        if isinstance(para, tuple):
            para = [para]
        for run in para:
            txt, size, bold, color = run[0], run[1], run[2], run[3]
            r = p.add_run(); r.text = txt
            r.font.size = Pt(size); r.font.bold = bold
            r.font.color.rgb = color; r.font.name = FONT
    return tb


def img_fit(path, box_l, box_t, box_w, box_h, align='center', valign='middle'):
    """Return (l, t, w, h) that fits the image inside the box, preserving ratio."""
    iw, ih = Image.open(path).size
    ar = iw / ih
    box_ar = box_w / box_h
    if ar > box_ar:
        w = box_w; h = int(box_w / ar)
    else:
        h = box_h; w = int(box_h * ar)
    if align == 'center':
        l = box_l + (box_w - w) // 2
    elif align == 'left':
        l = box_l
    else:
        l = box_l + (box_w - w)
    if valign == 'middle':
        t = box_t + (box_h - h) // 2
    elif valign == 'top':
        t = box_t
    else:
        t = box_t + (box_h - h)
    return int(l), int(t), int(w), int(h)


def picture(s, path, box_l, box_t, box_w, box_h, **kw):
    l, t, w, h = img_fit(path, box_l, box_t, box_w, box_h, **kw)
    return s.shapes.add_picture(path, l, t, w, h)


def cover(s, path, l, t, w, h):
    """Fill the box with the image, cropping overflow (object-fit: cover)."""
    iw, ih = Image.open(path).size
    ar = iw / ih; box_ar = w / h
    pic = s.shapes.add_picture(path, l, t, w, h)
    if ar > box_ar:
        crop = (1 - box_ar / ar) / 2
        pic.crop_left = crop; pic.crop_right = crop
    else:
        crop = (1 - ar / box_ar) / 2
        pic.crop_top = crop; pic.crop_bottom = crop
    return pic


def kicker(s, l, t, txt, color=BLUE):
    text(s, l, t, Inches(9), Inches(0.4),
         [[(txt.upper(), 13, True, color)]])


def footer(s, n, dark=False):
    c = MIST if dark else SUBTLE
    text(s, Inches(0.55), Inches(7.02), Inches(4), Inches(0.35),
         [[('MagPilot', 10, True, c), ('  ·  pilot a robot arm with a magnet',
                                       10, False, c)]])
    text(s, Inches(11.3), Inches(7.02), Inches(1.5), Inches(0.35),
         [[('%02d' % n, 10, True, c)]], align=PP_ALIGN.RIGHT)


A = lambda name: os.path.join(ASSETS, name)


# ═══════════════════════════════════════════════════════════════════════════
# 1 — TITLE
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
# soft accent band
rect(s, 0, Inches(6.9), SW, Inches(0.6), BLUE_DK)
if os.path.exists(A('logo.png')):
    picture(s, A('logo.png'), Inches(0.85), Inches(0.8), Inches(1.2), Inches(1.2),
            align='left', valign='top')
text(s, Inches(0.85), Inches(2.35), Inches(11.6), Inches(2.2),
     [[('MagPilot', 88, True, WHITE)],
      [('Pilot a robot arm with nothing but a magnet.', 30, False, MIST)]],
     space_after=14)
text(s, Inches(0.9), Inches(5.15), Inches(11.6), Inches(0.9),
     [[('No joystick.  No teach pendant.  No code.', 20, True, BLUE)]])
text(s, Inches(0.9), Inches(6.98), Inches(11.6), Inches(0.5),
     [[('COLMAG  ·  TUM seminar project  ·  Simon Kruelle', 13, False, MIST)]])
notes(s, """
Opening line: "This is a Franka research robot — the kind of arm that normally
takes a 10,000-euro teach pendant and trained operators to move. I'm going to
fly it around, pick up a package, and have it recognise handwriting — using
nothing but this." (hold up the magnet.)
Pause. Let the contrast land before advancing.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 2 — PROBLEM
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'The problem')
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(1.6),
     [[('Moving a robot arm is still', 40, False, INK)],
      [('expensive, clunky, and expert-only.', 40, True, INK)]], space_after=4)
cards = [
    ('10,000 €+', 'for a teach pendant or joystick teleop rig'),
    ('Days', 'of training before an operator is productive'),
    ('Code', 'or CAD for every new motion or waypoint'),
]
cx = Inches(0.85); cw = Inches(3.72); gap = Inches(0.22)
for i, (big, small) in enumerate(cards):
    l = cx + i * (cw + gap)
    rounded(s, l, Inches(3.25), cw, Inches(2.7), CARD)
    text(s, l + Inches(0.35), Inches(3.6), cw - Inches(0.7), Inches(1.0),
         [[(big, 34, True, BLUE)]])
    text(s, l + Inches(0.35), Inches(4.55), cw - Inches(0.7), Inches(1.3),
         [[(small, 17, False, SUBTLE)]], line_spacing=1.05)
footer(s, 2)
notes(s, """
Every intuitive way to move a robot today costs real money and real training.
Teach pendants are thousands of euros; teleop rigs need joysticks, VR, or motion
capture; anything repeatable needs code. That's fine for a factory integrator —
it's a wall for everyone else: students, labs, small shops, assistive use.
The controller is the bottleneck, not the robot.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 3 — INSIGHT (statement)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
kicker(s, Inches(0.85), Inches(1.5), 'The insight', color=BLUE)
text(s, Inches(0.85), Inches(2.1), Inches(11.6), Inches(3.2),
     [[('A permanent magnet costs cents.', 46, True, WHITE)],
      [('Its magnetic field carries five signals through the air —', 30, False, MIST)],
      [('position, height, tilt and twist. We just had to listen.', 30, False, MIST)]],
     space_after=10, line_spacing=1.05)
footer(s, 3, dark=True)
notes(s, """
Here's the insight the whole product is built on. A magnet is the cheapest,
most robust "sensor beacon" in the world — no battery, no pairing, no wear.
A magnetometer grid can recover its full pose from the field alone. So the
controller everyone already has in a drawer becomes a five-degree-of-freedom
robot input. The hard part isn't the magnet — it's the software that turns
that field into safe, smooth robot motion. That's what we built.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 4 — SOLUTION (split: text + magpilot UI)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'The product')
text(s, Inches(0.85), Inches(1.15), Inches(6.6), Inches(2.0),
     [[('MagPilot turns one magnet', 34, True, INK)],
      [('into a complete robot interface.', 34, True, INK)]], space_after=2)
bullets = [
    ('Fly the arm', 'glide the magnet over a sensor board and the end-effector follows in real time'),
    ('Set the height', 'raise or lower the magnet to raise or lower the hand'),
    ('Grip & rotate', 'tilt to open/close the gripper, twist to rotate the wrist'),
    ('Air-write', 'draw a letter or digit and the robot performs the mapped action'),
]
y = Inches(3.05)
for title, desc in bullets:
    rect(s, Inches(0.9), y + Inches(0.06), Inches(0.12), Inches(0.55), BLUE)
    text(s, Inches(1.2), y, Inches(6.3), Inches(0.9),
         [[(title + '  —  ', 18, True, INK), (desc, 18, False, SUBTLE)]],
         line_spacing=1.02)
    y += Inches(0.95)
rounded(s, Inches(7.9), Inches(1.15), Inches(4.9), Inches(5.3), CARD)
if os.path.exists(A('ui_magpilot.png')):
    picture(s, A('ui_magpilot.png'), Inches(8.1), Inches(1.35),
            Inches(4.5), Inches(4.9))
footer(s, 4)
notes(s, """
MagPilot is the software layer that makes that magnet a real interface. Five
control channels, plus a writing mode: air-write a character and it triggers a
mapped action — a wave, a pick, a move to a target. One surface, no menus.
On the right is the live "flight deck" — your magnet becomes the little plane,
and the arm follows it.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 5 — FIVE CHANNELS
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
kicker(s, Inches(0.85), Inches(0.7), 'Five channels, hiding in a piece of metal',
       color=BLUE)
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(0.9),
     [[('You do  →  the robot does', 34, True, WHITE)]])
rows = [
    ('Glide over the board', 'End-effector moves in X / Y'),
    ('Raise / lower the magnet', 'End-effector height follows'),
    ('Tilt past a threshold', 'Gripper opens / closes'),
    ('Twist the magnet', 'Wrist rotates'),
    ('Lift it away', 'Arm pauses — walk away safely'),
]
y = Inches(2.35)
for you, robot in rows:
    rounded(s, Inches(0.85), y, Inches(11.63), Inches(0.78), BLUE_DK)
    text(s, Inches(1.2), y, Inches(5.6), Inches(0.78),
         [[(you, 18, True, WHITE)]], anchor=MSO_ANCHOR.MIDDLE)
    text(s, Inches(6.7), y, Inches(0.6), Inches(0.78),
         [[('→', 18, True, BLUE)]], anchor=MSO_ANCHOR.MIDDLE)
    text(s, Inches(7.3), y, Inches(5.0), Inches(0.78),
         [[(robot, 18, False, MIST)]], anchor=MSO_ANCHOR.MIDDLE)
    y += Inches(0.9)
footer(s, 5, dark=True)
notes(s, """
The full control vocabulary, and it's learnable in seconds because it's
physical, not abstract. Move to move, lift to raise, tilt to grip, twist to
rotate, and — crucially for safety — lift the magnet away and the arm freezes
so you can reposition without dragging it. No button assignment to memorise.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 6 — DEMO 1 : classification
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
cover(s, A('poster_classify.jpg'), 0, 0, SW, SH)
# left scrim for text legibility
scrim = rect(s, 0, 0, Inches(6.6), SH, NAVY)
scrim.fill.fore_color.rgb = NAVY
scrim.fill.transparency = 0  # set alpha via xml below
# apply ~55% transparency to the scrim
sp = scrim.fill._xPr.find(qn('a:solidFill'))
srgb = sp.find(qn('a:srgbClr'))
alpha = srgb.makeelement(qn('a:alpha'), {'val': '62000'})
srgb.append(alpha)
kicker(s, Inches(0.85), Inches(0.85), 'Live demo 1  ·  air-writing', color=WHITE)
text(s, Inches(0.85), Inches(1.4), Inches(5.6), Inches(3.0),
     [[('Write a letter.', 40, True, WHITE)],
      [('The robot reads it and', 40, True, WHITE)],
      [('performs the action.', 40, True, WHITE)]], space_after=2)
text(s, Inches(0.85), Inches(4.5), Inches(5.4), Inches(1.6),
     [[('Draw A, B, D over the board — the stroke is inked live, classified '
        'when you pause, and the arm executes on confirm.', 18, False, MIST)]],
     line_spacing=1.1)
rounded(s, Inches(0.85), Inches(6.1), Inches(4.7), Inches(0.62), BLUE)
text(s, Inches(0.85), Inches(6.1), Inches(4.7), Inches(0.62),
     [[('▶  PLAY:  ABD.MP4', 15, True, WHITE)]],
     align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
notes(s, """
DEMO — insert ABD.MP4 here (drag the file onto this slide, set "Play Full
Screen" / automatically). Talk track while it plays: "She's writing an A on the
sensor board with the magnet. The interface inks the stroke, the classifier
reads it, and on confirm the arm performs the mapped gesture. Then B, then D."
This is the "it recognises handwriting" beat from the opening promise.
Real Franka FR3, real magnet, no other input device.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 7 — DEMO 2 : teleop + pick
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
cover(s, A('poster_teleop.jpg'), 0, 0, SW, SH)
scrim = rect(s, 0, 0, Inches(6.6), SH, NAVY)
sp = scrim.fill._xPr.find(qn('a:solidFill'))
srgb = sp.find(qn('a:srgbClr'))
srgb.append(srgb.makeelement(qn('a:alpha'), {'val': '62000'}))
kicker(s, Inches(0.85), Inches(0.85), 'Live demo 2  ·  teleoperation', color=WHITE)
text(s, Inches(0.85), Inches(1.4), Inches(5.6), Inches(3.0),
     [[('Fly the arm.', 40, True, WHITE)],
      [('Pick up a package.', 40, True, WHITE)],
      [('All from the magnet.', 40, True, WHITE)]], space_after=2)
text(s, Inches(0.85), Inches(4.5), Inches(5.4), Inches(1.6),
     [[('Move, lower, tilt to grip, lift — a real pick-and-place, then a '
        'handwritten B to finish. Motion is jerk-limited for smooth, safe '
        'tracking.', 18, False, MIST)]], line_spacing=1.1)
rounded(s, Inches(0.85), Inches(6.1), Inches(5.4), Inches(0.62), BLUE)
text(s, Inches(0.85), Inches(6.1), Inches(5.4), Inches(0.62),
     [[('▶  PLAY:  Package_Lift_&_B.MP4', 15, True, WHITE)]],
     align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
notes(s, """
DEMO — insert Package_Lift_&_B.MP4 here. Talk track: "Now teleoperation. She
glides the magnet to move the arm over the box, lowers it by lowering the
magnet, tilts to close the gripper, and lifts the package — then finishes with a
handwritten B. Every motion is smoothed by a jerk-limited controller so the real
arm tracks cleanly." This is the payoff slide — let the video breathe.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 8 — HOW IT WORKS (three pillars)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'Under the hood')
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(0.9),
     [[('Three systems, one magnet', 34, True, INK)]])
pillars = [
    ('Sensing', 'A 48-channel magnetometer grid samples the field 30× a second. '
                'A dipole model recovers the magnet\'s position, height, tilt and twist.'),
    ('Recognition', 'Strokes are inked into a 64-px canvas with a velocity gate '
                    'and classified by an OCR backend into letters and digits.'),
    ('Motion', 'Damped-least-squares IK streams to the arm through a jerk-limited '
               'S-curve, so targets never jump and the real robot stays smooth.'),
]
cx = Inches(0.85); cw = Inches(3.72); gap = Inches(0.22)
for i, (title, body) in enumerate(pillars):
    l = cx + i * (cw + gap)
    rounded(s, l, Inches(2.4), cw, Inches(3.9), CARD)
    rect(s, l + Inches(0.35), Inches(2.75), Inches(0.55), Inches(0.12), BLUE)
    text(s, l + Inches(0.35), Inches(3.0), cw - Inches(0.7), Inches(0.7),
         [[(title, 24, True, INK)]])
    text(s, l + Inches(0.35), Inches(3.75), cw - Inches(0.7), Inches(2.4),
         [[(body, 15.5, False, SUBTLE)]], line_spacing=1.15)
footer(s, 8)
notes(s, """
Three subsystems. Sensing turns the field into a pose. Recognition turns strokes
into commands. Motion turns commands into safe robot trajectories — and that
last part is where most of the engineering went: a jerk-limited streaming
controller so a 30 Hz magnet signal drives a real arm without vibration.
Everything runs on ROS in one Docker container.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 9 — THE HARDWARE (sensor board hero)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
picture(s, A('hw_sensor_board.jpg'), Inches(0.5), Inches(0.5), Inches(5.7),
        Inches(6.5), align='center', valign='middle')
kicker(s, Inches(6.9), Inches(1.6), 'The hardware', color=BLUE)
text(s, Inches(6.9), Inches(2.15), Inches(5.7), Inches(3.6),
     [[('A 4×4 grid of', 32, True, WHITE)],
      [('off-the-shelf', 32, True, WHITE)],
      [('magnetometers.', 32, True, WHITE)],
      [('', 12, False, WHITE)],
      [('16 sensors, one breakout board, a flat pad on the desk. The only '
        'thing in the user\'s hand is a magnet.', 18, False, MIST)]],
     space_after=4, line_spacing=1.05)
notes(s, """
This is the entire sensing rig: sixteen magnetometer modules on a flat board,
one microcontroller reading them out. It's cheap, it's flat, it hides under a
cloth. Contrast the cost: a teach pendant is thousands; this is a handful of
sensor chips and a magnet. The bill of materials is the moat's opposite — it's
why this can be everywhere.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 10 — FULL SETUP (wide photo)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
cover(s, A('setup_wide.jpg'), 0, 0, SW, SH)
band = rect(s, 0, Inches(6.15), SW, Inches(1.35), NAVY)
sp = band.fill._xPr.find(qn('a:solidFill'))
srgb = sp.find(qn('a:srgbClr'))
srgb.append(srgb.makeelement(qn('a:alpha'), {'val': '55000'}))
text(s, Inches(0.85), Inches(6.35), Inches(11.6), Inches(1.0),
     [[('The whole workcell:  ', 20, True, WHITE),
       ('robot, a flat sensor board, a magnet, and a screen. '
        'Nothing worn, nothing wired to the operator.', 20, False, MIST)]],
     anchor=MSO_ANCHOR.MIDDLE)
notes(s, """
The full setup in the lab. Point out how ordinary it is: the operator just sits
at a table with a magnet. No gloves, no VR headset, no tracker on the hand.
That approachability is the product.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 11 — SIM → REAL (staged safety)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'De-risked by design')
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(1.5),
     [[('Full digital twin first,', 36, True, INK)],
      [('then a staged path to the real robot.', 36, True, INK)]], space_after=2)
steps = [
    ('1', 'Simulation', 'The whole pipeline runs in Gazebo — validate every mapping before hardware.'),
    ('2', 'Dry-run', 'On the real arm, the controller logs targets and IK residuals but sends no motion.'),
    ('3', 'Supervised live', 'Enable motion with an E-stop reachable; the arm pauses when the magnet lifts away.'),
]
cx = Inches(0.85); cw = Inches(3.72); gap = Inches(0.22)
for i, (num, title, body) in enumerate(steps):
    l = cx + i * (cw + gap)
    rounded(s, l, Inches(3.0), cw, Inches(3.2), CARD)
    circ = s.shapes.add_shape(MSO_SHAPE.OVAL, l + Inches(0.35), Inches(3.35),
                              Inches(0.7), Inches(0.7))
    circ.fill.solid(); circ.fill.fore_color.rgb = BLUE
    circ.line.fill.background(); circ.shadow.inherit = False
    tf = circ.text_frame; tf.word_wrap = False
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
    r = p.add_run(); r.text = num
    r.font.size = Pt(24); r.font.bold = True; r.font.color.rgb = WHITE
    r.font.name = FONT
    text(s, l + Inches(0.35), Inches(4.25), cw - Inches(0.7), Inches(0.6),
         [[(title, 21, True, INK)]])
    text(s, l + Inches(0.35), Inches(4.9), cw - Inches(0.7), Inches(1.3),
         [[(body, 15, False, SUBTLE)]], line_spacing=1.15)
footer(s, 11)
notes(s, """
Safety and credibility. We never point software at an expensive arm and hope.
Everything is validated in a Gazebo digital twin, then dry-run on the real robot
(no motion, just logged targets), then supervised live with an E-stop. And the
lift-to-pause gesture is a safety primitive, not a feature. Investors and lab
supervisors both care about this slide.
[Optional: add a Gazebo screen-capture here once recorded.]
""")

# ═══════════════════════════════════════════════════════════════════════════
# 12 — PRODUCT UX (launcher)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'Productised, not a script')
text(s, Inches(0.85), Inches(1.15), Inches(6.4), Inches(2.4),
     [[('One window.', 36, True, INK)],
      [('Zero terminals.', 36, True, INK)],
      [('', 10, False, INK)],
      [('Robot, control nodes and interface each start with one button. '
        'Status dots poll live; one click returns to a clean slate. '
        'It already feels like a product, not a research demo.', 18, False, SUBTLE)]],
     space_after=4, line_spacing=1.1)
rounded(s, Inches(7.7), Inches(1.3), Inches(5.1), Inches(4.9), CARD)
if os.path.exists(A('ui_launcher.png')):
    picture(s, A('ui_launcher.png'), Inches(7.9), Inches(1.5),
            Inches(4.7), Inches(4.5))
footer(s, 12)
notes(s, """
The whole stack — a real robot, ROS, Docker, controllers — is driven from one
control center. That matters commercially: the distance from "research code" to
"a thing a non-expert can run" is most of the work, and it's done. This is what
makes it demoable anywhere in one click.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 13 — MARKET / WHO
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
kicker(s, Inches(0.85), Inches(0.7), 'Where it goes', color=BLUE)
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(0.9),
     [[('A cent-cheap controller unlocks robots for everyone', 30, True, WHITE)]])
markets = [
    ('Education & labs', 'Teach robotics without a teach pendant — every student gets an intuitive arm.'),
    ('Rapid prototyping', 'Jog and program cells by hand in seconds, no CAD or pendant round-trips.'),
    ('Assistive & accessible', 'A low-effort, wearable-free input for users who can\'t use joysticks or VR.'),
    ('Field & shared robots', 'No device to pair, charge or lose — hand someone a magnet and go.'),
]
positions = [(Inches(0.85), Inches(2.4)), (Inches(6.75), Inches(2.4)),
             (Inches(0.85), Inches(4.55)), (Inches(6.75), Inches(4.55))]
for (l, t), (title, body) in zip(positions, markets):
    rounded(s, l, t, Inches(5.73), Inches(1.95), BLUE_DK)
    text(s, l + Inches(0.4), t + Inches(0.28), Inches(5.0), Inches(0.6),
         [[(title, 20, True, WHITE)]])
    text(s, l + Inches(0.4), t + Inches(0.85), Inches(5.0), Inches(1.0),
         [[(body, 15.5, False, MIST)]], line_spacing=1.12)
footer(s, 13, dark=True)
notes(s, """
Who buys this. Wherever the controller — not the robot — is the barrier.
Education is the beachhead: cheap, safe, intuitive, and every robotics course
needs it. From there: rapid cell prototyping, accessible/assistive control, and
shared or field robots where there's nothing to pair or charge. The magnet is
the cheapest, most durable input device that exists.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 14 — WHY NOW
# ═══════════════════════════════════════════════════════════════════════════
s = slide(SKY)
kicker(s, Inches(0.85), Inches(0.7), 'Why now')
text(s, Inches(0.85), Inches(1.15), Inches(11.6), Inches(1.4),
     [[('The hardware got cheap.', 38, True, INK)],
      [('The software was the missing piece.', 38, True, BLUE)]], space_after=2)
points = [
    ('Sensors are commodities', 'High-resolution magnetometer arrays now cost cents per channel.'),
    ('Robots are proliferating', 'Collaborative arms are everywhere — and all still need an easier way in.'),
    ('We built the hard part', 'A smooth, safe, real-time magnet-to-robot control stack — running today on a real FR3.'),
]
y = Inches(3.2)
for title, body in points:
    rect(s, Inches(0.9), y + Inches(0.05), Inches(0.12), Inches(0.9), BLUE)
    text(s, Inches(1.2), y, Inches(11.0), Inches(1.0),
         [[(title + '  —  ', 20, True, INK), (body, 20, False, SUBTLE)]],
         line_spacing=1.05)
    y += Inches(1.05)
footer(s, 14)
notes(s, """
Timing. The sensing hardware finally got cheap and good enough; collaborative
robots are everywhere and still hard to drive; and the piece nobody had built —
a real-time, jerk-limited magnet-to-motion control stack that works on real
hardware — is what we've made. That's the defensible part.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 15 — VISION (statement)
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
text(s, Inches(1.0), Inches(2.4), Inches(11.3), Inches(2.8),
     [[('Every magnet', 54, True, WHITE)],
      [('is a robot controller.', 54, True, BLUE)]], space_after=6)
text(s, Inches(1.02), Inches(5.0), Inches(10.5), Inches(1.0),
     [[('MagPilot makes the most intuitive robot interface also the cheapest.',
        22, False, MIST)]])
footer(s, 15, dark=True)
notes(s, """
The vision in one line. We want the cheapest object in the room to be the way
you command the most capable one. Land this, then go to the close.
""")

# ═══════════════════════════════════════════════════════════════════════════
# 16 — CLOSE / THANK YOU
# ═══════════════════════════════════════════════════════════════════════════
s = slide(NAVY)
rect(s, 0, Inches(6.9), SW, Inches(0.6), BLUE_DK)
if os.path.exists(A('logo.png')):
    picture(s, A('logo.png'), Inches(0.85), Inches(1.3), Inches(1.1), Inches(1.1),
            align='left', valign='top')
text(s, Inches(0.85), Inches(2.9), Inches(11.6), Inches(2.0),
     [[('Thank you.', 60, True, WHITE)],
      [('Pilot a robot arm with nothing but a magnet.', 26, False, MIST)]],
     space_after=12)
text(s, Inches(0.9), Inches(5.4), Inches(11.6), Inches(0.6),
     [[('MagPilot  ·  COLMAG  ·  ', 16, True, BLUE),
       ('github.com/simonkruelle  ·  simon.kruelle@gmail.com', 16, False, MIST)]])
notes(s, """
Close on the same line you opened with — the magnet in your hand. Invite
questions, and offer a hands-on: let someone in the room fly the arm with the
magnet. The live hand-off is the strongest possible ending.
""")

out = os.path.join(HERE, 'MagPilot_Keynote.pptx')
prs.save(out)
print('Wrote', out, '—', len(prs.slides._sldIdLst), 'slides')
