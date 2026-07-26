#!/usr/bin/env python3
"""Generate the short MagPilot assistive-robotics keynote.

The deck is deliberately concise: eight slides, about 147 seconds of spoken
material, and 67 seconds of auto-playing demo GIFs. At a calm pace it runs for
roughly 3:34, leaving room for pauses while staying below five minutes.

Regenerate with:
    python3 presentation/build_keynote.py
"""

import os

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ASSETS = os.path.join(HERE, "assets")
DOCS = os.path.join(ROOT, "docs")

INK = RGBColor(0x10, 0x2A, 0x43)
NAVY = RGBColor(0x09, 0x1A, 0x2B)
BLUE_DK = RGBColor(0x06, 0x3A, 0x6E)
BLUE = RGBColor(0x0A, 0x84, 0xFF)
SKY = RGBColor(0xEA, 0xF3, 0xFB)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
MIST = RGBColor(0xB9, 0xD3, 0xE8)
SUBTLE = RGBColor(0x5C, 0x74, 0x89)
GREEN = RGBColor(0x2E, 0xC4, 0x6B)
WARM = RGBColor(0xFF, 0xC8, 0x57)
FONT = "Arial"

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
BLANK = prs.slide_layouts[6]
SW, SH = prs.slide_width, prs.slide_height


def make_slide(bg=SKY):
    s = prs.slides.add_slide(BLANK)
    s.background.fill.solid()
    s.background.fill.fore_color.rgb = bg
    return s


def add_notes(s, body):
    s.notes_slide.notes_text_frame.text = body.strip()


def box(s, left, top, width, height, color, radius=False, line=None):
    shape_type = MSO_SHAPE.ROUNDED_RECTANGLE if radius else MSO_SHAPE.RECTANGLE
    shp = s.shapes.add_shape(shape_type, left, top, width, height)
    if radius:
        shp.adjustments[0] = 0.08
    shp.fill.solid()
    shp.fill.fore_color.rgb = color
    if line is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line
        shp.line.width = Pt(1)
    shp.shadow.inherit = False
    return shp


def set_opacity(shp, opacity):
    solid = shp.fill._xPr.find(qn("a:solidFill"))
    color = solid.find(qn("a:srgbClr"))
    color.append(color.makeelement(qn("a:alpha"), {
        "val": str(int(max(0.0, min(1.0, opacity)) * 100000))
    }))


def add_text(s, left, top, width, height, paragraphs,
             align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP,
             space_after=4, line_spacing=1.0):
    """Add styled text.

    ``paragraphs`` is a list of paragraphs. Each paragraph is either one
    ``(text, size, bold, color)`` tuple or a list of those tuples.
    """
    tb = s.shapes.add_textbox(left, top, width, height)
    tf = tb.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = 0
    tf.margin_top = tf.margin_bottom = 0

    for index, paragraph in enumerate(paragraphs):
        p = tf.paragraphs[0] if index == 0 else tf.add_paragraph()
        p.alignment = align
        p.space_before = Pt(0)
        p.space_after = Pt(space_after)
        p.line_spacing = line_spacing
        runs = [paragraph] if isinstance(paragraph, tuple) else paragraph
        for value, size, bold, color in runs:
            run = p.add_run()
            run.text = value
            run.font.name = FONT
            run.font.size = Pt(size)
            run.font.bold = bold
            run.font.color.rgb = color
    return tb


def fit_image(path, left, top, width, height):
    iw, ih = Image.open(path).size
    image_ratio = iw / ih
    box_ratio = width / height
    if image_ratio > box_ratio:
        w = width
        h = int(width / image_ratio)
    else:
        h = height
        w = int(height * image_ratio)
    return (
        int(left + (width - w) / 2),
        int(top + (height - h) / 2),
        int(w),
        int(h),
    )


def add_picture(s, path, left, top, width, height):
    l, t, w, h = fit_image(path, left, top, width, height)
    return s.shapes.add_picture(path, l, t, w, h)


def add_cover(s, path, left, top, width, height):
    iw, ih = Image.open(path).size
    image_ratio = iw / ih
    box_ratio = width / height
    pic = s.shapes.add_picture(path, left, top, width, height)
    if image_ratio > box_ratio:
        crop = (1 - box_ratio / image_ratio) / 2
        pic.crop_left = crop
        pic.crop_right = crop
    else:
        crop = (1 - image_ratio / box_ratio) / 2
        pic.crop_top = crop
        pic.crop_bottom = crop
    return pic


def kicker(s, value, dark=False, left=Inches(0.78), top=Inches(0.55)):
    add_text(
        s, left, top, Inches(8.5), Inches(0.35),
        [(value.upper(), 13, True, WHITE if dark else BLUE)],
    )


def footer(s, number, dark=False):
    color = MIST if dark else SUBTLE
    add_text(
        s, Inches(0.6), Inches(7.08), Inches(4), Inches(0.25),
        [("MagPilot | TUM ARIES Lab", 9, False, color)],
    )
    add_text(
        s, Inches(12), Inches(7.08), Inches(0.7), Inches(0.25),
        [(str(number), 9, True, color)],
        align=PP_ALIGN.RIGHT,
    )


def metric(s, left, top, number, label, dark=False):
    primary = WHITE if dark else INK
    secondary = MIST if dark else SUBTLE
    add_text(
        s, left, top, Inches(3.5), Inches(0.72),
        [(number, 28, True, BLUE)],
    )
    add_text(
        s, left, top + Inches(0.65), Inches(3.5), Inches(0.75),
        [(label, 14, False, secondary)],
        line_spacing=1.05,
    )
    return primary


SETUP = os.path.join(DOCS, "setup.png")
SENSOR = os.path.join(DOCS, "hardware.jpg")
WRITING_UI = os.path.join(DOCS, "interface.png")
UI = os.path.join(DOCS, "magpilot.png")
LAUNCHER = os.path.join(DOCS, "launcher.png")
LOGO = os.path.join(ASSETS, "logo.png")
TUM_LOGO = os.path.join(ASSETS, "tum-logo.png")
MIRMI_LOGO = os.path.join(DOCS, "affiliations", "mirmi-logo.png")
ASSISTIVE_CONCEPT = os.path.join(ASSETS, "assistive_concept.png")
GIF_CLASSIFY = os.path.join(ASSETS, "demo_classify_slide.gif")
GIF_TELEOP = os.path.join(ASSETS, "demo_teleop_slide.gif")


# 1. Title and promise
s = make_slide(NAVY)
add_cover(s, SETUP, 0, 0, SW, SH)
scrim = box(s, 0, 0, Inches(7.45), SH, NAVY)
set_opacity(scrim, 0.90)
add_picture(
    s, LOGO, Inches(0.8), Inches(0.62), Inches(0.82), Inches(0.82)
)
add_text(
    s, Inches(0.8), Inches(1.72), Inches(6.25), Inches(1.35),
    [("MagPilot", 72, True, WHITE)],
)
add_text(
    s, Inches(0.82), Inches(3.02), Inches(5.95), Inches(1.65),
    [
        ("Robot control from", 33, False, MIST),
        ("one passive magnet.", 37, True, WHITE),
    ],
    space_after=4,
)
box(s, Inches(0.82), Inches(5.12), Inches(0.12), Inches(0.92), BLUE)
add_text(
    s, Inches(1.15), Inches(5.1), Inches(5.7), Inches(1.1),
    [
        ("Toward assistance without a keyboard, joystick, or hand-held controller.",
         20, True, WHITE)
    ],
    line_spacing=1.05,
)
add_text(
    s, Inches(0.82), Inches(6.76), Inches(6.3), Inches(0.35),
    [("COLMAG | TUM ARIES Lab | Simon Kruelle", 11, False, MIST)],
)
add_notes(s, """
[0:00-0:18]
Robot arms can already pick up objects for us. But their controllers still
assume that a person can grip a joystick, use a keyboard, or operate a teach
pendant. MagPilot asks a different question: can one passive magnet become the
entire interface?
""")


# 2. The accessibility gap
s = make_slide(SKY)
kicker(s, "The accessibility gap")
add_text(
    s, Inches(0.78), Inches(1.05), Inches(11.9), Inches(1.55),
    [
        ("The robot can help.", 42, False, INK),
        ("The controller can exclude.", 42, True, INK),
    ],
    space_after=2,
)
labels = [
    ("KEYBOARD", "precise reach and finger control"),
    ("JOYSTICK", "a reliable grip"),
    ("TEACH PENDANT", "both hands and training"),
]
for i, (title, detail) in enumerate(labels):
    left = Inches(0.8 + i * 4.1)
    box(s, left, Inches(3.12), Inches(3.55), Inches(0.08), BLUE)
    add_text(
        s, left, Inches(3.48), Inches(3.55), Inches(0.45),
        [(title, 14, True, BLUE)],
    )
    add_text(
        s, left, Inches(4.05), Inches(3.55), Inches(1.0),
        [(detail, 22, True, INK)],
        line_spacing=1.05,
    )
band = box(s, 0, Inches(5.72), SW, Inches(1.2), NAVY)
add_text(
    s, Inches(0.82), Inches(5.72), Inches(11.8), Inches(1.2),
    [
        ("For a person with an upper-limb difference or limited hand function, ",
         19, False, MIST),
        ("the interface can be the barrier.", 19, True, WHITE),
    ],
    anchor=MSO_ANCHOR.MIDDLE,
)
footer(s, 2)
add_notes(s, """
[0:18-0:40]
For people with an upper-limb difference or limited hand function, the robot
may be capable while its interface is not. Keyboards need finger precision,
joysticks need grip, and teach pendants need both. The problem I want to address
is not robot capability. It is access to that capability.
""")


# 3. Assistive concept
s = make_slide(NAVY)
add_cover(s, ASSISTIVE_CONCEPT, 0, 0, SW, SH)
concept_scrim = box(s, 0, 0, Inches(3.75), SH, NAVY)
set_opacity(concept_scrim, 0.92)
kicker(
    s, "The assistive direction", dark=True,
    left=Inches(0.58), top=Inches(0.48),
)
add_text(
    s, Inches(0.58), Inches(1.1), Inches(2.8), Inches(1.92),
    [
        ("Put the input", 26, False, WHITE),
        ("where reliable movement remains.", 26, True, BLUE),
    ],
    space_after=2,
)
add_text(
    s, Inches(0.6), Inches(3.34), Inches(2.72), Inches(1.42),
    [
        ("A passive magnet on a comfortable cuff could turn residual-limb "
         "movement into direct robot control.", 15, False, MIST)
    ],
    line_spacing=1.08,
)
box(s, Inches(0.6), Inches(5.12), Inches(0.09), Inches(0.82), BLUE)
add_text(
    s, Inches(0.92), Inches(5.12), Inches(2.4), Inches(0.86),
    [
        ("NO BATTERY\nNO BUTTONS\nNO GRIP REQUIRED", 12, True, WHITE),
    ],
    space_after=2,
)
add_text(
    s, Inches(0.6), Inches(6.86), Inches(2.78), Inches(0.34),
    [("AI-GENERATED CONCEPT\nNOT USER-VALIDATED", 8, True, MIST)],
    space_after=1,
)
add_notes(s, """
[0:40-1:02]
This image is an AI-generated concept, not a user trial. The proposed assistive
form is simple: place a passive magnet on a comfortable cuff wherever a person
has reliable movement, for example on a residual limb. Nothing needs charging,
gripping, or pressing. The sensor array reads that movement through the air.
""")


# 4. Two operating modes
s = make_slide(SKY)
kicker(s, "Two modes, one interface")
add_text(
    s, Inches(0.78), Inches(1.02), Inches(11.8), Inches(0.72),
    [("Choose a task, or pilot continuously.", 32, True, INK)],
)
add_text(
    s, Inches(0.8), Inches(1.66), Inches(11.7), Inches(0.26),
    [
        ("MAGNET-ONLY SWITCH  |  Dwell on MagPilot to enter, "
         "Draw to return.", 13, True, BLUE)
    ],
    align=PP_ALIGN.LEFT,
)
mode_columns = [
    (
        Inches(0.78),
        "01  CLASSIFIED COMMANDS",
        "Write a letter or digit",
        "The classifier maps the symbol to a stored robot task.",
        WRITING_UI,
        "AIR-WRITE  |  CLASSIFY  |  EXECUTE",
    ),
    (
        Inches(6.82),
        "02  MAGPILOT TELEOPERATION",
        "Move the magnet directly",
        "X/Y, height, and gripper state follow continuously.",
        UI,
        "MOVE  |  RAISE  |  TILT  |  LIFT AWAY TO PAUSE",
    ),
]
for left, label, title, detail, image_path, flow in mode_columns:
    box(s, left, Inches(2.03), Inches(5.72), Inches(4.72), WHITE, radius=True)
    add_text(
        s, left + Inches(0.25), Inches(2.22), Inches(5.2), Inches(0.3),
        [(label, 11, True, BLUE)],
    )
    add_text(
        s, left + Inches(0.25), Inches(2.57), Inches(5.2), Inches(0.42),
        [(title, 21, True, INK)],
    )
    add_text(
        s, left + Inches(0.25), Inches(3.0), Inches(5.2), Inches(0.42),
        [(detail, 13, False, SUBTLE)],
    )
    add_picture(
        s, image_path, left + Inches(0.25), Inches(3.5),
        Inches(5.22), Inches(2.55),
    )
    add_text(
        s, left + Inches(0.25), Inches(6.25), Inches(5.2), Inches(0.28),
        [(flow, 10, True, BLUE)],
        align=PP_ALIGN.CENTER,
    )
footer(s, 4)
add_notes(s, """
[1:02-1:27]
The same interface offers two ways to work. In command mode, a person writes a
letter or digit and the classifier maps it to a stored robot task. In MagPilot
mode, the magnet becomes a continuous controller for position, height, and the
gripper. The magnet itself switches modes by dwelling on MagPilot to enter or
Draw to return. Lifting it away pauses the arm.
""")


# 5. Auto-playing classified-command demo
s = make_slide(NAVY)
s.shapes.add_picture(GIF_CLASSIFY, 0, 0, SW, SH)
top_band = box(s, 0, 0, SW, Inches(1.03), NAVY)
set_opacity(top_band, 0.92)
add_text(
    s, Inches(0.68), Inches(0.18), Inches(9.8), Inches(0.7),
    [
        ("PROOF 1  ", 13, True, BLUE),
        ("One movement becomes a command.", 26, True, WHITE),
    ],
    anchor=MSO_ANCHOR.MIDDLE,
)
badge = box(
    s, Inches(10.62), Inches(0.23), Inches(1.98), Inches(0.52), BLUE, radius=True
)
add_text(
    s, Inches(10.62), Inches(0.23), Inches(1.98), Inches(0.52),
    [("AUTO PLAY  0:39", 10, True, WHITE)],
    align=PP_ALIGN.CENTER,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[1:27-2:11]
This is classified-command mode on the real robot. The animation starts
automatically and runs for 39 seconds. Let it play without speaking over it.
""")


# 6. Auto-playing MagPilot demo
s = make_slide(NAVY)
s.shapes.add_picture(GIF_TELEOP, 0, 0, SW, SH)
top_band = box(s, 0, 0, SW, Inches(1.03), NAVY)
set_opacity(top_band, 0.92)
add_text(
    s, Inches(0.68), Inches(0.18), Inches(9.8), Inches(0.7),
    [
        ("PROOF 2  ", 13, True, BLUE),
        ("One magnet picks up an object.", 26, True, WHITE),
    ],
    anchor=MSO_ANCHOR.MIDDLE,
)
badge = box(
    s, Inches(10.62), Inches(0.23), Inches(1.98), Inches(0.52), BLUE, radius=True
)
add_text(
    s, Inches(10.62), Inches(0.23), Inches(1.98), Inches(0.52),
    [("AUTO PLAY  0:28", 10, True, WHITE)],
    align=PP_ALIGN.CENTER,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[2:11-2:44]
Now MagPilot teleoperation: move, lower, close the gripper, and lift the
package. The animation starts automatically and runs for 28 seconds. This is
the direct proof that magnetic input can perform a useful physical task.
""")


# 7. Modular verification pipeline
s = make_slide(SKY)
kicker(s, "Modular by design", left=Inches(0.68), top=Inches(0.55))
add_text(
    s, Inches(0.68), Inches(1.02), Inches(4.25), Inches(1.35),
    [
        ("One control center.", 30, False, INK),
        ("Verify three ways.", 30, True, INK),
    ],
    space_after=3,
)
layers = [
    ("1", "TRACKPAD INPUT", "Simulate the magnet sensor for fast debugging."),
    ("2", "GAZEBO", "Verify robot motion safely before hardware."),
    ("3", "REAL SENSOR + FR3", "Run the same GUI and ROS command path."),
]
for index, (number, title, detail) in enumerate(layers):
    top = Inches(2.55 + index * 1.22)
    add_text(
        s, Inches(0.72), top, Inches(0.48), Inches(0.48),
        [(number, 23, True, BLUE)],
    )
    add_text(
        s, Inches(1.3), top + Inches(0.02), Inches(3.25), Inches(0.32),
        [(title, 13, True, INK)],
    )
    add_text(
        s, Inches(1.3), top + Inches(0.4), Inches(3.25), Inches(0.62),
        [(detail, 13, False, SUBTLE)],
        line_spacing=1.05,
    )
box(s, Inches(5.0), Inches(0.62), Inches(7.72), Inches(6.18), WHITE, radius=True)
add_picture(
    s, LAUNCHER, Inches(5.2), Inches(0.83), Inches(7.32), Inches(5.72)
)
add_text(
    s, Inches(5.25), Inches(6.47), Inches(7.2), Inches(0.27),
    [("Simulation and real hardware share the same staged launch workflow.",
      10, True, SUBTLE)],
    align=PP_ALIGN.CENTER,
)
footer(s, 7)
add_notes(s, """
[2:44-3:07]
The pipeline is fully modular. Trackpad mode simulates the magnet sensor for
quick debugging. Gazebo verifies the same robot commands safely. Then the real
sensor and FR3 use the same GUI and ROS path. The Control Center makes those
input and backend swaps explicit without changing the staged workflow.
""")


# 8. Honest next step and close
s = make_slide(NAVY)
add_cover(s, SENSOR, Inches(7.72), 0, Inches(5.61), SH)
box(s, Inches(7.62), 0, Inches(0.1), SH, BLUE)
kicker(s, "The next step", dark=True)
add_text(
    s, Inches(0.78), Inches(1.02), Inches(6.4), Inches(1.35),
    [
        ("The mechanism works.", 34, False, WHITE),
        ("Now design the tool with users.", 34, True, BLUE),
    ],
    space_after=2,
)
steps = [
    ("01", "MOUNT", "Build a light, adjustable cuff."),
    ("02", "CO-DESIGN", "Choose movements and useful tasks together."),
    ("03", "VALIDATE", "Test comfort, safety, fatigue, and daily use."),
]
for i, (number, title, detail) in enumerate(steps):
    top = Inches(2.75 + i * 0.88)
    add_text(
        s, Inches(0.8), top, Inches(0.62), Inches(0.38),
        [(number, 18, True, BLUE)],
    )
    add_text(
        s, Inches(1.55), top, Inches(1.35), Inches(0.36),
        [(title, 13, True, WHITE)],
    )
    add_text(
        s, Inches(2.92), top, Inches(4.15), Inches(0.42),
        [(detail, 13, False, MIST)],
        line_spacing=1.05,
    )
box(s, Inches(0.8), Inches(5.47), Inches(6.35), Inches(0.08), BLUE)
add_text(
    s, Inches(0.8), Inches(5.72), Inches(6.35), Inches(0.7),
    [("Extend movement into robotic reach.", 23, True, WHITE)],
)
add_text(
    s, Inches(0.8), Inches(6.33), Inches(6.35), Inches(0.3),
    [("Assistive use is a design direction and has not yet been user-validated.",
      10, False, MIST)],
)
add_picture(
    s, TUM_LOGO, Inches(0.78), Inches(6.82), Inches(0.72), Inches(0.34)
)
add_picture(
    s, MIRMI_LOGO, Inches(1.7), Inches(6.82), Inches(1.03), Inches(0.34)
)
add_text(
    s, Inches(2.95), Inches(6.79), Inches(4.2), Inches(0.4),
    [
        ("ARIES Lab | F. Masiero, E. Aimi, L. Borriello, Prof. L. Masia",
         8, False, MIST)
    ],
    align=PP_ALIGN.RIGHT,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[3:07-3:34]
This is not yet an assistive product, and the next step should not begin with
more software. It should begin with users: build a comfortable mount, co-design
the control language and useful tasks, then validate safety and fatigue. What
we have proved is the mechanism: a person's chosen movement can be extended
into robotic reach.
""")


output = os.path.join(HERE, "MagPilot_Keynote.pptx")
prs.save(output)
print("Wrote", output, "-", len(prs.slides), "slides")
