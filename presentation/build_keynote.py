#!/usr/bin/env python3
"""Generate the short MagPilot assistive-robotics keynote.

The deck is deliberately concise: eight slides, about 137 seconds of spoken
material, and 68 seconds of embedded demo video. At a calm pace it runs for
roughly 3:25, leaving room for pauses while staying below five minutes.

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


def add_movie(s, movie_path, poster_path, left=0, top=0, width=SW, height=SH):
    return s.shapes.add_movie(
        movie_path,
        left,
        top,
        width,
        height,
        poster_frame_image=poster_path,
        mime_type="video/mp4",
    )


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
UI = os.path.join(DOCS, "magpilot.png")
LOGO = os.path.join(ASSETS, "logo.png")
TUM_LOGO = os.path.join(ASSETS, "tum-logo.png")
MIRMI_LOGO = os.path.join(DOCS, "affiliations", "mirmi-logo.png")
VIDEO_COMMAND = os.path.join(ASSETS, "demo_command.mp4")
VIDEO_PICK = os.path.join(ASSETS, "demo_pick.mp4")
POSTER_COMMAND = os.path.join(ASSETS, "poster_command.jpg")
POSTER_PICK = os.path.join(ASSETS, "poster_pick.jpg")


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
kicker(s, "The idea", dark=True)
add_text(
    s, Inches(0.78), Inches(1.03), Inches(5.55), Inches(1.7),
    [
        ("Put the input where", 35, False, WHITE),
        ("reliable movement remains.", 35, True, BLUE),
    ],
    space_after=2,
)
add_text(
    s, Inches(0.8), Inches(2.95), Inches(5.0), Inches(1.4),
    [
        ("A small magnet could sit on a cuff or residual limb. The sensor board "
         "reads its movement through the air.", 19, False, MIST)
    ],
    line_spacing=1.12,
)
magnet = s.shapes.add_shape(
    MSO_SHAPE.OVAL, Inches(0.85), Inches(4.72), Inches(1.18), Inches(1.18)
)
magnet.fill.solid()
magnet.fill.fore_color.rgb = BLUE
magnet.line.fill.background()
magnet.shadow.inherit = False
tf = magnet.text_frame
tf.clear()
tf.vertical_anchor = MSO_ANCHOR.MIDDLE
p = tf.paragraphs[0]
p.alignment = PP_ALIGN.CENTER
r = p.add_run()
r.text = "N\nS"
r.font.name = FONT
r.font.size = Pt(19)
r.font.bold = True
r.font.color.rgb = WHITE
add_text(
    s, Inches(2.3), Inches(4.63), Inches(3.6), Inches(1.35),
    [
        ("NO BATTERY", 14, True, BLUE),
        ("NO BUTTONS", 14, True, BLUE),
        ("NO GRIP REQUIRED", 14, True, BLUE),
    ],
    space_after=7,
)
box(s, Inches(6.48), Inches(0.58), Inches(6.25), Inches(6.28), WHITE, radius=True)
add_picture(
    s, SENSOR, Inches(6.7), Inches(0.78), Inches(5.82), Inches(5.82)
)
add_text(
    s, Inches(6.75), Inches(6.26), Inches(5.7), Inches(0.35),
    [("16 x three-axis MLX90393 magnetometers", 12, True, SUBTLE)],
    align=PP_ALIGN.CENTER,
)
footer(s, 3, dark=True)
add_notes(s, """
[0:40-1:00]
The proposed assistive form is simple: mount a small passive magnet wherever a
person has comfortable, repeatable movement, for example on a cuff or residual
limb. Nothing needs charging, gripping, or pressing. A flat sensor array turns
that movement into robot commands.
""")


# 4. Control language
s = make_slide(SKY)
kicker(s, "The control language")
add_text(
    s, Inches(0.78), Inches(1.02), Inches(5.35), Inches(1.0),
    [("One magnet. Four direct actions.", 32, True, INK)],
)
controls = [
    ("MOVE", "30 x 60 cm X/Y workspace"),
    ("RAISE", "56 cm vertical travel"),
    ("TILT", "open or close the gripper"),
    ("LIFT AWAY", "pause and hold safely"),
]
for i, (title, detail) in enumerate(controls):
    top = Inches(2.18 + i * 1.05)
    box(s, Inches(0.82), top + Inches(0.04), Inches(0.10), Inches(0.72), BLUE)
    add_text(
        s, Inches(1.17), top, Inches(1.48), Inches(0.75),
        [(title, 15, True, BLUE)],
        anchor=MSO_ANCHOR.MIDDLE,
    )
    add_text(
        s, Inches(2.58), top, Inches(3.35), Inches(0.75),
        [(detail, 17, False, INK)],
        anchor=MSO_ANCHOR.MIDDLE,
    )
box(s, Inches(6.28), Inches(0.75), Inches(6.42), Inches(5.98), WHITE, radius=True)
add_picture(
    s, UI, Inches(6.5), Inches(0.98), Inches(5.98), Inches(5.52)
)
add_text(
    s, Inches(6.45), Inches(6.35), Inches(6.0), Inches(0.3),
    [("The live interface shows position, height, tilt, and gripper state.",
      11, False, SUBTLE)],
    align=PP_ALIGN.CENTER,
)
footer(s, 4)
add_notes(s, """
[1:00-1:20]
The mapping stays physical: move the magnet and the arm moves in X and Y;
raise it and the arm rises; tilt it to control the gripper; lift it away and
the robot pauses. The interface exposes a bounded workspace and makes the
current state visible.
""")


# 5. Embedded command demo
s = make_slide(NAVY)
add_movie(s, VIDEO_COMMAND, POSTER_COMMAND)
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
    s, Inches(10.72), Inches(0.23), Inches(1.88), Inches(0.52), BLUE, radius=True
)
add_text(
    s, Inches(10.72), Inches(0.23), Inches(1.88), Inches(0.52),
    [("CLICK TO PLAY  0:20", 10, True, WHITE)],
    align=PP_ALIGN.CENTER,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[1:20-1:45]
This is the current hand-held laboratory proof. One air-written A becomes a
discrete robot command. Click the video and let the 20-second clip play. Do not
talk over it unless the room needs a short explanation.
""")


# 6. Embedded pick demo
s = make_slide(NAVY)
add_movie(s, VIDEO_PICK, POSTER_PICK)
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
    s, Inches(10.72), Inches(0.23), Inches(1.88), Inches(0.52), BLUE, radius=True
)
add_text(
    s, Inches(10.72), Inches(0.23), Inches(1.88), Inches(0.52),
    [("CLICK TO PLAY  0:48", 10, True, WHITE)],
    align=PP_ALIGN.CENTER,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[1:45-2:38]
Now the continuous mode: move, lower, close the gripper, and lift the package.
Click the video and let the 48-second clip play. This is the central proof that
the magnetic input can perform a useful physical task on a real robot.
""")


# 7. What is working today
s = make_slide(NAVY)
add_cover(s, SETUP, 0, 0, SW, SH)
panel = box(s, 0, 0, Inches(5.25), SH, NAVY)
set_opacity(panel, 0.92)
kicker(s, "What works today", dark=True, left=Inches(0.68), top=Inches(0.58))
add_text(
    s, Inches(0.68), Inches(1.12), Inches(4.15), Inches(1.55),
    [
        ("Not a simulation.", 34, False, WHITE),
        ("A working real-robot system.", 34, True, WHITE),
    ],
    space_after=3,
)
metric(s, Inches(0.72), Inches(3.08), "16", "three-axis magnetic sensors", True)
metric(s, Inches(0.72), Inches(4.25), "30 Hz", "filtered Cartesian control", True)
metric(s, Inches(0.72), Inches(5.42), "30 x 60 x 56 cm", "bounded robot workspace", True)
caption_band = box(
    s, Inches(7.15), Inches(6.47), Inches(5.58), Inches(0.56), NAVY, radius=True
)
set_opacity(caption_band, 0.84)
add_text(
    s, Inches(7.32), Inches(6.54), Inches(5.18), Inches(0.38),
    [
        ("Real FR3 | jerk-limited motion | one-window control center",
         12, True, WHITE)
    ],
    align=PP_ALIGN.RIGHT,
)
add_notes(s, """
[2:38-2:58]
What is already real: sixteen three-axis sensors, a 30-hertz filtered control
loop, a bounded workspace, jerk-limited motion, and one control center driving
the full FR3 pipeline. The assistive mount is future work, but the sensing and
robot-control mechanism is working now.
""")


# 8. Honest next step and close
s = make_slide(NAVY)
kicker(s, "The next step", dark=True)
add_text(
    s, Inches(0.78), Inches(1.03), Inches(11.7), Inches(1.45),
    [
        ("The next prototype starts", 38, False, WHITE),
        ("with the people who would use it.", 38, True, BLUE),
    ],
    space_after=2,
)
steps = [
    ("1", "MOUNT", "Build a light, adjustable cuff or residual-limb carrier."),
    ("2", "CO-DESIGN", "Choose movements and tasks with users, not for them."),
    ("3", "VALIDATE", "Test comfort, safety, fatigue, and useful daily tasks."),
]
for i, (number, title, detail) in enumerate(steps):
    left = Inches(0.8 + i * 4.12)
    box(s, left, Inches(2.82), Inches(3.55), Inches(0.08), BLUE)
    add_text(
        s, left, Inches(3.16), Inches(0.52), Inches(0.52),
        [(number, 26, True, BLUE)],
    )
    add_text(
        s, left + Inches(0.58), Inches(3.2), Inches(2.95), Inches(0.42),
        [(title, 14, True, WHITE)],
    )
    add_text(
        s, left, Inches(3.86), Inches(3.55), Inches(1.15),
        [(detail, 16, False, MIST)],
        line_spacing=1.1,
    )
add_text(
    s, Inches(0.8), Inches(5.32), Inches(11.7), Inches(0.75),
    [("Extend a person's movement into robotic reach.", 30, True, WHITE)],
    align=PP_ALIGN.CENTER,
)
add_text(
    s, Inches(0.8), Inches(6.05), Inches(11.7), Inches(0.35),
    [("Assistive use is a design direction and has not yet been user-validated.",
      11, False, MIST)],
    align=PP_ALIGN.CENTER,
)
add_picture(
    s, TUM_LOGO, Inches(0.78), Inches(6.63), Inches(0.72), Inches(0.38)
)
add_picture(
    s, MIRMI_LOGO, Inches(1.72), Inches(6.63), Inches(1.15), Inches(0.38)
)
add_text(
    s, Inches(3.2), Inches(6.62), Inches(9.25), Inches(0.42),
    [
        ("ARIES Lab | F. Masiero, E. Aimi, L. Borriello, Prof. L. Masia",
         10, False, MIST)
    ],
    align=PP_ALIGN.RIGHT,
    anchor=MSO_ANCHOR.MIDDLE,
)
add_notes(s, """
[2:58-3:25]
This is not yet an assistive product, and the next step should not begin with
more software. It should begin with users: build a comfortable mount, co-design
the control language and useful tasks, then validate safety and fatigue. What
we have proved is the mechanism: a person's chosen movement can be extended
into robotic reach.
""")


output = os.path.join(HERE, "MagPilot_Keynote.pptx")
prs.save(output)
print("Wrote", output, "-", len(prs.slides), "slides")
