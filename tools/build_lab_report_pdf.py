#!/usr/bin/env python3
"""Render a two-page, presentation-style PDF from the existing lab report.

Uses Matplotlib and the original GUI screenshots. Authored content comes from
build_lab_report.py, so the concise Word handout and PDF share the same facts.
Text, cards and the timeline remain vector graphics; screenshots keep their
original resolution and aspect ratio. Optional PNG previews aid layout review.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle, FancyBboxPatch
from matplotlib.textpath import TextToPath

import build_lab_report as content


WIDTH, HEIGHT = 841.89, 595.28  # A4 landscape, in points.
MARGIN = 44
FONT = 'Liberation Sans'
INK = '#1D1D1F'
BODY = '#515155'
MUTED = '#86868B'
BLUE = '#007AFF'
PALE_BLUE = '#EDF5FF'
SURFACE = '#F5F5F7'
MEASURE = TextToPath()

matplotlib.rcParams.update({
    'font.family': FONT,
    'pdf.fonttype': 42,
    'pdf.compression': 9,
    'savefig.facecolor': 'white',
})


def text_width(text, size, bold=False):
    font = FontProperties(family=FONT, size=size,
                          weight='bold' if bold else 'normal')
    return MEASURE.get_text_width_height_descent(text, font, False)[0]


def wrap(text, width, size, bold=False):
    lines, current = [], ''
    for word in text.split():
        candidate = '{} {}'.format(current, word).strip()
        if current and text_width(candidate, size, bold) > width:
            lines.append(current)
            current = word
        else:
            current = candidate
    return lines + [current]


def label(ax, text, x, y, size=11, colour=BODY, bold=False, align='left',
          url=None):
    return ax.text(x, y, text, fontsize=size, color=colour, family=FONT,
                   fontweight='bold' if bold else 'normal', va='top', ha=align,
                   url=url, zorder=5)


def paragraph(ax, text, x, y, width, size=11, line=14.5, colour=BODY,
              bold=False):
    lines = wrap(text, width, size, bold)
    for number, text_line in enumerate(lines):
        label(ax, text_line, x, y + number * line, size, colour, bold)
    return y + len(lines) * line


def round_box(ax, x, y, width, height, radius=12, colour=SURFACE,
              edge='none', alpha=1, zorder=1):
    box = FancyBboxPatch((x, y), width, height,
                         boxstyle='round,pad=0,rounding_size={}'.format(radius),
                         facecolor=colour, edgecolor=edge, linewidth=.55,
                         alpha=alpha, zorder=zorder)
    ax.add_patch(box)
    return box


def screenshot(ax, path, x, y, width):
    image = plt.imread(path)
    height = width * image.shape[0] / image.shape[1]
    # Quiet, layered shadow and a white frame; the screenshot itself is intact.
    for offset, alpha in ((4, .018), (2, .025), (1, .03)):
        round_box(ax, x - 8, y - 8 + offset, width + 16, height + 16,
                  radius=13, colour=INK, alpha=alpha)
    round_box(ax, x - 8, y - 8, width + 16, height + 16,
              radius=13, colour='white', edge='#E5E5EA', zorder=2)
    ax.imshow(image, extent=(x, x + width, y + height, y),
              interpolation='none', aspect='auto', zorder=3)
    return y + height + 8


def page(number, title, subtitle=None):
    fig = plt.figure(figsize=(WIDTH / 72, HEIGHT / 72), facecolor='white')
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(HEIGHT, 0)
    ax.set_axis_off()
    week_caption = ' '.join(content.WEEK_LABEL.upper())
    label(ax, week_caption + '   /   L A B   U P D A T E',
          MARGIN, 28, 8, BLUE, True)
    label(ax, title, MARGIN, 49, 31.5, INK, True)
    if subtitle:
        label(ax, subtitle, MARGIN, 94, 10, MUTED)
    label(ax, 'MagPilot', MARGIN, 578, 8, MUTED)
    label(ax, '{:02d} / 02'.format(number), WIDTH - MARGIN, 578, 8, MUTED,
          align='right')
    return fig, ax


def section(ax, title, items, x, y, width):
    label(ax, title, x, y, 13.5, INK, True)
    y += 24
    for item in items:
        ax.add_patch(Circle((x + 1.8, y + 6.5), 1.45,
                            facecolor=BLUE, edgecolor='none', zorder=5))
        y = paragraph(ax, item, x + 12, y, width - 12) + 4
    return y


def timeline(ax):
    paragraph(ax, content.TIMELINE_TITLE, MARGIN, 463, 145,
              size=13.5, line=17, colour=INK, bold=True)
    label(ax, 'Relative weeks', MARGIN, 505, 9, MUTED)
    phases = content.PROJECT_PHASES
    chart_x, chart_width = 382, WIDTH - MARGIN - 382
    week_width = chart_width / 10
    for week in range(10):
        x = chart_x + (week + .5) * week_width
        label(ax, 'W{}'.format(week + 1), x, 461, 8, MUTED, align='center')
        ax.plot([x, x], [479, 547], color='#EDEDF0', linewidth=.65, zorder=0)
    for index, (name, start, end) in enumerate(phases):
        y = 482 + index * 9.7
        label(ax, name, chart_x - 14, y - 1.5, 8.25, BODY, align='right')
        round_box(ax, chart_x + (start - 1) * week_width + 3, y,
                  (end - start + 1) * week_width - 6, 5.5, radius=2.75,
                  colour=BLUE if index < 2 else '#BFC7D2')


def overview(assets):
    fig, ax = page(1, content.PAGE1_TITLE, content.SUBTITLE)
    bottom = screenshot(ax, assets / 'launcher_data_demo.png', MARGIN + 8,
                        134, 274)
    paragraph(ax, content.DATA_CAPTION, MARGIN, bottom + 10, 294,
              size=8.5, line=11, colour=MUTED)
    x, width = 364, WIDTH - MARGIN - 364
    y = section(ax, 'Collection foundations', content.UPDATES, x, 128, width)
    y = section(ax, content.HARDWARE_TITLE, content.PEN_QUESTIONS, x, y + 10, width)
    y = section(ax, 'Next milestones', content.NEXT_STEPS, x, y + 10, width)
    if y > 449:
        raise ValueError('Overview text overlaps the timeline: {}'.format(y))
    ax.plot([MARGIN, WIDTH - MARGIN], [450, 450], color='#E5E5EA', linewidth=.6)
    timeline(ax)
    label(ax, 'Reference:', MARGIN, 560, 8, MUTED)
    label(ax, content.REFERENCE_TEXT, MARGIN + 40, 560, 8, BLUE,
          url=content.KJ_REFERENCE)
    return fig


def capture(assets):
    fig, ax = page(2, content.PAGE2_TITLE)
    steps = [step.strip() for step in content.FLOW.split('→')]
    gap = 17
    width = (WIDTH - 2 * MARGIN - gap * (len(steps) - 1)) / len(steps)
    for index, step in enumerate(steps):
        x = MARGIN + index * (width + gap)
        operator = 'operator Enter' in step
        round_box(ax, x, 100, width, 28, radius=9,
                  colour=PALE_BLUE if operator else SURFACE)
        label(ax, step, x + width / 2, 108.5, 9.7,
              BLUE if operator else BODY, align='center')
        if index < len(steps) - 1:
            label(ax, '›', x + width + gap / 2, 106.5, 14, MUTED,
                  align='center')
    screenshot_width = 624
    bottom = screenshot(ax, assets / 'collection_demo.png',
                        (WIDTH - screenshot_width) / 2, 141, screenshot_width)
    label(ax, content.COLLECTION_CAPTION, WIDTH / 2, bottom + 9, 8.5, MUTED,
          align='center')
    # Two compact columns preserve every existing note while adding breathing
    # room around the screenshot and avoiding a dense four-line bullet stack.
    column_width = (WIDTH - 2 * MARGIN - 32) / 2
    for column, items in enumerate((content.CAPTURE_NOTES[:2],
                                  content.CAPTURE_NOTES[2:])):
        x = MARGIN + column * (column_width + 32)
        y = 529
        for item in items:
            ax.add_patch(Circle((x + 1.8, y + 6), 1.4,
                                facecolor=BLUE, edgecolor='none', zorder=5))
            y = paragraph(ax, item, x + 12, y, column_width - 12,
                          size=10.25, line=12.5) + 5
        if y > 578:
            raise ValueError('Capture notes overlap the footer: {}'.format(y))
    return fig


def render_timeline_asset(output):
    """Keep the Word and GitHub timeline aligned with the PDF's phase data."""
    fig, ax = plt.subplots(figsize=(10, 3.65), facecolor='white')
    fig.subplots_adjust(left=.32, right=.985, top=.83, bottom=.13)
    ax.set_xlim(.5, 10.5)
    ax.set_ylim(6.5, -.5)
    ax.set_xticks(range(1, 11), ['W{}'.format(week) for week in range(1, 11)])
    ax.set_yticks(range(len(content.PROJECT_PHASES)),
                  [phase[0] for phase in content.PROJECT_PHASES])
    ax.tick_params(axis='both', length=0, labelsize=10, labelcolor=BODY, pad=10)
    ax.grid(axis='x', color='#EDEDF0', linewidth=.7)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_visible(False)
    for index, (_, start, end) in enumerate(content.PROJECT_PHASES):
        ax.add_patch(FancyBboxPatch(
            (start - .42, index - .17), end - start + .84, .34,
            boxstyle='round,pad=0,rounding_size=.10',
            facecolor=BLUE if index < 2 else '#BFC7D2', edgecolor='none',
            zorder=3))
    fig.text(.025, .94, 'Ten-week plan · Week 2 starts 5 October',
             fontsize=15, fontweight='bold', color=INK, va='top')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-dir', type=Path,
                        default=content.REPORT_DIR / 'assets')
    parser.add_argument('--output', type=Path,
                        default=content.REPORT_DIR / (content.REPORT_NAME + '.pdf'))
    parser.add_argument('--preview-dir', type=Path)
    parser.add_argument('--refresh-timeline-asset', action='store_true',
                        help='Update the shared timeline PNG for Word/GitHub.')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.preview_dir:
        args.preview_dir.mkdir(parents=True, exist_ok=True)
    if args.refresh_timeline_asset:
        render_timeline_asset(args.asset_dir / 'project_timeline.png')
    figures = (overview(args.asset_dir), capture(args.asset_dir))
    metadata = {
        'Title': 'MagPilot lab update — 5 October 2026',
        'Author': 'MagPilot',
        'Subject': 'Week 1 character collection and Week 2 handoff',
    }
    with PdfPages(args.output, metadata=metadata) as pdf:
        for number, fig in enumerate(figures, 1):
            pdf.savefig(fig, dpi=200)
            if args.preview_dir:
                fig.savefig(args.preview_dir / 'page-{}.png'.format(number),
                            dpi=160)
            plt.close(fig)
    print('{} ({} authored words; two landscape pages)'.format(
        args.output, content.report_word_count()))


if __name__ == '__main__':
    main()
