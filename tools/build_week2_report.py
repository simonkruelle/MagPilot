#!/usr/bin/env python3
"""Build the concise Week 2 meeting draft as Markdown, PDF and editable Word.

Run with a Python environment containing Matplotlib and Pillow. Reuses the
existing report's vector layout and standard-library Word helpers. Screenshots
are public repository assets; raw recordings and participant data are not read.
Optional --preview-dir writes review images outside the published report set.
"""

import argparse
from pathlib import Path
import re
import zipfile
from xml.etree import ElementTree

import matplotlib

matplotlib.use('Agg')

import build_lab_report as word
import build_lab_report_pdf as layout
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / 'docs' / 'reports'
NAME = 'MagPilot_Lab_Update_2026-10-12'
SUBTITLE = '5–11 October 2026 · meeting 12 October · draft as of 6 October'
TITLES = ('The teleoperation pipeline.', 'Board repaired. Pen printing.')
SECTIONS = (
    (
        ('Data pipeline', (
            'MuJoCo target-reaching lives in Data → Teleoperation Pipeline.',
            'Trackpad + board inputs; Enter starts; a stable 2 s hold saves.',
            'Participant progress; time, path and endpoint error logged.',
        )),
        ('Next checks', (
            'Validate board mapping with the assembled pen.',
            'Rehearse identical targets and start pose for both inputs.',
        )),
    ),
    (
        ('Magnet diagnosis', (
            'Board repaired; 1/2/3 magnets × 3 repeats completed.',
            'Approx. 5 mm cardboard; placements by hand.',
            'One magnet: practical surface baseline; no accuracy winner established.',
        )),
        ('Reloadable pen', (
            '1–5 disks; spacers above shorter stacks.',
            'PLA printing started; fit and assembled tracking untested.',
        )),
        ('Next checks', (
            'Check fit coupons and closure; measure pen geometry.',
            'Compare heights with a placement template, then freeze setup.',
        )),
    ),
)
CAPTIONS = (
    'Implemented GUI · participant timing comparison not collected.',
    'CAD prototype · print and tracking validation pending.',
)
ASSETS = (ROOT / 'docs' / 'teleoperation.png',
          ROOT / 'hardware' / 'magnetic_pen_v1' / 'preview.png')
BOTTOM_NOTES = (
    'Priority for 12 October: a repeatable target-reaching pilot.',
    'Surface tests do not validate operation at greater heights.',
)


def authored_words():
    passages = [SUBTITLE, *TITLES, *CAPTIONS, *BOTTOM_NOTES]
    for sections in SECTIONS:
        for title, items in sections:
            passages.extend((title, *items))
    return len(re.findall(r'\S+', ' '.join(passages)))


def markdown():
    lines = ['# Week 2 — MagPilot lab update', '',
             '**5–11 October 2026 · meeting 12 October · draft as of 6 October**', '',
             f'[Two-page PDF]({NAME}.pdf) · [Editable Word report]({NAME}.docx)', '']
    for index, title in enumerate(TITLES):
        lines.extend((f'## Week 2 — {title}', ''))
        for section, items in SECTIONS[index]:
            lines.extend((f'**{section}**', ''))
            lines.extend(f'- {item}' for item in items)
            lines.append('')
        asset = ('../teleoperation.png' if index == 0
                 else '../../hardware/magnetic_pen_v1/preview.png')
        lines.extend((f'![{CAPTIONS[index]}]({asset})', '', CAPTIONS[index], '',
                      BOTTOM_NOTES[index], ''))
    lines.extend(('Tracking accuracy requires measured placement references; '
                  'these hand-placed surface tests establish no accuracy ranking.', '',
                  '[Teleoperation protocol](../TELEOPERATION_COLLECTION.md) · '
                  '[Pen CAD and printing](../../hardware/magnetic_pen_v1/README.md)', ''))
    return '\n'.join(lines)


def pdf_page(index):
    fig = plt.figure(figsize=(layout.WIDTH / 72, layout.HEIGHT / 72), facecolor='white')
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, layout.WIDTH)
    ax.set_ylim(layout.HEIGHT, 0)
    ax.set_axis_off()
    layout.label(ax, 'W E E K  2   /   I N  P R O G R E S S', 44, 28,
                 size=8, colour=layout.BLUE, bold=True)
    layout.label(ax, TITLES[index], 44, 49, size=30, colour=layout.INK, bold=True)
    layout.label(ax, SUBTITLE, 44, 94, size=10, colour=layout.MUTED)
    width = 426
    bottom = layout.screenshot(ax, ASSETS[index], 52, 144, width)
    layout.paragraph(ax, CAPTIONS[index], 44, bottom + 12, width + 16,
                     size=9, line=12, colour=layout.MUTED)
    y = 136
    for title, items in SECTIONS[index]:
        y = layout.section(ax, title, items, 516, y, 280) + 17
    if y > 526:
        raise ValueError(f'Page {index + 1} text overlaps the note: {y}')
    layout.round_box(ax, 44, 539, 754, 27, radius=9, colour=layout.SURFACE)
    layout.label(ax, BOTTOM_NOTES[index], 58, 548, size=10, colour=layout.BODY)
    layout.label(ax, 'MagPilot · next review 12 October', 44, 578,
                 size=8, colour=layout.MUTED)
    layout.label(ax, f'{index + 1:02d} / 02', 798, 578, size=8,
                 colour=layout.MUTED, align='right')
    return fig


def render_pdf(output, preview_dir):
    with PdfPages(output, metadata={
        'Title': 'MagPilot · Week 2 · meeting draft for 12 October 2026',
        'Author': 'MagPilot', 'Subject': 'Progress as of 6 October; week 5–11 October',
    }) as pdf:
        for index in range(2):
            fig = pdf_page(index)
            pdf.savefig(fig)
            if preview_dir:
                preview_dir.mkdir(parents=True, exist_ok=True)
                fig.savefig(preview_dir / f'week2_slide_{index + 1}.png', dpi=150)
            plt.close(fig)


def word_xml():
    body = ''
    for index, title in enumerate(TITLES):
        if index:
            body += '<w:p><w:r><w:br w:type="page"/></w:r></w:p>'
        body += word.paragraph('WEEK 2 · IN PROGRESS', size=18, bold=True,
                               colour='007AFF', line=200, after=45, keep_next=True)
        body += word.paragraph(title, size=40, bold=True, colour='1D1D1F',
                               line=460, after=45, keep_next=True)
        body += word.paragraph(SUBTITLE, size=20, colour='86868B',
                               line=240, after=120, keep_next=True)
        left = word.image_paragraph(ASSETS[index], f'rIdImage{index}', index + 1,
                                    4.65, 4.9, CAPTIONS[index])
        left += word.caption(CAPTIONS[index])
        right = ''
        for heading, items in SECTIONS[index]:
            right += word.heading(heading) + word.bullets(items)
        body += word.two_columns(left, right)
        body += word.paragraph(BOTTOM_NOTES[index], size=20, colour='546675',
                               before=140, line=240)
    body += '<w:sectPr><w:footerReference w:type="default" r:id="rIdFooter"/>' \
            '<w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/>' \
            '<w:pgMar w:top="565" w:right="565" w:bottom="565" w:left="565" ' \
            'w:header="180" w:footer="240" w:gutter="0"/></w:sectPr>'
    return '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' \
           '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" ' \
           'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" ' \
           'xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" ' \
           'xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" ' \
           'xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">' \
           f'<w:body>{body}</w:body></w:document>'


def build_word(output):
    overrides = {
        'word/document.xml': 'wordprocessingml.document.main',
        'word/styles.xml': 'wordprocessingml.styles',
        'word/numbering.xml': 'wordprocessingml.numbering',
        'word/footer1.xml': 'wordprocessingml.footer',
    }
    content_types = '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">' \
                    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>' \
                    '<Default Extension="png" ContentType="image/png"/>' \
                    '<Default Extension="xml" ContentType="application/xml"/>'
    for name, kind in overrides.items():
        content_types += f'<Override PartName="/{name}" ContentType="application/vnd.openxmlformats-officedocument.{kind}+xml"/>'
    content_types += '</Types>'
    package_rels = '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' \
                   '<Relationship Id="rIdDocument" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>' \
                   '</Relationships>'
    records = [(f'rIdImage{index}', 'image', f'media/{path.name}', False)
               for index, path in enumerate(ASSETS)]
    records.extend((('rIdStyles', 'styles', 'styles.xml', False),
                    ('rIdNumbering', 'numbering', 'numbering.xml', False),
                    ('rIdFooter', 'footer', 'footer1.xml', False)))
    parts = {'[Content_Types].xml': content_types, '_rels/.rels': package_rels,
             'word/document.xml': word_xml(),
             'word/_rels/document.xml.rels': word.relationships(records),
             'word/styles.xml': word.STYLES, 'word/numbering.xml': word.NUMBERING,
             'word/footer1.xml': word.FOOTER}
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as package:
        for name, xml in parts.items():
            ElementTree.fromstring(xml)
            package.writestr(name, xml.encode('utf-8'))
        for path in ASSETS:
            package.write(path, 'word/media/' + path.name)
    with zipfile.ZipFile(output) as package:
        if package.testzip() is not None:
            raise ValueError('Word package CRC check failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=REPORT_DIR)
    parser.add_argument('--preview-dir', type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for path in ASSETS:
        word.png_size(path)
    if authored_words() > 230:
        raise ValueError(f'Report exceeds 230 authored words: {authored_words()}')
    (args.output_dir / f'{NAME}.md').write_text(markdown(), encoding='utf-8')
    render_pdf(args.output_dir / f'{NAME}.pdf', args.preview_dir)
    build_word(args.output_dir / f'{NAME}.docx')
    print(f'{NAME}: two slides, {authored_words()} authored words; PDF/Word/Markdown written.')


if __name__ == '__main__':
    main()
