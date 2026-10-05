#!/usr/bin/env python3
"""Build the two-page lab handout using only Python's standard library.

PNG inputs are screenshots and a timeline prepared separately. Their aspect
ratios are preserved. No office package, GUI, or network connection is needed
to create the editable Word document. The presentation-style PDF is rendered
separately by build_lab_report_pdf.py using the same authored content.
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import re
import struct
from xml.sax.saxutils import escape, quoteattr
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = REPO_ROOT / 'docs' / 'reports'
REPORT_NAME = 'MagPilot_Lab_Update_2026-10-05'
FONT = 'Liberation Sans'
KJ_REFERENCE = 'https://www.kjmagnetics.com/blog/magnetic-dipole-moment'
WEEK_LABEL = 'Week 1'

PAGE1_TITLE = 'MagPilot | Lab update'
SUBTITLE = '5 October 2026 · meeting update · paper branch'
UPDATES = (
    'Protocol: 0–9 / A–J; ten takes each (200 per participant).',
    'Participant names and stable IDs; experiment names accompany takes.',
    'Enter marks capture boundaries; CSV/PNG/JSON use frozen rows.',
    'Separate demo files/counts; software checks + GUI rehearsal passed.',
)
PEN_QUESTIONS = (
    'Sensor board repaired; three smaller magnets available for experiments.',
    'Compare one-, two- and three-magnet configurations; document geometry, grade and orientation.',
    'Measure signal/clipping at the writing gap and teleoperation range.',
)
HARDWARE_TITLE = 'Hardware update'
NEXT_STEPS = (
    'Start the virtual robot-arm target-reaching pilot now; compare completion times and endpoint errors.',
    'Character pilot: 40–60 takes/person (2–3 repetitions); myself + 1–2 colleagues. Not collected.',
    'Next: review/resume, preprocessing and participant-separated splits.',
)
DATA_CAPTION = 'DEMO · fictitious participant names; synthetic mouse input.'
TIMELINE_TITLE = 'Proposed ten-week plan'
REFERENCE_TEXT = 'K&J Magnetics: magnetic moment / geometry'
PAGE2_TITLE = 'Explicit one-character capture'
FLOW = 'Select label → operator Enter: start → participant draws → operator Enter: stop/save → inspect.'
COLLECTION_CAPTION = 'Actual GUI rehearsal: synthetic A; one of ten takes saved.'
CAPTURE_NOTES = (
    'Operator uses Enter for recording and demo ink; participant only moves the mouse.',
    'Demo mouse input; no sensor board needed.',
    'Stop freezes rows before matched artifacts are saved.',
    'Hardware pilot and new-pen validation remain pending.',
)
PROJECT_PHASES = (
    ('Virtual task + pilot', 1, 3),
    ('Hardware + recording', 1, 2),
    ('Pilot + protocol freeze', 3, 3),
    ('Full collection + preprocessing', 4, 6),
    ('Models + tracking analysis', 5, 7),
    ('Participant evaluation', 9, 9),
    ('Analysis + report / buffer', 10, 10),
)


def report_word_count():
    """Count authored document text (screenshots and diagram labels excluded)."""
    passages = [WEEK_LABEL, PAGE1_TITLE, SUBTITLE, 'Collection foundations', *UPDATES,
                HARDWARE_TITLE, *PEN_QUESTIONS, 'Next milestones', *NEXT_STEPS,
                DATA_CAPTION, TIMELINE_TITLE, 'Reference:', REFERENCE_TEXT,
                WEEK_LABEL, PAGE2_TITLE, FLOW, COLLECTION_CAPTION, *CAPTURE_NOTES]
    return len(re.findall(r'\S+', ' '.join(passages)))


def run(text, size=22, bold=False, colour=None):
    props = '<w:rFonts w:ascii="{0}" w:hAnsi="{0}"/><w:sz w:val="{1}"/>'.format(
        FONT, size)
    if bold:
        props += '<w:b/>'
    if colour:
        props += '<w:color w:val="{}"/>'.format(colour)
    return '<w:r><w:rPr>{}</w:rPr><w:t xml:space="preserve">{}</w:t></w:r>'.format(
        props, escape(text))


def paragraph(text='', size=22, bold=False, colour=None, before=0, after=45,
              line=264, keep_next=False, align=None, bullet=False, raw=None):
    props = '<w:spacing w:before="{}" w:after="{}" w:line="{}" w:lineRule="exact"/>'.format(
        before, after, line)
    props += '<w:keepLines/>'
    if keep_next:
        props += '<w:keepNext/>'
    if align:
        props += '<w:jc w:val="{}"/>'.format(align)
    if bullet:
        props += '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="1"/></w:numPr>'
    return '<w:p><w:pPr>{}</w:pPr>{}</w:p>'.format(
        props, raw if raw is not None else run(text, size, bold, colour))


def heading(text):
    return paragraph(text, bold=True, colour='174B65', before=70, after=40,
                     line=264, keep_next=True)


def bullets(items):
    return ''.join(paragraph(item, bullet=True, after=35) for item in items)


def png_size(path):
    with path.open('rb') as source:
        header = source.read(24)
    if header[:8] != b'\x89PNG\r\n\x1a\n' or header[12:16] != b'IHDR':
        raise ValueError('{} is not a PNG image'.format(path))
    return struct.unpack('>II', header[16:24])


def image_paragraph(path, relationship_id, drawing_id, width_in, height_in,
                    description, after=30):
    width, height = png_size(path)
    scale = min(width_in / width, height_in / height)
    cx = round(width * scale * 914400)
    cy = round(height * scale * 914400)
    image_xml = '''<w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0">
<wp:extent cx="{cx}" cy="{cy}"/><wp:effectExtent l="0" t="0" r="0" b="0"/>
<wp:docPr id="{drawing_id}" name={name} descr={description}/>
<wp:cNvGraphicFramePr><a:graphicFrameLocks noChangeAspect="1"/></wp:cNvGraphicFramePr>
<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:pic><pic:nvPicPr><pic:cNvPr id="0" name={name}/><pic:cNvPicPr/></pic:nvPicPr>
<pic:blipFill><a:blip r:embed="{relationship_id}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>
<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm>
<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr></pic:pic>
</a:graphicData></a:graphic></wp:inline></w:drawing></w:r>'''.format(
        cx=cx, cy=cy, drawing_id=drawing_id, name=quoteattr(path.name),
        description=quoteattr(description), relationship_id=relationship_id)
    # Automatic line spacing lets inline drawings use their actual height.
    return '<w:p><w:pPr><w:spacing w:after="{}"/><w:jc w:val="center"/></w:pPr>{}</w:p>'.format(
        after, image_xml)


def caption(text):
    return paragraph(text, size=20, colour='546675', line=240, after=35)


def two_columns(left, right):
    # The no-split row and borderless fixed table keep the two page-one panels
    # aligned while allowing independent paragraph flow inside each cell.
    def cell(body, width, margin):
        return '<w:tc><w:tcPr><w:tcW w:w="{0}" w:type="dxa"/>' \
               '<w:tcMar><w:left w:w="{1}" w:type="dxa"/>' \
               '<w:right w:w="{1}" w:type="dxa"/></w:tcMar>' \
               '<w:vAlign w:val="top"/></w:tcPr>{2}</w:tc>'.format(width, margin, body)
    return '<w:tbl><w:tblPr><w:tblW w:w="15708" w:type="dxa"/>' \
           '<w:tblLayout w:type="fixed"/><w:tblBorders>' \
           '<w:top w:val="nil"/><w:left w:val="nil"/><w:bottom w:val="nil"/>' \
           '<w:right w:val="nil"/><w:insideH w:val="nil"/><w:insideV w:val="nil"/>' \
           '</w:tblBorders></w:tblPr><w:tblGrid><w:gridCol w:w="7200"/>' \
           '<w:gridCol w:w="8508"/></w:tblGrid><w:tr><w:trPr><w:cantSplit/></w:trPr>' + \
           cell(left, 7200, 100) + cell(right, 8508, 100) + '</w:tr></w:tbl>'


def document_xml(assets):
    left = image_paragraph(
        assets['launcher_data_demo'], 'rIdData', 1, 4.65, 5.6,
        'MagPilot launcher Data window in DEMO mode with three fictitious named '
        'participants, experiment name, and coverage counts separate from sensor data.')
    left += caption(DATA_CAPTION)
    right = heading('Collection foundations') + bullets(UPDATES)
    right += heading(HARDWARE_TITLE) + bullets(PEN_QUESTIONS)
    right += heading('Next milestones') + bullets(NEXT_STEPS)
    right += heading(TIMELINE_TITLE)
    right += image_paragraph(
        assets['project_timeline'], 'rIdTimeline', 2, 5.35, 2.3,
        'Proposed weeks 1 through 10: virtual target-reaching task and pilot start '
        'in week one alongside hardware and recording; setup freeze, formal '
        'collection, model and tracking analysis, evaluation, final report and buffer.')
    reference = run('Reference: ', size=20, colour='546675')
    reference += '<w:hyperlink r:id="rIdKJ"><w:r><w:rPr><w:rStyle w:val="Hyperlink"/>' \
                 '<w:sz w:val="20"/></w:rPr><w:t>{}</w:t></w:r></w:hyperlink>'.format(
                     escape(REFERENCE_TEXT))
    right += paragraph(raw=reference, line=240, after=0)
    body = paragraph(WEEK_LABEL, size=18, bold=True, colour='186B91',
                     line=200, after=45, keep_next=True)
    body += paragraph(PAGE1_TITLE, size=40, bold=True, colour='123C54',
                     line=460, after=45, keep_next=True)
    body += paragraph(SUBTITLE, size=22, colour='546675', after=100, keep_next=True)
    body += two_columns(left, right)
    body += '<w:p><w:pPr><w:spacing w:before="0" w:after="0" w:line="20" ' \
            'w:lineRule="exact"/></w:pPr><w:r><w:br w:type="page"/></w:r></w:p>'
    body += paragraph(WEEK_LABEL, size=18, bold=True, colour='186B91',
                      line=200, after=45, keep_next=True)
    body += paragraph(PAGE2_TITLE, size=40, bold=True, colour='123C54',
                      line=460, after=55, keep_next=True)
    body += paragraph(FLOW, after=55, keep_next=True)
    body += image_paragraph(
        assets['collection_demo'], 'rIdCollection', 3, 10.75, 5.7,
        'Actual MagPilot collection GUI using synthetic mouse input: a visible '
        'letter A, operator-controlled Enter start and stop for recording and demo '
        'ink, and one of ten saved takes; participant mouse movement requires no held key.')
    body += caption(COLLECTION_CAPTION)
    body += bullets(CAPTURE_NOTES)
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
           '<w:body>{}</w:body></w:document>'.format(body)


STYLES = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Liberation Sans" w:hAnsi="Liberation Sans"/>
<w:sz w:val="22"/></w:rPr></w:rPrDefault><w:pPrDefault><w:pPr><w:spacing w:after="40"/>
</w:pPr></w:pPrDefault></w:docDefaults><w:style w:type="paragraph" w:default="1" w:styleId="Normal">
<w:name w:val="Normal"/></w:style><w:style w:type="character" w:styleId="Hyperlink">
<w:name w:val="Hyperlink"/><w:rPr><w:color w:val="186B91"/><w:u w:val="single"/>
</w:rPr></w:style></w:styles>'''

NUMBERING = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:numbering xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:abstractNum w:abstractNumId="0"><w:multiLevelType w:val="singleLevel"/>
<w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/>
<w:lvlJc w:val="left"/><w:pPr><w:tabs><w:tab w:val="num" w:pos="210"/></w:tabs>
<w:ind w:left="210" w:hanging="210"/></w:pPr><w:rPr><w:rFonts w:ascii="Liberation Sans"
w:hAnsi="Liberation Sans"/></w:rPr></w:lvl></w:abstractNum><w:num w:numId="1">
<w:abstractNumId w:val="0"/></w:num></w:numbering>'''

FOOTER = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:p><w:pPr><w:jc w:val="right"/><w:spacing w:before="0" w:after="0"/></w:pPr>
<w:r><w:rPr><w:rFonts w:ascii="Liberation Sans" w:hAnsi="Liberation Sans"/>
<w:sz w:val="18"/><w:color w:val="546675"/></w:rPr><w:t>MagPilot · </w:t></w:r>
<w:fldSimple w:instr="PAGE"><w:r><w:rPr><w:sz w:val="18"/></w:rPr><w:t>1</w:t></w:r></w:fldSimple>
<w:r><w:rPr><w:sz w:val="18"/></w:rPr><w:t> / 2</w:t></w:r></w:p></w:ftr>'''


def relationships(items):
    records = []
    for relationship_id, kind, target, external in items:
        records.append('<Relationship Id="{}" Type="{}" Target={}{} />'.format(
            relationship_id,
            'http://schemas.openxmlformats.org/officeDocument/2006/relationships/' + kind,
            quoteattr(target), ' TargetMode="External"' if external else ''))
    return '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' \
           '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' \
           + ''.join(records) + '</Relationships>'


def build_report(asset_dir, output):
    word_count = report_word_count()
    if word_count > 220:
        raise ValueError('Report exceeds 220 words: {}'.format(word_count))
    assets = {name: asset_dir / (name + '.png') for name in
              ('launcher_data_demo', 'collection_demo', 'project_timeline')}
    for path in assets.values():
        if not path.is_file():
            raise FileNotFoundError('Required report asset is missing: {}'.format(path))
        png_size(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    content_types = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="xml" ContentType="application/xml"/><Default Extension="png" ContentType="image/png"/>
<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
<Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>
<Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>
<Override PartName="/word/settings.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.settings+xml"/>
<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>'''
    package_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rIdDocument" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
<Relationship Id="rIdCore" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
<Relationship Id="rIdApp" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>'''
    document_rels = relationships([
        ('rIdData', 'image', 'media/launcher_data_demo.png', False),
        ('rIdCollection', 'image', 'media/collection_demo.png', False),
        ('rIdTimeline', 'image', 'media/project_timeline.png', False),
        ('rIdStyles', 'styles', 'styles.xml', False),
        ('rIdNumbering', 'numbering', 'numbering.xml', False),
        ('rIdFooter', 'footer', 'footer1.xml', False),
        ('rIdSettings', 'settings', 'settings.xml', False),
        ('rIdKJ', 'hyperlink', KJ_REFERENCE, True),
    ])
    now = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    core = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/"
xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><dc:title>MagPilot lab update — 5 October 2026</dc:title>
<dc:subject>Data collection, pen decisions, and proposed project timeline</dc:subject>
<dc:creator>MagPilot</dc:creator><dcterms:created xsi:type="dcterms:W3CDTF">{}</dcterms:created>
</cp:coreProperties>'''.format(now)
    app = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">
<Application>MagPilot report generator</Application><Pages>2</Pages><Words>{}</Words>
</Properties>'''.format(word_count)
    settings = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:settings xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:updateFields w:val="true"/><w:compat/></w:settings>'''
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, xml in {
            '[Content_Types].xml': content_types, '_rels/.rels': package_rels,
            'word/document.xml': document_xml(assets),
            'word/_rels/document.xml.rels': document_rels,
            'word/styles.xml': STYLES, 'word/numbering.xml': NUMBERING,
            'word/footer1.xml': FOOTER, 'word/settings.xml': settings,
            'docProps/core.xml': core, 'docProps/app.xml': app,
        }.items():
            archive.writestr(name, xml.encode('utf-8'))
        for path in assets.values():
            archive.write(path, 'word/media/' + path.name)
    return word_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-dir', type=Path, default=REPORT_DIR / 'assets')
    parser.add_argument('--output', type=Path, default=REPORT_DIR / (REPORT_NAME + '.docx'))
    args = parser.parse_args()
    word_count = build_report(args.asset_dir, args.output)
    print('{} ({} authored words; two landscape pages)'.format(args.output, word_count))


if __name__ == '__main__':
    main()
