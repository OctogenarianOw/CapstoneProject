#!/usr/bin/env python3
"""
Build the final report (Capstone B) as a Word document from the text in
content_a.py and content_b.py.

Role: turn a list of typed blocks (headings, paragraphs, equations,
figures, tables, an algorithm, references) into a .docx with numbered
captions and resolved cross-references, so the text can be revised in
Python and rebuilt, or edited directly in Word once frozen.
Choice: python-docx and a small inline markup (**bold**, *italic*,
_{sub}, ^{sup}, `mono`, [text](url) for a hyperlink, \\* for a literal
asterisk) rather than writing in Word by hand, so figure and table
numbers follow their order and a reference to {fig:key}, {tab:key},
{eq:key} or {alg:key} can never point at the wrong number. Rejected:
pandoc from Markdown, not installed here and weaker on table borders and
equation numbering.

Algorithms are typeset like LaTeX's algorithmicx rather than as a
monospace listing (draft v2): caption between two rules, proportional
type, keywords in bold and variables in italic, small line numbers,
real indentation per level, comments flush right after a triangle.
The supervisor's reader is used to that layout, and the monospace
version was hard to read (review of 1 Nov).

    PYTHONPATH=<dir with python-docx> python3 build_report.py
"""

import re
import struct
import sys
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_TAB_ALIGNMENT
from docx.opc.constants import RELATIONSHIP_TYPE as RT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from content_a import BLOCKS as BLOCKS_A  # noqa: E402
from content_b import BLOCKS as BLOCKS_B  # noqa: E402

OUT = HERE / 'report_draft_v3.docx'
FONT = 'Times New Roman'
MONO = 'Consolas'
TEXT_WIDTH_CM = 16.0
STAR = '\x00'
LINK_COLOUR = RGBColor(0x1F, 0x4E, 0x9A)
GREY = RGBColor(0x59, 0x59, 0x59)
ALG_PT = 10                 # algorithm type size
ALG_NUM_CM = 0.75           # where the code starts, after the line number
ALG_INDENT_CM = 0.6         # per nesting level
ALG_RIGHT_CM = 15.5         # right edge of the comments


# ------------------------------------------------------------ numbering

def number_blocks(blocks):
    """Assign numbers to figures, tables, equations, algorithms in order."""
    counters = {'fig': 0, 'tab': 0, 'eq': 0, 'alg': 0}
    labels = {}
    for b in blocks:
        kind = b[0]
        if kind in counters:
            counters[kind] += 1
            labels[f'{kind}:{b[1]}'] = counters[kind]
    return labels


def resolve(text, labels):
    def sub(m):
        key = m.group(1)
        if key not in labels:
            raise KeyError(f'unresolved reference {{{key}}}')
        kind, n = key.split(':')[0], labels[key]
        return {'fig': f'Figure {n}', 'tab': f'Table {n}', 'eq': f'({n})',
                'alg': f'Algorithm {n}'}[kind]
    return re.sub(r'\{((?:fig|tab|eq|alg):[a-z0-9_]+)\}', sub, text)


# --------------------------------------------------------- inline markup

LINK = re.compile(r'\[([^\]]+)\]\(([^)\s]+)\)')
TOKEN = re.compile(r'(\[[^\]]+\]\([^)\s]+\)|\*\*.+?\*\*|\*.+?\*|_\{.+?\}'
                   r'|\^\{.+?\}|`.+?`)')


def visible(text, labels):
    """The text as the reader sees it: references resolved, links as text."""
    return LINK.sub(r'\1', resolve(text, labels))


def add_hyperlink(par, text, url, size=None):
    """A clickable external link, blue and underlined."""
    r_id = par.part.relate_to(url, RT.HYPERLINK, is_external=True)
    link = OxmlElement('w:hyperlink')
    link.set(qn('r:id'), r_id)
    run = OxmlElement('w:r')
    # Children of w:rPr in the order the schema requires (color, sz, szCs,
    # u): Word refuses a file whose run properties are out of order.
    rpr = OxmlElement('w:rPr')
    colour = OxmlElement('w:color')
    colour.set(qn('w:val'), str(LINK_COLOUR))
    rpr.append(colour)
    if size:
        for tag in ('w:sz', 'w:szCs'):
            sz = OxmlElement(tag)
            sz.set(qn('w:val'), str(int(round(size * 2))))
            rpr.append(sz)
    under = OxmlElement('w:u')
    under.set(qn('w:val'), 'single')
    rpr.append(under)
    run.append(rpr)
    t = OxmlElement('w:t')
    t.set(qn('xml:space'), 'preserve')
    t.text = text
    run.append(t)
    link.append(run)
    par._p.append(link)


def add_runs(par, text, labels, size=None, italic=False, bold=False,
             font=None, colour=None):
    text = resolve(text, labels).replace('\\*', STAR)
    for piece in TOKEN.split(text):
        if not piece:
            continue
        m = LINK.fullmatch(piece)
        if m:
            add_hyperlink(par, m.group(1).replace(STAR, '*'), m.group(2),
                          size)
            continue
        kw = {'bold': bold, 'italic': italic, 'sub': False, 'sup': False,
              'mono': False}
        if piece.startswith('**') and piece.endswith('**'):
            piece, kw['bold'] = piece[2:-2], True
        elif piece.startswith('*') and piece.endswith('*') and len(piece) > 1:
            piece, kw['italic'] = piece[1:-1], True
        elif piece.startswith('_{'):
            piece, kw['sub'] = piece[2:-1], True
        elif piece.startswith('^{'):
            piece, kw['sup'] = piece[2:-1], True
        elif piece.startswith('`'):
            piece, kw['mono'] = piece[1:-1], True
        run = par.add_run(piece.replace(STAR, '*'))
        # Only set what is asked: an explicit False would override the
        # paragraph style (headings are bold by style).
        if kw['bold']:
            run.bold = True
        if kw['italic']:
            run.italic = True
        if kw['sub']:
            run.font.subscript = True
        if kw['sup']:
            run.font.superscript = True
        if kw['mono']:
            set_font(run, MONO)
            run.font.size = Pt((size or 11) - 1.5)
        elif font:
            set_font(run, font)
        if size and not kw['mono']:
            run.font.size = Pt(size)
        if colour is not None:
            run.font.color.rgb = colour
    return par


def set_font(run, name):
    run.font.name = name
    rpr = run._element.get_or_add_rPr()
    fonts = rpr.find(qn('w:rFonts'))
    if fonts is None:
        fonts = OxmlElement('w:rFonts')
        rpr.append(fonts)
    for attr in ('w:ascii', 'w:hAnsi', 'w:cs', 'w:eastAsia'):
        fonts.set(qn(attr), name)


# --------------------------------------------------------------- styles

def setup_document():
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    for side in ('left_margin', 'right_margin'):
        setattr(sec, side, Cm(2.5))
    sec.top_margin, sec.bottom_margin = Cm(2.5), Cm(2.5)
    normal = doc.styles['Normal']
    normal.font.name = FONT
    normal.font.size = Pt(11)
    rpr = normal.element.get_or_add_rPr()
    fonts = rpr.find(qn('w:rFonts'))
    if fonts is None:
        fonts = OxmlElement('w:rFonts')
        rpr.append(fonts)
    for attr in ('w:ascii', 'w:hAnsi', 'w:cs', 'w:eastAsia'):
        fonts.set(qn(attr), FONT)
    pf = normal.paragraph_format
    pf.space_after = Pt(6)
    pf.line_spacing = 1.15
    for level, size, before in ((1, 14, 18), (2, 12, 12), (3, 11, 10)):
        st = doc.styles[f'Heading {level}']
        st.font.name = FONT
        st.font.size = Pt(size)
        st.font.bold = True
        st.font.italic = level == 3
        st.font.color.rgb = RGBColor(0, 0, 0)
        hr = st.element.get_or_add_rPr()
        hf = hr.find(qn('w:rFonts'))
        if hf is None:
            hf = OxmlElement('w:rFonts')
            hr.append(hf)
        for attr in ('w:ascii', 'w:hAnsi', 'w:cs', 'w:eastAsia'):
            hf.set(qn(attr), FONT)
        # The built-in heading styles name theme fonts, which Word applies
        # over w:ascii: drop them or the headings come out in Calibri.
        for attr in ('w:asciiTheme', 'w:hAnsiTheme', 'w:eastAsiaTheme',
                     'w:cstheme'):
            if hf.get(qn(attr)) is not None:
                del hf.attrib[qn(attr)]
        st.paragraph_format.space_before = Pt(before)
        st.paragraph_format.space_after = Pt(6)
        st.paragraph_format.keep_with_next = True
    add_page_number(sec)
    return doc


def add_page_number(section):
    par = section.footer.paragraphs[0]
    par.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = par.add_run()
    for tag, text in (('begin', None), (None, 'PAGE'), ('end', None)):
        if tag:
            el = OxmlElement('w:fldChar')
            el.set(qn('w:fldCharType'), tag)
        else:
            el = OxmlElement('w:instrText')
            el.set(qn('xml:space'), 'preserve')
            el.text = text
        run._element.append(el)


def table_borders(table, header_rows=1):
    """Horizontal rules only: top, under the header, bottom."""
    tbl = table._tbl
    pr = tbl.tblPr
    borders = OxmlElement('w:tblBorders')
    for edge, size in (('top', 12), ('bottom', 12), ('left', 0),
                       ('right', 0), ('insideH', 0), ('insideV', 0)):
        el = OxmlElement(f'w:{edge}')
        el.set(qn('w:val'), 'single' if size else 'nil')
        if size:
            el.set(qn('w:sz'), str(size))
            el.set(qn('w:color'), '000000')
        borders.append(el)
    pr.append(borders)
    for cell in table.rows[header_rows - 1].cells:
        tcpr = cell._tc.get_or_add_tcPr()
        tcb = OxmlElement('w:tcBorders')
        bottom = OxmlElement('w:bottom')
        bottom.set(qn('w:val'), 'single')
        bottom.set(qn('w:sz'), '6')
        bottom.set(qn('w:color'), '000000')
        tcb.append(bottom)
        tcpr.append(tcb)


def png_size(path):
    with open(path, 'rb') as fh:
        head = fh.read(24)
    return struct.unpack('>II', head[16:24])


def keep_row_together(row):
    trpr = row._tr.get_or_add_trPr()
    el = OxmlElement('w:cantSplit')
    trpr.append(el)


def render_algorithm(doc, block, labels):
    """('alg', key, title, header lines, numbered lines, footer lines).

    A numbered line is (level, code, comment): code indented by level,
    comment flush right after a triangle, both with the inline markup.
    """
    _, key, title, head, lines, foot = block
    table = doc.add_table(rows=2, cols=1)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    for row in table.rows:
        keep_row_together(row)
        row.cells[0].width = Cm(TEXT_WIDTH_CM)
    cap = table.cell(0, 0).paragraphs[0]
    cap.paragraph_format.space_before = Pt(2)
    cap.paragraph_format.space_after = Pt(2)
    cap.paragraph_format.keep_with_next = True
    add_runs(cap, f'**Algorithm {labels["alg:" + key]}.** ' + title, labels,
             size=9.5)
    body = table.cell(1, 0)
    pars = iter([body.paragraphs[0]])

    def new_par(space_after=0.5):
        par = next(pars, None) or body.add_paragraph()
        pf = par.paragraph_format
        pf.space_before = Pt(0)
        pf.space_after = Pt(space_after)
        pf.line_spacing = 1.08
        return par

    for k, text in enumerate(head):
        par = new_par(3 if k == len(head) - 1 else 0.5)
        par.paragraph_format.left_indent = Cm(ALG_NUM_CM)
        par.paragraph_format.first_line_indent = Cm(-ALG_NUM_CM)
        add_runs(par, text, labels, size=ALG_PT)
    for n, (level, code, comment) in enumerate(lines, 1):
        par = new_par()
        x = ALG_NUM_CM + level * ALG_INDENT_CM
        pf = par.paragraph_format
        pf.left_indent = Cm(x)
        pf.first_line_indent = Cm(-x)
        stops = pf.tab_stops
        stops.add_tab_stop(Cm(ALG_NUM_CM - 0.3), WD_TAB_ALIGNMENT.RIGHT)
        stops.add_tab_stop(Cm(x), WD_TAB_ALIGNMENT.LEFT)
        stops.add_tab_stop(Cm(ALG_RIGHT_CM), WD_TAB_ALIGNMENT.RIGHT)
        run = par.add_run(f'\t{n}:\t')
        run.font.size = Pt(ALG_PT - 2)
        run.font.color.rgb = GREY
        add_runs(par, code, labels, size=ALG_PT)
        if comment:
            par.add_run('\t')
            add_runs(par, '▷ ' + comment, labels, size=ALG_PT - 0.5,
                     italic=True, colour=GREY)
    for text in foot:
        par = new_par()
        par.paragraph_format.space_before = Pt(3)
        add_runs(par, text, labels, size=ALG_PT - 0.5, colour=GREY)
    table_borders(table)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


# --------------------------------------------------------------- blocks

def render(doc, blocks, labels):
    words = {'body': 0}
    counting = False
    for b in blocks:
        kind = b[0]
        if kind == 'title':
            info = b[1]
            for text, size, bold, space in (
                    (info['title'], 18, True, 18),
                    (info['subtitle'], 12, False, 24),
                    (info['author'], 12, True, 4),
                    (info['id'], 11, False, 4),
                    (info['supervisor'], 11, False, 4),
                    (info['school'], 11, False, 4),
                    (info['date'], 11, False, 24)):
                par = doc.add_paragraph()
                par.alignment = WD_ALIGN_PARAGRAPH.CENTER
                par.paragraph_format.space_after = Pt(space)
                add_runs(par, text, labels, size=size, bold=bold)
        elif kind == 'abstract':
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.CENTER
            add_runs(par, '**Abstract**', labels, size=12)
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            par.paragraph_format.left_indent = Cm(1.0)
            par.paragraph_format.right_indent = Cm(1.0)
            add_runs(par, b[1], labels, size=10.5)
        elif kind == 'keywords':
            par = doc.add_paragraph()
            par.paragraph_format.left_indent = Cm(1.0)
            par.paragraph_format.right_indent = Cm(1.0)
            add_runs(par, '**Keywords:** ' + b[1], labels, size=10.5)
        elif kind == 'count_on':
            counting = True
        elif kind == 'count_off':
            counting = False
        elif kind in ('h1', 'h2', 'h3'):
            level = int(kind[1])
            h = doc.add_heading(level=level)
            add_runs(h, b[1], labels)
            if counting:
                words['body'] += len(b[1].split())
        elif kind == 'p':
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            add_runs(par, b[1], labels)
            if counting:
                words['body'] += len(visible(b[1], labels).split())
        elif kind == 'quote':
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            par.paragraph_format.left_indent = Cm(1.2)
            par.paragraph_format.right_indent = Cm(1.2)
            add_runs(par, b[1], labels, italic=True)
            if counting:
                words['body'] += len(b[1].split())
        elif kind == 'ul':
            for item in b[1]:
                par = doc.add_paragraph(style='List Bullet')
                par.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
                add_runs(par, item, labels)
                if counting:
                    words['body'] += len(visible(item, labels).split())
        elif kind == 'eq':
            par = doc.add_paragraph()
            stops = par.paragraph_format.tab_stops
            stops.add_tab_stop(Cm(TEXT_WIDTH_CM / 2), WD_TAB_ALIGNMENT.CENTER)
            stops.add_tab_stop(Cm(TEXT_WIDTH_CM), WD_TAB_ALIGNMENT.RIGHT)
            par.add_run('\t')
            add_runs(par, b[2], labels)
            par.add_run(f'\t({labels["eq:" + b[1]]})')
        elif kind == 'fig':
            _, key, rel, width_cm, caption = b
            path = ROOT / rel
            w, h = png_size(path)
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.CENTER
            par.paragraph_format.keep_with_next = True
            par.paragraph_format.space_before = Pt(6)
            par.add_run().add_picture(str(path), width=Cm(width_cm))
            cap = doc.add_paragraph()
            cap.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            cap.paragraph_format.space_after = Pt(12)
            add_runs(cap, f'**Figure {labels["fig:" + key]}.** ' + caption,
                     labels, size=9.5)
        elif kind == 'tab':
            _, key, caption, header, rows, widths = b[:6]
            cap = doc.add_paragraph()
            cap.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            cap.paragraph_format.keep_with_next = True
            cap.paragraph_format.space_before = Pt(6)
            add_runs(cap, f'**Table {labels["tab:" + key]}.** ' + caption,
                     labels, size=9.5)
            table = doc.add_table(rows=1 + len(rows), cols=len(header))
            table.alignment = WD_TABLE_ALIGNMENT.CENTER
            table.autofit = False
            for r, row in enumerate([header] + rows):
                for c, text in enumerate(row):
                    cell = table.cell(r, c)
                    cell.width = Cm(widths[c])
                    par = cell.paragraphs[0]
                    par.paragraph_format.space_after = Pt(1)
                    par.paragraph_format.line_spacing = 1.0
                    add_runs(par, f'**{text}**' if r == 0 else text, labels,
                             size=9)
            table_borders(table)
            doc.add_paragraph().paragraph_format.space_after = Pt(2)
        elif kind == 'alg':
            render_algorithm(doc, b, labels)
        elif kind == 'ref':
            par = doc.add_paragraph()
            par.alignment = WD_ALIGN_PARAGRAPH.LEFT
            par.paragraph_format.left_indent = Cm(0.8)
            par.paragraph_format.first_line_indent = Cm(-0.8)
            par.paragraph_format.space_after = Pt(4)
            add_runs(par, b[1], labels, size=10)
        elif kind == 'placeholder':
            par = doc.add_paragraph()
            run = par.add_run(b[1])
            run.italic = True
            run.font.highlight_color = 7   # yellow
        elif kind == 'pagebreak':
            doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
        else:
            raise ValueError(f'unknown block {kind}')
    return words


def main():
    blocks = BLOCKS_A + BLOCKS_B
    labels = number_blocks(blocks)
    doc = setup_document()
    words = render(doc, blocks, labels)
    doc.save(OUT)
    n = {k: sum(1 for x in labels if x.startswith(k)) for k in
         ('fig', 'tab', 'eq', 'alg')}
    print(f'{OUT}\nbody words (sections 1 to 7, text only) {words["body"]}; '
          f'figures {n["fig"]}, tables {n["tab"]}, equations {n["eq"]}, '
          f'algorithms {n["alg"]}')


if __name__ == '__main__':
    main()
