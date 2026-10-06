#!/usr/bin/env python3
"""Export the SVG with host librsvg/Cairo; no Python graphics package needed."""
import ctypes as C
from ctypes.util import find_library
from pathlib import Path
import sys

folder = Path(sys.argv[1] if len(sys.argv) > 1 else 'hardware/magnetic_pen_v1').resolve()
rsvg = C.CDLL(find_library('rsvg-2'))
cairo = C.CDLL(find_library('cairo'))
gobject = C.CDLL(find_library('gobject-2.0'))
pointer, string, integer, number = C.c_void_p, C.c_char_p, C.c_int, C.c_double


def bind(lib, name, args, result):
    function = getattr(lib, name)
    function.argtypes, function.restype = args, result
    return function


load = bind(rsvg, 'rsvg_handle_new_from_file', [string, C.POINTER(pointer)], pointer)
render = bind(rsvg, 'rsvg_handle_render_cairo', [pointer, pointer], integer)
pdf_surface = bind(cairo, 'cairo_pdf_surface_create', [string, number, number], pointer)
image_surface = bind(cairo, 'cairo_image_surface_create', [integer, integer, integer], pointer)
create = bind(cairo, 'cairo_create', [pointer], pointer)
scale = bind(cairo, 'cairo_scale', [pointer, number, number], None)
status = bind(cairo, 'cairo_status', [pointer], integer)
surface_status = bind(cairo, 'cairo_surface_status', [pointer], integer)
finish = bind(cairo, 'cairo_surface_finish', [pointer], None)
write_png = bind(cairo, 'cairo_surface_write_to_png', [pointer, string], integer)
destroy = bind(cairo, 'cairo_destroy', [pointer], None)
destroy_surface = bind(cairo, 'cairo_surface_destroy', [pointer], None)
unref = bind(gobject, 'g_object_unref', [pointer], None)
handle = load(str(folder / 'drawing.svg').encode(), None)
assert handle, 'librsvg could not load the drawing'
for filename, factor in [('drawing.pdf', 0.5), ('drawing.png', 1.5)]:
    surface = (pdf_surface(str(folder / filename).encode(), 1440 * factor, 1110 * factor)
               if filename.endswith('.pdf') else image_surface(0, round(1440 * factor), round(1110 * factor)))
    assert surface_status(surface) == 0
    context = create(surface)
    scale(context, factor, factor)
    assert render(handle, context), 'librsvg could not render'
    assert status(context) == 0
    if filename.endswith('.png'):
        assert write_png(surface, str(folder / filename).encode()) == 0
    finish(surface)
    assert surface_status(surface) == 0
    destroy(context)
    destroy_surface(surface)
    print(filename, (folder / filename).stat().st_size, 'bytes')
unref(handle)
