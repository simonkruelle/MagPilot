"""Parametric prototype: run with Blender 4.5+, no third-party packages.

blender --background --factory-startup --python hardware/magnetic_pen_v1/build_pen.py
STL coordinates are millimetres. Raw sensor data are never read or altered.
"""

import json
import math
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector

HERE = Path(__file__).resolve().parent
P = json.loads((HERE/'parameters.json').read_text())
SEGMENTS = 160
STL = HERE/'stl'
STL.mkdir(exist_ok=True)


def active(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def clean(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=1e-5)
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=1e-7)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()


def mesh(name, vertices, faces):
    data = bpy.data.meshes.new(name)
    data.from_pydata(vertices, [], faces)
    data.update()
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    clean(obj)
    return obj


def lathe(name, profile):
    vertices, rings, faces = [], [], []
    for radius, z in profile:
        if radius < 1e-8:
            rings.append([len(vertices)])
            vertices.append((0, 0, z))
        else:
            ring = []
            for i in range(SEGMENTS):
                angle = i*2*math.pi/SEGMENTS
                ring.append(len(vertices))
                vertices.append((radius*math.cos(angle), radius*math.sin(angle), z))
            rings.append(ring)
    for first, second in zip(rings, rings[1:]):
        if len(first) == 1:
            faces.extend((first[0], second[(i+1)%SEGMENTS], second[i]) for i in range(SEGMENTS))
        elif len(second) == 1:
            faces.extend((first[i], first[(i+1)%SEGMENTS], second[0]) for i in range(SEGMENTS))
        else:
            faces.extend((first[i], first[(i+1)%SEGMENTS], second[(i+1)%SEGMENTS], second[i])
                         for i in range(SEGMENTS))
    return mesh(name, vertices, faces)


def sector(name, polygon_rz, first_deg, last_deg):
    steps = max(2, math.ceil(abs(last_deg-first_deg)/3))
    vertices, faces = [], []
    size = len(polygon_rz)
    for i in range(steps+1):
        angle = math.radians(first_deg+(last_deg-first_deg)*i/steps)
        vertices.extend((r*math.cos(angle), r*math.sin(angle), z) for r, z in polygon_rz)
    faces.append(tuple(range(size-1, -1, -1)))
    faces.append(tuple(steps*size+i for i in range(size)))
    for i in range(steps):
        for j in range(size):
            k = (j+1)%size
            faces.append((i*size+j, i*size+k, (i+1)*size+k, (i+1)*size+j))
    return mesh(name, vertices, faces)


def cylinder(name, radius, z0, z1, xy=(0, 0)):
    obj = lathe(name, [(0,z0),(radius,z0),(radius,z1),(0,z1)])
    if xy != (0,0):
        for v in obj.data.vertices:
            v.co.x += xy[0]
            v.co.y += xy[1]
    return obj


def box(name, dimensions, location):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    active(obj)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj


def boolean(obj, cutter, operation='DIFFERENCE'):
    active(obj)
    mod = obj.modifiers.new('Solid geometry', 'BOOLEAN')
    mod.operation, mod.solver, mod.object = operation, 'EXACT', cutter
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)
    return obj


def clone(obj, name):
    new = obj.copy()
    new.data = obj.data.copy()
    new.name = name
    bpy.context.collection.objects.link(new)
    return new


def material(name, color, metallic=0, roughness=.3):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color,1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color,1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    return mat


def assign(obj, mat, smooth=True):
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    # Curved outer walls look smooth; planar faces retain flat normals.
    for face in obj.data.polygons:
        face.use_smooth = smooth and abs(face.normal.z) < .995


def validate(obj):
    clean(obj)
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.triangulate(bm,faces=list(bm.faces))
    bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=1e-6)
    bmesh.ops.triangulate(bm,faces=list(bm.faces))
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(obj.data)
    obj.data.update()
    bad = sum(not edge.is_manifold for edge in bm.edges)
    volume = bm.calc_volume(signed=True)
    unseen = set(bm.verts)
    components = 0
    while unseen:
        components += 1
        todo = [unseen.pop()]
        while todo:
            vertex = todo.pop()
            for edge in vertex.link_edges:
                neighbour = edge.other_vert(vertex)
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    todo.append(neighbour)
    result = dict(non_manifold_edges=bad, connected_components=components,
                  volume_mm3=volume, vertices=len(bm.verts), faces=len(bm.faces))
    bm.free()
    if bad or volume <= 0 or components != 1:
        raise ValueError('Invalid printable mesh '+obj.name+': '+str(result))
    return result


def export_stl(obj, name, inverted=False):
    copy = clone(obj, 'STL temporary')
    copy.location = (0,0,0)
    copy.rotation_euler = (math.pi if inverted else 0,0,0)
    active(copy)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    low = min(v.co.z for v in copy.data.vertices)
    for vertex in copy.data.vertices:
        vertex.co.z -= low
    bpy.ops.wm.stl_export(filepath=str(STL/(name+'.stl')), export_selected_objects=True,
                          apply_modifiers=True, global_scale=1, use_scene_unit=False,
                          ascii_format=False)
    bpy.data.objects.remove(copy, do_unlink=True)


def overlap_volume(first, second, shift_z=0, turn=0):
    first_copy = clone(first, 'Fit intersection')
    second_copy = clone(second, 'Fit transformed cap')
    second_copy.location.z += shift_z
    second_copy.rotation_euler.z = math.radians(turn)
    boolean(first_copy, second_copy, 'INTERSECT')
    bm = bmesh.new()
    bm.from_mesh(first_copy.data)
    volume = abs(bm.calc_volume(signed=True))
    bm.free()
    bpy.data.objects.remove(first_copy, do_unlink=True)
    return volume


def build():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    floor = P['lower_magnet_face_from_contact_mm']
    roof = floor+P['max_magnets']*P['magnet_thickness_mm']+P['stack_axial_allowance_mm']
    radius = P['grip_diameter_mm']/2
    bore = P['magnet_bore_diameter_mm']/2
    neck = P['neck_diameter_mm']/2
    neck_top = roof+P['neck_length_mm']
    tip = P['stylus_tip_radius_mm']
    if not (radius > neck+P['closure_radial_clearance_mm'] and neck-bore >= .8 and floor > tip):
        raise ValueError('Parameters leave insufficient prototype wall thickness.')
    if max(P['magnet_diameter_mm'],P['spacer_diameter_mm'],P['washer_diameter_mm']) >= P['magnet_bore_diameter_mm']:
        raise ValueError('Magnet, spacer and washer must each fit the selected bore.')
    if P['washer_nominal_thickness_mm'] <= P['stack_axial_allowance_mm']+P['closure_axial_clearance_mm']:
        raise ValueError('Washer must retain compression with the locked cap axial play.')
    tip_profile = [(tip*math.cos(math.radians(a)), tip+tip*math.sin(math.radians(a)))
                   for a in range(-90,1,6)]
    grip = lathe('lower_grip', tip_profile+[(radius,floor+1.2),(radius,roof-1.4),
                 (neck,roof),(neck,neck_top),(0,neck_top)])
    boolean(grip, cylinder('Magnet chamber', bore, floor, neck_top+.1))
    lug_bottom = roof+4
    lug_top = lug_bottom+P['lug_height_mm']
    half_angle = P['lug_angle_deg']/2
    for angle in (0,180):
        lug = sector('Bayonet lug', [(neck-.1,lug_bottom),
                     (neck+P['lug_radial_height_mm'],lug_bottom),
                     (neck+P['lug_radial_height_mm'],lug_top-.6),
                     (neck,lug_top),(neck-.1,lug_top)], angle-half_angle, angle+half_angle)
        boolean(grip, lug, 'UNION')
    barrel_radius = P['upper_barrel_diameter_mm']/2
    cap_roof = neck_top+.4
    cap_profile = [(0,roof),(radius,roof),(radius,cap_roof+.8),
                   (barrel_radius,cap_roof+2.8),
                   (5.7,P['overall_length_mm']-5.7)]
    cap_profile += [(5.7*math.cos(math.radians(a)),
                     P['overall_length_mm']-5.7+5.7*math.sin(math.radians(a)))
                    for a in range(6,91,6)]
    cap = lathe('upper_barrel', cap_profile)
    boolean(cap, cylinder('Cap socket', neck+P['closure_radial_clearance_mm'], roof-.1, cap_roof))
    pusher_radius = bore-.25
    boolean(cap, cylinder('Fixed stack roof', pusher_radius, roof, cap_roof+.1), 'UNION')
    boolean(cap, cylinder('Lightweight handle interior', 3.5, cap_roof+1.2, P['overall_length_mm']-6))
    boolean(cap, cylinder('Handle vent', P['washer_vent_diameter_mm']/2, roof-.1, cap_roof+1.3))
    for angle in (0,180):
        width = half_angle+P['closure_angle_clearance_deg']
        z0 = lug_bottom-P['closure_axial_clearance_mm']
        z1 = lug_top+P['closure_axial_clearance_mm']
        slot = [(neck-.1,roof-.1),(radius+.2,roof-.1),(radius+.2,z1),(neck-.1,z1)]
        boolean(cap, sector('Insertion slot', slot, angle-width, angle+width))
        channel = [(neck-.1,z0),(radius+.2,z0),(radius+.2,z1),(neck-.1,z1)]
        boolean(cap, sector('Twist channel', channel,
                            angle-P['closure_turn_deg']-width, angle+width))
    spacers = []
    for missing in range(1,P['max_magnets']):
        length = missing*P['magnet_thickness_mm']
        spacer = cylinder('spacer_{:g}mm'.format(length), P['spacer_diameter_mm']/2, 0, length)
        boolean(spacer, cylinder('Vent / extraction hole', P['spacer_vent_diameter_mm']/2, -.1, length+.1))
        spacers.append(spacer)
    washer_name = 'washer_{:g}mm'.format(P['washer_nominal_thickness_mm']).replace('.', 'p')
    washer = cylinder(washer_name, P['washer_diameter_mm']/2, 0, P['washer_nominal_thickness_mm'])
    boolean(washer, cylinder('Washer vent', P['washer_vent_diameter_mm']/2, -.1,
                            P['washer_nominal_thickness_mm']+.1))
    coupon = box('bore_fit_coupon', (58,17,7.2), (0,0,3.6))
    for index, diameter in enumerate(P['fit_coupon_bores_mm']):
        xy = (-21+index*14,0)
        boolean(coupon, cylinder('Test bore', diameter/2, 1.2, 7.3, xy))
    boolean(coupon, cylinder('Lower-left identification notch',2,-.1,7.3,(-29,-8.5)))
    fit_grip = clone(grip, 'closure_fit_grip')
    boolean(fit_grip, box('Remove full lower body', (40,40,roof-1+20), (0,0,(roof-1-20)/2)))
    boolean(fit_grip, cylinder('Coupon floor', radius, roof-2.4, roof-.8), 'UNION')
    fit_cap = clone(cap, 'closure_fit_cap')
    boolean(fit_cap, box('Remove full handle', (40,40,200), (0,0,cap_roof+4+100)))
    parts = [grip,cap,*spacers,washer,coupon,fit_grip,fit_cap]
    checks = {obj.name:validate(obj) for obj in parts}
    fit_checks = []
    poses = [(shift,0) for shift in (12,10,8,6,4,3,2,1,.5,0)]
    poses += [(0,P['closure_turn_deg']*i/12) for i in range(1,13)]
    for shift, turn in poses:
        volume = overlap_volume(grip,cap,shift,turn)
        fit_checks.append(dict(cap_lift_mm=shift, cap_turn_deg=turn, rigid_overlap_mm3=volume))
        if volume > .001:
            raise ValueError('Rigid parts collide on insertion/turn: '+str(fit_checks[-1]))
    retention_volume = overlap_volume(grip,cap,.6,P['closure_turn_deg'])
    if retention_volume <= .01:
        raise ValueError('Locked cap is not retained against straight pull.')
    for obj in parts:
        export_stl(obj,obj.name,inverted=obj in (grip,fit_grip))
    output = dict(parameters=P, printable_meshes=checks, rigid_fit_checks=fit_checks,
                  locked_cap_pull_at_0p6mm_overlap_mm3=retention_volume,
                  lower_face_from_contact_mm=floor, usable_pocket_height_mm=roof-floor,
                  configurations=[dict(magnets=n, spacer_length_mm=(P['max_magnets']-n)*P['magnet_thickness_mm'],
                                       geometric_stack_centre_from_contact_mm=floor+n*P['magnet_thickness_mm']/2)
                                  for n in range(1,P['max_magnets']+1)],
                  limitations=['Geometric prototype; physical print, closure retention and bore fit untested.',
                               'Washer must be compressible foam or sufficiently compliant elastomer, validated by fit; rigid plastic STL would prevent closing at nominal thickness.',
                               'No magnets, graphite or metal parts are included in printable meshes.',
                               'Bayonet requires washer preload and a retention test; no validated mechanical life.',
                               'Pen changes magnet height relative to bare-disk recordings; test height/tilt and recalibrate.'])
    (HERE/'validation.json').write_text(json.dumps(output,indent=2)+'\n')
    return grip,cap,spacers,washer,coupon,fit_grip,fit_cap,roof


def add_light(name, position, power, size):
    data = bpy.data.lights.new(name,'AREA')
    data.energy, data.shape, data.size = power,'DISK',size
    obj = bpy.data.objects.new(name,data)
    bpy.context.collection.objects.link(obj)
    obj.location = position
    obj.rotation_euler = (Vector((0,0,35))-obj.location).to_track_quat('-Z','Y').to_euler()


def label(text, position, size, mat):
    data = bpy.data.curves.new(text,'FONT')
    data.body, data.size, data.extrude = text,size,0
    obj = bpy.data.objects.new(text,data)
    bpy.context.collection.objects.link(obj)
    obj.location = position
    obj.rotation_euler = (math.pi/2,0,0)
    data.materials.append(mat)
    return obj


def render(parts):
    grip,cap,spacers,washer,coupon,fit_grip,fit_cap,roof = parts
    white = material('Graphite polymer body',(.055,.075,.11),roughness=.28)
    grey = material('Graphite polymer',(.12,.15,.20),roughness=.3)
    blue = material('Spacer blue',(.025,.3,.78),roughness=.3)
    silver = material('Magnet · separate hardware',(.28,.30,.33),metallic=.7,roughness=.23)
    black = material('Compliant washer · separate hardware',(.055,.065,.08),roughness=.6)
    ink = material('Labels',(.09,.12,.18),roughness=1)
    assign(grip,white)
    assign(cap,white)
    for obj in spacers:
        assign(obj,blue)
    assign(washer,black)
    for obj in (coupon,fit_grip,fit_cap):
        obj.hide_render = True
        obj.hide_set(True)
    # Actual assembled mesh, then an exploded duplicate of the same parts.
    grip.location.x,cap.location.x = -39,-39
    cap.rotation_euler.z = math.radians(P['closure_turn_deg'])
    cap['assembly'] = 'Align entry slots, insert, then turn 45 degrees counterclockwise viewed from tail toward tip.'
    exploded_grip = clone(grip,'Exploded lower grip')
    exploded_grip.location.x = 22
    exploded_cap = clone(fit_cap,'Exploded closure detail / upper barrel omitted')
    exploded_cap.hide_render = False
    exploded_cap.hide_set(False)
    exploded_cap.location.x = 22
    exploded_cap.location.z = 54
    exploded_cap.rotation_euler.z = math.radians(P['closure_turn_deg'])
    assign(exploded_cap,white)
    for count in range(3):
        magnet = cylinder('Exploded magnet {}'.format(count+1), P['magnet_diameter_mm']/2,0,P['magnet_thickness_mm'])
        magnet.location = (22,0,46+count*(P['magnet_thickness_mm']+2))
        assign(magnet,silver)
        magnet['not_printed'] = True
    show_spacer = spacers[1]
    for spacer in spacers:
        spacer.hide_render = True
        spacer.hide_set(True)
    display = clone(show_spacer,'Exploded 10 mm spacer')
    display.hide_render = False
    display.hide_set(False)
    display.location = (22,0,69)
    washer.location = (22,0,82)
    # Cross section: an actual boolean cut through a grip copy, two magnets
    # and the 15 mm spacer, at their true assembled heights.
    section = clone(grip,'Cutaway lower grip')
    section.location = (66,0,0)
    cutter = box('Section half-space',(40,20,70),(66,-10,30))
    boolean(section,cutter)
    assign(section,grey,smooth=False)
    for n in range(2):
        lower = P['lower_magnet_face_from_contact_mm']+P['magnet_thickness_mm']*n
        magnet = cylinder('Section magnet {}'.format(n+1),P['magnet_diameter_mm']/2,lower,lower+P['magnet_thickness_mm'])
        magnet.location.x = 66
        boolean(magnet,box('Magnet section',(40,20,70),(66,-10,30)))
        assign(magnet,silver,smooth=False)
    actual_spacer = clone(spacers[2],'Section spacer')
    actual_spacer.hide_render = False
    actual_spacer.hide_set(False)
    actual_spacer.location = (66,0,P['lower_magnet_face_from_contact_mm']+2*P['magnet_thickness_mm'])
    boolean(actual_spacer,box('Spacer section',(40,20,70),(66,-10,30)))
    assign(actual_spacer,blue,smooth=False)
    for text, pos, size in [
        ('MagPilot',(-74,-1,152),9),
        ('Magnetic pen / v1 prototype',(-73,-1,142),3.8),
        ('{:g} mm / {:g} mm grip'.format(P['overall_length_mm'],P['grip_diameter_mm']),(-67,-1,-12),3.5),
        ('45-degree twist',(-63,-1,-20),3.3),
        ('3 magnets + {:g} mm spacer'.format(2*P['magnet_thickness_mm']),(0,-1,-12),3.1),
        ('Upper barrel omitted',(2,-1,-20),2.6),
        ('2 magnets + {:g} mm spacer'.format(3*P['magnet_thickness_mm']),(46,-1,-20),2.6),
        ('Fixed lower stop: {:g} mm'.format(P['lower_magnet_face_from_contact_mm']),(47,-1,-27),2.6)]:
        label(text,pos,size,ink)
    # Tilt very slightly for a useful perspective of the round and cut parts.
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = .001
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    scene.render.resolution_x,scene.render.resolution_y = 1900,1600
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.film_transparent = False
    scene.world.use_nodes = True
    world_background = scene.world.node_tree.nodes.get('Background')
    world_background.inputs['Color'].default_value = (.85,.88,.95,1)
    world_background.inputs['Strength'].default_value = .7
    scene.view_settings.view_transform = 'Standard'
    backdrop = material('Background',(0,0,0),roughness=1)
    backdrop_shader = backdrop.node_tree.nodes.get('Principled BSDF')
    backdrop_shader.inputs['Emission Color'].default_value = (1,1,1,1)
    backdrop_shader.inputs['Emission Strength'].default_value = 1
    plane = box('Backdrop',(300,2,280),(0,9,70))
    assign(plane,backdrop,smooth=False)
    add_light('Large soft key',(-70,-90,155),180000,100)
    add_light('Soft fill',(100,-50,90),90000,80)
    data = bpy.data.cameras.new('Preview camera')
    camera = bpy.data.objects.new('Preview camera',data)
    bpy.context.collection.objects.link(camera)
    camera.location = (0,-410,168)
    camera.rotation_euler = (Vector((0,0,65))-camera.location).to_track_quat('-Z','Y').to_euler()
    data.type,data.ortho_scale = 'ORTHO',250
    scene.camera = camera
    scene.render.filepath = str(HERE/'preview.png')
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'magnetic_pen_v1.blend'))
    bpy.ops.render.render(write_still=True)


if __name__ == '__main__':
    parts = build()
    print('PRINTABLE MESH AND RIGID FIT CHECKS PASSED')
    sys.path.insert(0,str(HERE))
    from verify_stl import main as verify_exports
    verify_exports()
    if '--no-render' not in sys.argv:
        render(parts)
