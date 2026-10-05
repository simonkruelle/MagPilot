"""Local FR3 physics and rendering for the position-reaching collection task.

The measured fingertip midpoint comes from MuJoCo forward dynamics. Cartesian
commands drive the model's joint position actuators; commanded coordinates are
never substituted for feedback. This backend has no ROS or real-robot output.
"""

from itertools import product
from copy import deepcopy
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

from colmag.robot_targets import DIGIT_CUBE_CENTER_M, DIGIT_CUBE_EDGE_M


MODEL_DIRECTORY = Path(__file__).resolve().parents[1] / 'assets' / 'teleoperation' / 'franka_fr3'
HAND_DIRECTORY = MODEL_DIRECTORY.parent / 'franka_hand'
GRIPPER_OPENING_M = .040


def _attach_franka_hand(root, hand_directory):
    """Attach the licensed hand geometry, with fixed opposing pad-face sites."""
    source = ET.parse(hand_directory / 'panda.xml').getroot()
    hand = deepcopy(source.find('.//body[@name="hand"]'))
    referenced_meshes = {geom.get('mesh') for geom in hand.iter('geom') if geom.get('mesh')}
    defaults = deepcopy(source.find('default'))
    for element in defaults.iter():
        if 'class' in element.attrib:
            element.set('class', 'hand_' + element.get('class'))
    root.find('default').extend(list(defaults))
    assets = root.find('asset')
    for element in source.find('asset'):
        if element.tag == 'mesh':
            original = element.get('name', Path(element.get('file')).stem)
            if original not in referenced_meshes:
                continue
            element = deepcopy(element)
            element.set('name', 'hand_' + original)
            element.set('file', 'hand/' + element.get('file'))
        elif element.tag == 'material':
            element = deepcopy(element)
            element.set('name', 'hand_' + element.get('name'))
            element.set('class', 'hand_' + element.get('class'))
        else:
            continue
        assets.append(element)
    for element in hand.iter():
        for attribute in ('class', 'mesh', 'material'):
            if attribute in element.attrib:
                element.set(attribute, 'hand_' + element.get(attribute))
    hand.set('name', 'franka_hand')
    hand.set('childclass', 'hand_panda')
    # The inner face and longitudinal centre are derived from the source
    # contact-pad box, rather than an arbitrary point on the flange.
    pad = source.find('.//default[@class="fingertip_pad_collision_1"]/geom')
    pad_position = np.fromstring(pad.get('pos'), sep=' ')
    pad_size = np.fromstring(pad.get('size'), sep=' ')
    inner_face_y = pad_position[1] - pad_size[1]
    displacement = GRIPPER_OPENING_M / 2 - inner_face_y
    finger_height = None
    for name, sign in (('left_finger', 1), ('right_finger', -1)):
        finger = hand.find('./body[@name="{}"]'.format(name))
        origin = np.fromstring(finger.get('pos'), sep=' ')
        origin[1] += sign * displacement
        finger_height = origin[2]
        finger.set('pos', ' '.join('{:.8g}'.format(value) for value in origin))
        for joint in list(finger.findall('joint')):
            finger.remove(joint)
        ET.SubElement(finger, 'site', name=name + '_tip',
                      pos='0 {:.8g} {:.8g}'.format(inner_face_y, pad_position[2]),
                      size='.002', rgba='0 0 0 0', group='4')
    ET.SubElement(hand, 'site', name='gripper_center',
                  pos='0 0 {:.8g}'.format(finger_height + pad_position[2]),
                  size='.002', rgba='0 0 0 0', group='4')
    root.find('.//body[@name="fr3_link7"]').append(hand)


def _scene_xml(model_directory, hand_directory=HAND_DIRECTORY):
    root = ET.parse(model_directory / 'fr3.xml').getroot()
    _attach_franka_hand(root, hand_directory)
    root.find('option').set('timestep', '0.002')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='960')
    # Software OpenGL (for example Xvfb) otherwise spends longer than the
    # feedback freshness interval on multisampling and shadow passes.
    ET.SubElement(visual, 'quality', offsamples='0')
    ET.SubElement(visual, 'headlight', diffuse='.65 .65 .65', ambient='.45 .45 .45', specular='.15 .15 .15')
    ET.SubElement(visual, 'rgba', haze='.96 .97 1 1')
    assets = root.find('asset')
    ET.SubElement(assets, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='.96 .97 1', rgb2='.84 .88 .94', width='256', height='256')
    ET.SubElement(assets, 'material', name='floor', rgba='.92 .94 .97 1', reflectance='.12')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', directional='true', diffuse='.7 .7 .7')
    ET.SubElement(world, 'geom', name='floor', type='plane', pos='0 0 -.025', size='2 2 .1', material='floor')
    # The existing pick-and-place positions provide spatial context. These are
    # fixed landmarks in this reaching-only task, rather than graspable bodies.
    for name, shape, pos, size, rgba in (
        ('pick_cube_red', 'box', '.42 -.12 .0025', '.0225 .0225 .0225', '.88 .2 .18 1'),
        ('pick_cube_green', 'box', '.46 .10 .0025', '.0225 .0225 .0225', '.15 .68 .40 1'),
        ('pick_cup_blue', 'cylinder', '.54 .18 .025', '.032 .05', '.2 .45 .90 1'),
    ):
        ET.SubElement(world, 'geom', name=name, type=shape, pos=pos, size=size, rgba=rgba, contype='0', conaffinity='0')
    return ET.tostring(root, encoding='unicode')


class MuJoCoRobot:
    """A position-actuated FR3 with a measured midpoint between the fingers.

    ``reset`` establishes the shared start pose before timing. During a trial,
    ``command_cartesian`` changes actuator setpoints and ``step`` advances real
    simulated dynamics. Rendering is lazy, so physics tests need no display.
    """

    backend_name = 'mujoco_franka_fr3'
    end_effector_site = 'gripper_center'
    end_effector_frame = 'gripper_center'
    end_effector_reference = 'fingertip_midpoint'
    flange_to_gripper_offset_frame = 'flange_local'

    def __init__(self, home=DIGIT_CUBE_CENTER_M, model_directory=None):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError('MuJoCo is required. Install requirements-teleoperation.txt in the Python environment used by the launcher.') from exc
        self.mujoco = mujoco
        directory = Path(model_directory) if model_directory else MODEL_DIRECTORY
        with zipfile.ZipFile(directory / 'meshes.zip') as archive:
            assets = {name: archive.read(name) for name in archive.namelist()}
        with zipfile.ZipFile(HAND_DIRECTORY / 'meshes.zip') as archive:
            assets.update({name: archive.read(name) for name in archive.namelist()})
        self.model = mujoco.MjModel.from_xml_string(_scene_xml(directory), assets=assets)
        self.data = mujoco.MjData(self.model)
        self._ik_data = mujoco.MjData(self.model)
        self._site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.end_effector_site)
        self._flange_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'attachment_site')
        self._finger_sites = tuple(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
                                   for name in ('left_finger_tip', 'right_finger_tip'))
        self._joint_ranges = self.model.jnt_range[:7].copy()
        self._reference_joints = self.model.key_qpos[0, :7].copy()
        self._reference_rotation = None
        self._renderer = None
        self._render_size = None
        self.camera = mujoco.MjvCamera()
        self.camera.azimuth = 135
        self.camera.elevation = -23
        self.camera.distance = 1.45
        self.camera.lookat[:] = (.30, .0, .35)
        self._remainder_s = 0.0
        self.home = self._vector(home)
        self.command_position = self.home.copy()
        self.reset()

    @staticmethod
    def _vector(position):
        value = np.asarray(position, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError('position must contain three finite coordinates')
        return value.copy()

    @property
    def position(self):
        return self.data.site_xpos[self._site].copy()

    @property
    def flange_position(self):
        return self.data.site_xpos[self._flange_site].copy()

    @property
    def fingertip_positions(self):
        return self.data.site_xpos[list(self._finger_sites)].copy()

    @property
    def gripper_opening_m(self):
        return float(np.linalg.norm(self.fingertip_positions[1] - self.fingertip_positions[0]))

    @property
    def flange_to_gripper_offset_m(self):
        rotation = self.data.site_xmat[self._flange_site].reshape(3, 3)
        return rotation.T @ (self.position - self.flange_position)

    @property
    def sim_time(self):
        return float(self.data.time)

    @property
    def joint_positions(self):
        return self.data.qpos[:7].copy()

    def solve_ik(self, position, initial=None, max_iterations=120):
        """Return a joint target and residual using the model's own Jacobian."""
        target = self._vector(position)
        scratch = self._ik_data
        scratch.qpos[:7] = self.data.qpos[:7] if initial is None else initial
        scratch.qvel[:] = 0
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        for _ in range(max_iterations):
            self.mujoco.mj_forward(self.model, scratch)
            actual = scratch.site_xpos[self._site]
            pos_error = target - actual
            rotation = scratch.site_xmat[self._site].reshape(3, 3)
            rot_error = sum((np.cross(rotation[:, i], self._reference_rotation[:, i]) for i in range(3)), np.zeros(3)) * .5
            if np.linalg.norm(pos_error) < .0002 and np.linalg.norm(rot_error) < .002:
                break
            self.mujoco.mj_jacSite(self.model, scratch, jac_pos, jac_rot, self._site)
            jac = np.vstack((jac_pos[:, :7], .25 * jac_rot[:, :7]))
            error = np.r_[pos_error, .25 * rot_error]
            delta = jac.T @ np.linalg.solve(jac @ jac.T + .002 ** 2 * np.eye(6), error)
            # A small null-space term avoids unnecessary elbow drift.
            null = np.eye(7) - jac.T @ np.linalg.solve(jac @ jac.T + .002 ** 2 * np.eye(6), jac)
            delta += null @ (.02 * (self._reference_joints - scratch.qpos[:7]))
            largest = np.max(np.abs(delta))
            if largest > .12:
                delta *= .12 / largest
            scratch.qpos[:7] = np.clip(scratch.qpos[:7] + delta, self._joint_ranges[:, 0] + .002, self._joint_ranges[:, 1] - .002)
        self.mujoco.mj_forward(self.model, scratch)
        return scratch.qpos[:7].copy(), float(np.linalg.norm(target - scratch.site_xpos[self._site]))

    def reset(self, home=None):
        if home is not None:
            self.home = self._vector(home)
        self.mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.mujoco.mj_forward(self.model, self.data)
        if self._reference_rotation is None:
            self._reference_rotation = self.data.site_xmat[self._site].reshape(3, 3).copy()
        joints, error = self.solve_ik(self.home, initial=self._reference_joints)
        if error > .005:
            raise RuntimeError('The requested start position is unreachable (IK error {:.1f} mm).'.format(error * 1000))
        self.data.qpos[:7] = joints
        self.data.qvel[:] = 0
        self.data.ctrl[:7] = joints
        self.command_position = self.home.copy()
        self._actuator_target = joints.copy()
        self._remainder_s = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def command_cartesian(self, position):
        """Set a bounded Cartesian actuator goal; return the achieved IK error."""
        target = self._vector(position)
        half = DIGIT_CUBE_EDGE_M * .5
        center = np.asarray(DIGIT_CUBE_CENTER_M)
        target = np.clip(target, center - half, center + half)
        joints, error = self.solve_ik(target, initial=self._actuator_target, max_iterations=35)
        self._actuator_target = joints
        self.command_position = target
        return error

    def hold(self):
        """Hold the measured pose when no trial/input is active."""
        self.command_position = self.position
        self._actuator_target = self.data.qpos[:7].copy()

    def step(self, dt):
        dt = float(dt)
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('physics dt must be finite and nonnegative')
        # Bound individual frame advances; the UI detects stale feedback using
        # wall time, so a frozen window cannot earn dwell by catching up physics.
        self._remainder_s += min(dt, .1)
        timestep = float(self.model.opt.timestep)
        steps = int((self._remainder_s + 1e-12) / timestep)
        self._remainder_s -= steps * timestep
        for _ in range(steps):
            difference = self._actuator_target - self.data.ctrl[:7]
            self.data.ctrl[:7] += np.clip(difference, -1.5 * timestep, 1.5 * timestep)
            self.data.qfrc_applied[:7] = self.data.qfrc_bias[:7]
            self.mujoco.mj_step(self.model, self.data)
        self.mujoco.mj_forward(self.model, self.data)
        return self.position

    def _add_geom(self, scene, shape, size, position, rgba):
        if scene.ngeom >= scene.maxgeom:
            return None
        geom = scene.geoms[scene.ngeom]
        self.mujoco.mjv_initGeom(geom, shape, np.asarray(size, dtype=float), np.asarray(position, dtype=float), np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32))
        geom.category = int(self.mujoco.mjtCatBit.mjCAT_DECOR)
        scene.ngeom += 1
        return geom

    def _line(self, scene, first, second, radius, rgba):
        geom = self._add_geom(scene, self.mujoco.mjtGeom.mjGEOM_CAPSULE, (radius, radius, radius), (0, 0, 0), rgba)
        if geom is not None:
            self.mujoco.mjv_connector(geom, self.mujoco.mjtGeom.mjGEOM_CAPSULE, radius, np.asarray(first), np.asarray(second))

    def render(self, width=960, height=640, target=None, tolerance=.025, dwell_progress=0.0):
        """RGB robot scene with a faint target volume for the UI's single rim."""
        width, height = int(width), int(height)
        if width < 32 or height < 32:
            raise ValueError('render dimensions must be at least 32 pixels')
        if self._renderer is None or self._render_size != (width, height):
            if self._renderer is not None:
                self._renderer.close()
            self.model.vis.global_.offwidth = max(1280, width)
            self.model.vis.global_.offheight = max(960, height)
            self._renderer = self.mujoco.Renderer(self.model, height=height, width=width)
            self._render_size = (width, height)
        self._renderer.update_scene(self.data, camera=self.camera)
        scene = self._renderer.scene
        scene.flags[self.mujoco.mjtRndFlag.mjRND_SHADOW] = False
        scene.flags[self.mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        center, half = np.asarray(DIGIT_CUBE_CENTER_M), DIGIT_CUBE_EDGE_M * .5
        corners = [center + half * np.asarray(values) for values in product((-1, 1), repeat=3)]
        for first_index, first in enumerate(corners):
            for second in corners[first_index + 1:]:
                if np.count_nonzero(np.abs(second - first) > .001) == 1:
                    self._line(scene, first, second, .0007, (.29, .40, .58, .24))
        self._add_geom(scene, self.mujoco.mjtGeom.mjGEOM_SPHERE, (.005, 0, 0), self.position, (.05, .80, .63, 1))
        if target is not None:
            target = self._vector(target)
            radius = float(tolerance)
            self._add_geom(scene, self.mujoco.mjtGeom.mjGEOM_SPHERE, (radius, 0, 0), target, (.20, .49, 1, .07))
        return self._renderer.render()

    def _mono_camera(self):
        # mjr_render averages these two eyes for a mono viewport. Using the
        # left eye alone offsets overlays by several pixels from the scene.
        return self.mujoco.mjv_averageCamera(self._renderer.scene.camera[0],
                                           self._renderer.scene.camera[1])

    def project_world(self, position, width=None, height=None):
        """Return target pixel coordinates after render, or None behind camera."""
        if self._renderer is None:
            return None
        width, height = self._render_size if width is None or height is None else (width, height)
        camera = self._mono_camera()
        delta = self._vector(position) - np.asarray(camera.pos)
        forward, up = np.asarray(camera.forward), np.asarray(camera.up)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        depth = float(delta @ forward)
        if depth <= 0:
            return None
        near = float(camera.frustum_near)
        top, bottom = float(camera.frustum_top), float(camera.frustum_bottom)
        half_height = (top - bottom) * .5
        half_width = half_height * float(width) / float(height)
        scale = 1.0 if camera.orthographic else near / depth
        projected_x, projected_y = float(delta @ right) * scale, float(delta @ up) * scale
        centre_x = float(camera.frustum_center)
        return (((projected_x - centre_x) / half_width + 1) * width * .5,
                (top - projected_y) / (top - bottom) * height)

    def pixel_to_workspace(self, pixel_xy, height_m, width=None, height=None):
        """Intersect a displayed camera pixel with the selected horizontal plane.

        The caller supplies pixel coordinates relative to the rendered image,
        rather than the containing GUI canvas. The returned world point is not
        clamped, so input handlers can apply the collection workspace bounds.
        Pixels whose camera rays miss the plane in front of the camera return
        ``None``. This uses the same mono camera as :meth:`project_world`.
        """
        pixel = np.asarray(pixel_xy, dtype=float)
        plane_height = float(height_m)
        if pixel.shape != (2,) or not np.all(np.isfinite(pixel)) or not math.isfinite(plane_height):
            raise ValueError('pixel and height must contain finite coordinates')
        if self._renderer is None:
            return None
        width, height = self._render_size if width is None or height is None else (float(width), float(height))
        if not all(math.isfinite(value) and value > 0 for value in (width, height)):
            raise ValueError('image dimensions must be finite and positive')
        camera = self._mono_camera()
        origin = np.asarray(camera.pos, dtype=float).copy()
        forward, up = np.asarray(camera.forward, dtype=float), np.asarray(camera.up, dtype=float)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        top, bottom = float(camera.frustum_top), float(camera.frustum_bottom)
        half_width = (top - bottom) * .5 * width / height
        horizontal = float(camera.frustum_center) + (2 * pixel[0] / width - 1) * half_width
        vertical = top - pixel[1] / height * (top - bottom)
        image_offset = right * horizontal + up * vertical
        if camera.orthographic:
            origin += image_offset
            direction = forward
        else:
            direction = forward * float(camera.frustum_near) + image_offset
        if abs(direction[2]) < 1e-12:
            return None
        distance = (plane_height - origin[2]) / direction[2]
        if distance <= 0:
            return None
        point = origin + distance * direction
        # Keep the selected input height exact despite floating-point ray math.
        point[2] = plane_height
        return point

    def projected_radius(self, position, radius_m, width=None, height=None):
        """Pixel radius in the camera's image plane for a physical margin."""
        if self._renderer is None:
            return None
        radius_m = float(radius_m)
        if not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError('projected radius must be finite and positive')
        point = self._vector(position)
        up = np.asarray(self._mono_camera().up, dtype=float)
        up /= np.linalg.norm(up)
        center = self.project_world(point, width, height)
        edge = self.project_world(point + radius_m * up, width, height)
        if center is None or edge is None:
            return None
        return float(np.linalg.norm(np.asarray(edge) - np.asarray(center)))

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
            self._render_size = None

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()
