"""Measured MuJoCo target-reaching trials, independent of the GUI and hardware.

The actual midpoint between the simulated fingertips determines success.
Commanded positions are logged
separately and never contribute to the dwell timer. All elapsed measurements use
monotonic wall time; simulation time is also retained, so pauses and resets are
visible rather than counted as successful target holds.
"""

import copy
import csv
import json
import math
import os
import random
import shutil
import tempfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone

from colmag.robot_targets import DIGIT_CUBE_CENTER_M, DIGIT_CUBE_EDGE_M
from colmag.target_reaching import distance, position


PROTOCOL_ID = 'mujoco_random_target_reaching_v2'
END_EFFECTOR_FRAME = 'gripper_center'
END_EFFECTOR_SITE = 'gripper_center'
END_EFFECTOR_REFERENCE = 'fingertip_midpoint'


@dataclass(frozen=True)
class TeleoperationSettings:
    tolerance_m: float = 0.025
    dwell_s: float = 2.0
    timeout_s: float = 60.0
    max_feedback_gap_s: float = 0.25
    require_common_start: bool = True
    start_position_m: tuple = DIGIT_CUBE_CENTER_M
    start_tolerance_m: float = 0.025
    workspace_center_m: tuple = DIGIT_CUBE_CENTER_M
    workspace_edge_m: float = DIGIT_CUBE_EDGE_M

    def __post_init__(self):
        for name in ('tolerance_m', 'dwell_s', 'timeout_s',
                     'max_feedback_gap_s', 'start_tolerance_m', 'workspace_edge_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('{} must be finite and positive'.format(name))
        if self.timeout_s <= self.dwell_s:
            raise ValueError('timeout_s must be greater than dwell_s')
        if self.tolerance_m >= self.workspace_edge_m / 2:
            raise ValueError('The target tolerance must fit inside the workspace.')
        object.__setattr__(self, 'start_position_m', position(self.start_position_m))
        object.__setattr__(self, 'workspace_center_m', position(self.workspace_center_m))
        if not isinstance(self.require_common_start, bool):
            raise ValueError('require_common_start must be a boolean')


def random_targets(count, seed, settings=None, min_distance_m=None):
    """Reproduce target centres uniformly within the declared allowed volume.

    The tolerance sphere fits inside the existing 24 cm digit cube. Rejection
    sampling excludes centres within two tolerance radii of the common start,
    avoiding trials that can be completed without navigating the robot. It does
    not preferentially select corners or reorder targets by difficulty.
    """
    settings = settings or TeleoperationSettings()
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('count must be a positive integer')
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError('seed must be an integer')
    minimum = 2 * settings.tolerance_m if min_distance_m is None else float(min_distance_m)
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError('min_distance_m must be finite and nonnegative')
    half = settings.workspace_edge_m / 2 - settings.tolerance_m
    bounds = [(coordinate - half, coordinate + half)
              for coordinate in settings.workspace_center_m]
    farthest = tuple(max(pair, key=lambda value: abs(value - start))
                     for pair, start in zip(bounds, settings.start_position_m))
    if distance(farthest, settings.start_position_m) <= minimum:
        raise ValueError('No workspace remains outside the minimum start distance.')
    generator = random.Random(seed)
    targets = []
    for index in range(1, count + 1):
        for attempt in range(10000):
            point = tuple(generator.uniform(low, high) for low, high in bounds)
            if distance(point, settings.start_position_m) >= minimum:
                break
        else:
            raise ValueError('The minimum start distance leaves too little workspace.')
        targets.append(dict(target_id='random_{:03d}'.format(index),
                            target_index=index, repetition=1, position_m=point))
    return targets


TRAJECTORY_FIELDS = [
    'wall_monotonic_s', 'simulation_time_s', 'wall_elapsed_s', 'sim_elapsed_s',
    'actual_x_m', 'actual_y_m', 'actual_z_m',
    'commanded_x_m', 'commanded_y_m', 'commanded_z_m',
    'input_sample_json',
    'target_x_m', 'target_y_m', 'target_z_m', 'target_error_m',
    'inside_tolerance', 'dwell_progress',
]


class TeleoperationTrial:
    """One operator-started trial receiving fresh simulated gripper-centre positions."""

    def __init__(self, target, settings, start_wall_s, start_sim_s, start_position):
        self.settings = settings
        self.target = copy.deepcopy(target)
        self.target['position_m'] = position(target['position_m'])
        self.target.setdefault('target_id', 'target')
        self.target.setdefault('repetition', 1)
        half = settings.workspace_edge_m / 2 - settings.tolerance_m
        if any(abs(value - centre) > half + 1e-12 for value, centre
               in zip(self.target['position_m'], settings.workspace_center_m)):
            raise ValueError('The target sphere must fit inside the workspace.')
        self.start_position = position(start_position)
        if (settings.require_common_start and
                distance(self.start_position, settings.start_position_m) >
                settings.start_tolerance_m + 1e-12):
            raise ValueError('Return the gripper centre to the common starting position first.')
        self.start_wall_s = float(start_wall_s)
        self.start_sim_s = float(start_sim_s)
        if not all(math.isfinite(value) for value in (self.start_wall_s, self.start_sim_s)):
            raise ValueError('Start times must be finite.')
        self.last_wall_s = self.start_wall_s
        self.last_sim_s = self.start_sim_s
        self.last_position = self.start_position
        self.last_commanded_position = None
        self.inside_since = None
        self.first_entry_wall_s = None
        self.status = 'running'
        self.path_length_m = 0.0
        self.rows = []
        self.result = None
        self._append_row(self.start_wall_s, self.start_sim_s,
                         self.start_position, None)

    @property
    def actual_position(self):
        return self.last_position

    @property
    def error_m(self):
        return distance(self.last_position, self.target['position_m'])

    @property
    def progress(self):
        return self.dwell_progress()

    def dwell_progress(self, wall_s=None):
        """Display measured progress; never advance it using render time alone."""
        if self.status == 'completed':
            return 1.0
        if wall_s is not None:
            now = float(wall_s)
            if not math.isfinite(now):
                raise ValueError('Wall time must be finite.')
            if now - self.last_wall_s > self.settings.max_feedback_gap_s:
                self.inside_since = None
        if self.inside_since is None:
            return 0.0
        return min(1.0, max(0.0, (self.last_wall_s - self.inside_since) /
                            self.settings.dwell_s))

    def _append_row(self, wall_s, sim_s, point, command, input_sample_json=''):
        target = self.target['position_m']
        command = command or ('', '', '')
        self.rows.append(dict(
            wall_monotonic_s=wall_s, simulation_time_s=sim_s,
            wall_elapsed_s=wall_s - self.start_wall_s,
            sim_elapsed_s=sim_s - self.start_sim_s,
            actual_x_m=point[0], actual_y_m=point[1], actual_z_m=point[2],
            commanded_x_m=command[0], commanded_y_m=command[1], commanded_z_m=command[2],
            input_sample_json=input_sample_json,
            target_x_m=target[0], target_y_m=target[1], target_z_m=target[2],
            target_error_m=distance(point, target),
            inside_tolerance=distance(point, target) <= self.settings.tolerance_m + 1e-12,
            dwell_progress=self.dwell_progress(),
        ))

    def update(self, wall_s, sim_s, actual_position, commanded_position=None,
               input_sample=None):
        if self.status != 'running':
            return None
        wall_s, sim_s = float(wall_s), float(sim_s)
        point = position(actual_position)
        command = None if commanded_position is None else position(commanded_position)
        raw_input = ('' if input_sample is None else
                     json.dumps(input_sample, sort_keys=True, allow_nan=False))
        if not all(math.isfinite(value) for value in (wall_s, sim_s)):
            raise ValueError('Sample times must be finite.')
        if wall_s <= self.last_wall_s:
            raise ValueError('Wall timestamps must increase.')
        clock_reset = sim_s < self.last_sim_s - 1e-6
        advanced = sim_s > self.last_sim_s
        if wall_s - self.last_wall_s > self.settings.max_feedback_gap_s or not advanced:
            self.inside_since = None
        self.path_length_m += distance(self.last_position, point)
        self.last_wall_s, self.last_sim_s, self.last_position = wall_s, sim_s, point
        self.last_commanded_position = command
        if advanced and not clock_reset and self.error_m <= self.settings.tolerance_m + 1e-12:
            if self.inside_since is None:
                self.inside_since = wall_s
            if self.first_entry_wall_s is None:
                self.first_entry_wall_s = wall_s
        else:
            self.inside_since = None
        self._append_row(wall_s, sim_s, point, command, raw_input)
        if clock_reset:
            return self.finish('clock_reset', wall_s, sim_s)
        # The deadline wins over a dwell that completes on the same sample.
        if wall_s - self.start_wall_s >= self.settings.timeout_s:
            return self.finish('timed_out', wall_s, sim_s)
        if self.inside_since is not None and self.dwell_progress() >= 1.0:
            return self.finish('completed', wall_s, sim_s)
        return None

    def tick(self, wall_s, sim_s=None):
        """Poll while no fresh sample is available; stale feedback resets dwell."""
        if self.status != 'running':
            return None
        wall_s = float(wall_s)
        if not math.isfinite(wall_s) or wall_s < self.last_wall_s:
            raise ValueError('Polling time must be finite and at least the last sample time.')
        self.dwell_progress(wall_s)
        if wall_s - self.start_wall_s >= self.settings.timeout_s:
            return self.finish('timed_out', wall_s, sim_s)
        return None

    def finish(self, status, wall_s, sim_s=None):
        if self.status != 'running':
            return self.result
        if status not in ('completed', 'cancelled', 'timed_out', 'feedback_lost',
                          'clock_reset', 'simulation_stopped'):
            raise ValueError('Unknown trial status: {}'.format(status))
        wall_s = float(wall_s)
        sim_s = self.last_sim_s if sim_s is None else float(sim_s)
        if (not all(math.isfinite(value) for value in (wall_s, sim_s)) or
                wall_s < self.last_wall_s):
            raise ValueError('Finish times must be finite and preserve monotonic wall time.')
        if status == 'completed' and (self.dwell_progress(wall_s) < 1.0 or
                                      self.error_m > self.settings.tolerance_m + 1e-12):
            raise ValueError('Completion requires uninterrupted measured target dwell.')
        duration = wall_s - self.start_wall_s
        held = (self.last_wall_s - self.inside_since
                if self.inside_since is not None else 0.0)
        self.status = status
        self.result = dict(
            target_id=self.target['target_id'], repetition=self.target['repetition'],
            status=status, target_x_m=self.target['position_m'][0],
            target_y_m=self.target['position_m'][1], target_z_m=self.target['position_m'][2],
            start_x_m=self.start_position[0], start_y_m=self.start_position[1],
            start_z_m=self.start_position[2], end_x_m=self.last_position[0],
            end_y_m=self.last_position[1], end_z_m=self.last_position[2],
            started_wall_monotonic_s=self.start_wall_s,
            finished_wall_monotonic_s=wall_s,
            started_simulation_time_s=self.start_sim_s,
            finished_simulation_time_s=sim_s,
            wall_duration_s=duration,
            completion_time_s=duration if status == 'completed' else '',
            sim_duration_s=max(0.0, sim_s - self.start_sim_s),
            first_entry_time_s=(self.first_entry_wall_s - self.start_wall_s
                                if self.first_entry_wall_s is not None else ''),
            held_duration_s=held,
            endpoint_error_m=self.error_m,
            path_length_m=self.path_length_m,
            mean_speed_m_s=self.path_length_m / duration if duration else 0.0,
            samples=len(self.rows),
        )
        return self.result


RESULT_FIELDS = [
    'target_id', 'repetition', 'status', 'target_x_m', 'target_y_m', 'target_z_m',
    'start_x_m', 'start_y_m', 'start_z_m', 'end_x_m', 'end_y_m', 'end_z_m',
    'started_wall_monotonic_s', 'finished_wall_monotonic_s',
    'started_simulation_time_s', 'finished_simulation_time_s',
    'wall_duration_s', 'completion_time_s', 'sim_duration_s', 'first_entry_time_s',
    'held_duration_s', 'endpoint_error_m', 'path_length_m', 'mean_speed_m_s', 'samples',
]
SUMMARY_FIELDS = [
    'trial', 'participant_id', 'participant_name', 'session_id', 'experiment_name',
    'input_source', 'magnet_count',
] + RESULT_FIELDS + ['trajectory_csv', 'result_json']


class TeleoperationRun:
    """Exclusive session directory with frozen, matching trial artefacts.

    The manifest is published last and is the commit marker. Each trajectory
    and result are created from the same frozen trial, including failed trials.
    """

    def __init__(self, output_dir, participant_id, session_id, settings, targets,
                 experiment_name='', participant_name='', input_source='trackpad',
                 magnet_count=None, seed=0, metadata=None, *,
                 end_effector_frame=END_EFFECTOR_FRAME,
                 end_effector_site=END_EFFECTOR_SITE,
                 end_effector_reference=END_EFFECTOR_REFERENCE):
        reference = dict(end_effector_frame=end_effector_frame,
                         end_effector_site=end_effector_site,
                         end_effector_reference=end_effector_reference)
        expected = dict(end_effector_frame=END_EFFECTOR_FRAME,
                        end_effector_site=END_EFFECTOR_SITE,
                        end_effector_reference=END_EFFECTOR_REFERENCE)
        if reference != expected:
            raise ValueError('Protocol v2 measures the gripper_center fingertip midpoint; '
                             'flange recordings belong to their original protocol.')
        if input_source not in ('serial', 'trackpad'):
            raise ValueError('input_source must be serial or trackpad')
        if (magnet_count is not None and (isinstance(magnet_count, bool) or
                not isinstance(magnet_count, int) or magnet_count < 0)):
            raise ValueError('magnet_count must be a nonnegative integer or unknown')
        self.output_dir = os.path.abspath(output_dir)
        self.summary_path = os.path.join(self.output_dir, 'summary.csv')
        self.manifest_path = os.path.join(self.output_dir, 'manifest.json')
        self.completed = []
        self.settings = settings
        self.targets = copy.deepcopy(targets)
        self.participant_id = participant_id
        self.participant_name = participant_name
        self.session_id = session_id
        self.experiment_name = experiment_name
        self.input_source = input_source
        self.magnet_count = magnet_count
        self.end_effector_identity = reference
        self.manifest = dict(
            schema_version=1, protocol=PROTOCOL_ID,
            task='virtual_random_target_reaching', source='mujoco_measured_gripper_center',
            created_utc=datetime.now(timezone.utc).isoformat(),
            participant_id=participant_id, participant_name=participant_name,
            session_id=session_id, experiment_name=experiment_name,
            input_source=input_source, magnet_count=magnet_count, seed=seed,
            robot_base_frame='fr3_link0', **reference,
            end_effector_description='Midpoint between the two simulated fingertips',
            position_units='m', elapsed_time_units='s',
            timing='monotonic wall time from Enter through uninterrupted measured dwell',
            completion_includes_dwell=True,
            target_sampling='uniform within tolerance-inset cube, excluding start distance',
            settings=asdict(settings), targets=self.targets,
            metadata=copy.deepcopy(metadata or {}), trials=self.completed,
        )
        # Validate serialisability before creating a session directory.
        json.dumps(self.manifest, allow_nan=False)
        os.makedirs(self.output_dir, exist_ok=False)
        self._write_csv(self.summary_path, SUMMARY_FIELDS, [])
        self._write_json(self.manifest_path, self.manifest)

    @staticmethod
    def _write_csv(path, fields, rows):
        with open(path, 'x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())

    @staticmethod
    def _write_json(path, data):
        with open(path, 'x', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())

    def save_trial(self, trial):
        if trial.status == 'running' or trial.result is None:
            raise ValueError('Finish the trial before saving it.')
        if trial.settings != self.settings:
            raise ValueError('Trial settings do not match the session protocol.')
        if getattr(trial, '_saved_run', None) is not None:
            raise ValueError('This trial has already been saved.')
        number = len(self.completed) + 1
        trace_name = 'trial_{:03d}_trajectory.csv'.format(number)
        result_name = 'trial_{:03d}_result.json'.format(number)
        trace_path = os.path.join(self.output_dir, trace_name)
        result_path = os.path.join(self.output_dir, result_name)
        if os.path.exists(trace_path) or os.path.exists(result_path):
            raise FileExistsError('Trial artefacts already exist; use a new session.')
        frozen_rows = copy.deepcopy(trial.rows)
        row = dict(copy.deepcopy(trial.result), trial=number,
                   participant_id=self.participant_id, participant_name=self.participant_name,
                   session_id=self.session_id, experiment_name=self.experiment_name,
                   input_source=self.input_source, magnet_count=self.magnet_count,
                   trajectory_csv=trace_name, result_json=result_name)
        completed = self.completed + [row]
        manifest = dict(self.manifest, trials=completed)
        pending = tempfile.mkdtemp(prefix='.trial.', dir=self.output_dir)
        installed = []
        try:
            self._write_csv(os.path.join(pending, trace_name), TRAJECTORY_FIELDS, frozen_rows)
            self._write_json(os.path.join(pending, result_name),
                             dict(row, protocol=PROTOCOL_ID, target=trial.target,
                                  settings=asdict(self.settings), **self.end_effector_identity))
            self._write_csv(os.path.join(pending, 'summary.csv'), SUMMARY_FIELDS, completed)
            self._write_json(os.path.join(pending, 'manifest.json'), manifest)
            # Keep the prior summary available until the manifest commits.
            shutil.copyfile(self.summary_path, os.path.join(pending, 'summary.previous.csv'))
            for name, destination in ((trace_name, trace_path), (result_name, result_path)):
                os.replace(os.path.join(pending, name), destination)
                installed.append(destination)
            os.replace(os.path.join(pending, 'summary.csv'), self.summary_path)
            os.replace(os.path.join(pending, 'manifest.json'), self.manifest_path)
        except Exception:
            previous = os.path.join(pending, 'summary.previous.csv')
            if os.path.exists(previous):
                os.replace(previous, self.summary_path)
            for path in installed:
                os.unlink(path)
            raise
        finally:
            shutil.rmtree(pending, ignore_errors=True)
        self.completed.append(row)
        self.manifest['trials'] = self.completed
        trial._saved_run = self.output_dir
        return row
