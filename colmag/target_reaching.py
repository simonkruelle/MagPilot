"""Reproducible target-reaching trials and logs, independent of ROS and the GUI.

Positions are measured flange positions in metres in the declared robot base
frame. A target is reached only after continuous, fresh feedback stays within
the tolerance for the dwell interval. Wall time includes the dwell interval.
"""

import csv
import json
import math
import os
from dataclasses import asdict, dataclass
from datetime import datetime, timezone

from colmag.robot_targets import DIGIT_CUBE_CENTER_M, digit_cube_target


@dataclass(frozen=True)
class TrialSettings:
    tolerance_m: float = 0.020
    dwell_s: float = 0.5
    timeout_s: float = 60.0
    start_position_m: tuple = DIGIT_CUBE_CENTER_M
    start_tolerance_m: float = 0.020
    max_feedback_gap_s: float = 0.25

    def __post_init__(self):
        for name in ('tolerance_m', 'dwell_s', 'timeout_s',
                     'start_tolerance_m', 'max_feedback_gap_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('{} must be finite and positive'.format(name))
        if self.timeout_s <= self.dwell_s:
            raise ValueError('timeout_s must be greater than dwell_s')
        position(self.start_position_m)


def position(values):
    point = tuple(float(value) for value in values)
    if len(point) != 3 or not all(math.isfinite(value) for value in point):
        raise ValueError('A position must contain three finite coordinates.')
    return point


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def default_targets(repetitions=1):
    """Eight existing digit-cube corners; centre is the common starting point."""
    if repetitions < 1:
        raise ValueError('repetitions must be positive')
    return [dict(target_id='corner_{}'.format(digit), repetition=rep,
                 position_m=digit_cube_target(digit))
            for rep in range(1, repetitions + 1) for digit in range(1, 9)]


def require_simulation(use_sim_time, nodes):
    """Refuse real-arm or ambiguous feedback sources before observing trials."""
    names = [name.rsplit('/', 1)[-1] for name in nodes]
    if any(name.startswith('franka_control') for name in names):
        raise RuntimeError('A real franka_control node is active. Use a separate simulation.')
    if not use_sim_time or 'gazebo' not in names:
        raise RuntimeError('Start the Gazebo robot in Simulation mode first.')


class TargetTrial:
    """One measured trial; the caller supplies fresh actual end-effector samples."""

    def __init__(self, target, settings, start_wall_s, start_sim_s, start_position):
        self.target = dict(target)
        self.target['position_m'] = position(target['position_m'])
        self.settings = settings
        self.start_position = position(start_position)
        if distance(self.start_position, settings.start_position_m) > settings.start_tolerance_m:
            raise ValueError('Return the flange to the common starting position first.')
        self.start_wall_s = float(start_wall_s)
        self.start_sim_s = float(start_sim_s)
        if not all(math.isfinite(value) for value in (self.start_wall_s, self.start_sim_s)):
            raise ValueError('Start times must be finite.')
        self.last_wall_s = self.start_wall_s
        self.last_sim_s = self.start_sim_s
        self.last_position = self.start_position
        self.inside_since = None
        self.status = 'running'
        self.path_length_m = 0.0
        self.rows = []
        self.result = None

    def update(self, wall_s, sim_s, actual_position):
        if self.status != 'running':
            return None
        wall_s, sim_s = float(wall_s), float(sim_s)
        point = position(actual_position)
        if not all(math.isfinite(v) for v in (wall_s, sim_s)):
            raise ValueError('Sample times must be finite.')
        if wall_s <= self.last_wall_s:
            raise ValueError('Wall timestamps must increase.')
        if sim_s < self.last_sim_s - 1e-6:
            return self.finish('clock_reset', wall_s, sim_s)
        if wall_s - self.last_wall_s > self.settings.max_feedback_gap_s:
            self.inside_since = None
        # A paused clock cannot count toward successful target dwell.
        sim_advanced = sim_s > self.last_sim_s
        if not sim_advanced:
            self.inside_since = None
        self.path_length_m += distance(self.last_position, point)
        self.last_wall_s, self.last_sim_s, self.last_position = wall_s, sim_s, point
        error = distance(point, self.target['position_m'])
        self.rows.append(dict(wall_elapsed_s=wall_s - self.start_wall_s,
                              sim_elapsed_s=sim_s - self.start_sim_s,
                              x_m=point[0], y_m=point[1], z_m=point[2],
                              target_error_m=error))
        # Reaching at the deadline is a timeout, even if the dwell threshold
        # becomes true on the same sample. Preserve the measured endpoint.
        if wall_s - self.start_wall_s >= self.settings.timeout_s:
            return self.finish('timed_out', wall_s, sim_s)
        if not sim_advanced:
            return None
        if error <= self.settings.tolerance_m:
            if self.inside_since is None:
                self.inside_since = wall_s
            if wall_s - self.inside_since >= self.settings.dwell_s:
                return self.finish('completed', wall_s, sim_s)
        else:
            self.inside_since = None
        return None

    def finish(self, status, wall_s, sim_s):
        if self.status != 'running':
            return self.result
        if status not in ('completed', 'cancelled', 'timed_out', 'feedback_lost',
                          'clock_reset', 'simulation_stopped'):
            raise ValueError('Unknown trial status: {}'.format(status))
        self.status = status
        duration = max(0.0, float(wall_s) - self.start_wall_s)
        self.result = dict(
            target_id=self.target['target_id'], repetition=self.target['repetition'],
            status=status, target_x_m=self.target['position_m'][0],
            target_y_m=self.target['position_m'][1], target_z_m=self.target['position_m'][2],
            start_x_m=self.start_position[0], start_y_m=self.start_position[1],
            start_z_m=self.start_position[2], end_x_m=self.last_position[0],
            end_y_m=self.last_position[1], end_z_m=self.last_position[2],
            wall_duration_s=duration,
            completion_time_s=duration if status == 'completed' else '',
            sim_duration_s=max(0.0, float(sim_s) - self.start_sim_s),
            endpoint_error_m=distance(self.last_position, self.target['position_m']),
            path_length_m=self.path_length_m,
            mean_speed_m_s=self.path_length_m / duration if duration else 0.0,
            samples=len(self.rows),
        )
        return self.result


SUMMARY_FIELDS = [
    'trial', 'condition', 'participant_id', 'target_id', 'repetition', 'status',
    'target_x_m', 'target_y_m', 'target_z_m', 'start_x_m', 'start_y_m', 'start_z_m',
    'end_x_m', 'end_y_m', 'end_z_m', 'wall_duration_s', 'completion_time_s',
    'sim_duration_s', 'endpoint_error_m', 'path_length_m', 'mean_speed_m_s',
    'samples', 'trajectory_csv',
]
TRAJECTORY_FIELDS = ['wall_elapsed_s', 'sim_elapsed_s', 'x_m', 'y_m', 'z_m', 'target_error_m']


class PilotRun:
    """Create a new run exclusively and retain unsuccessful trials in its summary."""

    def __init__(self, output_dir, run_id, participant_id, condition, settings,
                 targets, base_frame='fr3_link0', ee_frame='fr3_link8', metadata=None):
        self.output_dir = os.path.abspath(output_dir)
        os.makedirs(self.output_dir, exist_ok=False)
        self.summary_path = os.path.join(self.output_dir, 'summary.csv')
        self.manifest_path = os.path.join(self.output_dir, 'manifest.json')
        self.completed = []
        self.participant_id = participant_id
        self.condition = condition
        self.manifest = dict(
            schema_version=1, task='virtual_target_reaching_pilot', run_id=run_id,
            created_utc=datetime.now(timezone.utc).isoformat(),
            participant_id=participant_id, condition=condition,
            source=('plan_only' if (metadata or {}).get('dry_run') else 'gazebo_measured_tf'),
            robot_base_frame=base_frame,
            end_effector_frame=ee_frame, end_effector_reference='flange',
            position_units='m', elapsed_time_units='s',
            timing='monotonic wall time from operator start through target dwell',
            settings=asdict(settings), targets=targets,
            metadata=metadata or {}, trials=self.completed,
        )
        self._write_manifest()
        with open(self.summary_path, 'w', newline='', encoding='utf-8') as stream:
            csv.DictWriter(stream, SUMMARY_FIELDS).writeheader()

    def _write_manifest(self):
        temporary = self.manifest_path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(self.manifest, stream, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, self.manifest_path)

    def save_trial(self, trial):
        if trial.status == 'running':
            raise ValueError('Finish the trial before saving it.')
        number = len(self.completed) + 1
        filename = 'trial_{:03d}_trajectory.csv'.format(number)
        with open(os.path.join(self.output_dir, filename), 'x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, TRAJECTORY_FIELDS)
            writer.writeheader()
            writer.writerows(trial.rows)
        row = dict(trial.result, trial=number, condition=self.condition,
                   participant_id=self.participant_id, trajectory_csv=filename)
        with open(self.summary_path, 'a', newline='', encoding='utf-8') as stream:
            csv.DictWriter(stream, SUMMARY_FIELDS).writerow(row)
        self.completed.append(row)
        self._write_manifest()
        return row
