"""Participants and coverage counts for the MagPilot data collection.

Everything lives in one folder next to the code that git ignores:

    data_collection/
      participants.json                  who took part (IDs and optional names)
      collection_settings.json           last selected experiment name
      characters/P03/S01/...             magnetometer_reader.py --record-data output
      demo/characters/P03/S01/...        trackpad rehearsal, separate from board data
      tracking_error/<run_id>/...        tools/record_tracking_error.py output

The launcher's Data window uses these functions. They only use the standard
library, so they can be tested without a display or the sensor board.
"""

import csv
import hashlib
import json
import math
import os
import re
import tempfile
from datetime import date
from functools import lru_cache

from colmag.collection_protocol import DIGITS, LETTERS, TARGET_REPS


DATA_DIR_NAME = 'data_collection'
DEMO_DIR = 'demo'
CHARACTERS_DIR = 'characters'
TRACKING_DIR = 'tracking_error'
TELEOPERATION_DIR = 'teleoperation'
VIRTUAL_TASK_DIR = 'virtual_task'
PARTICIPANTS_FILE = 'participants.json'
COLLECTION_SETTINGS_FILE = 'collection_settings.json'
SCHEMA_VERSION = 1

HEIGHTS_MM = (0, 10, 50, 100, 150, 200)
HANDS = ('right', 'left')
AGE_BANDS = ('not given', '18-24', '25-34', '35-44', '45-54', '55+')

PARTICIPANT_ID = re.compile(r'^P\d{2,}$')
SESSION_ID = re.compile(r'^S\d{2,}$')


# ── Participants ────────────────────────────────────────────────────────────

def load_participants(data_dir):
    """Return the participant list; an empty list if the file does not exist.

    A damaged file raises ValueError instead of being silently reset, so no
    participant is ever lost by saving over it.
    """
    path = os.path.join(data_dir, PARTICIPANTS_FILE)
    if not os.path.exists(path):
        return []
    try:
        with open(path, 'r', encoding='utf-8') as f:
            payload = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError('{} cannot be read: {}'.format(path, exc))
    if not isinstance(payload, dict) or payload.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('{} is not a version {} participants file'.format(
            path, SCHEMA_VERSION))
    participants = payload.get('participants')
    if not isinstance(participants, list) or not all(
            isinstance(p, dict) and PARTICIPANT_ID.match(str(p.get('participant_id')))
            for p in participants):
        raise ValueError('{} has a participant without a valid ID'.format(path))
    return participants


def save_participants(data_dir, participants):
    """Write participants.json atomically, sorted by ID."""
    os.makedirs(data_dir, exist_ok=True)
    payload = {
        'schema_version': SCHEMA_VERSION,
        'participants': sorted(participants, key=lambda p: p['participant_id']),
    }
    fd, temporary_path = tempfile.mkstemp(
        prefix='.participants.', suffix='.tmp', dir=data_dir)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary_path, os.path.join(data_dir, PARTICIPANTS_FILE))
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


def next_participant_id(participants, seen_ids=()):
    """Return the next free ID (P01, P02, ...), never reusing one seen before."""
    numbers = [0]
    for participant_id in [p['participant_id'] for p in participants] + list(seen_ids):
        if participant_id and PARTICIPANT_ID.match(participant_id):
            numbers.append(int(participant_id[1:]))
    return 'P{:02d}'.format(max(numbers) + 1)


def new_participant(participant_id, name=''):
    """Return a fresh participant record with default values."""
    return {
        'participant_id': participant_id,
        'name': name.strip(),
        'created_at': date.today().isoformat(),
        'handedness': HANDS[0],
        'age_band': AGE_BANDS[0],
        'consent': False,
        'consent_date': None,
        'excluded': False,
        'notes': '',
    }


def participant_label(record):
    """Display a participant's optional name with their stable folder ID."""
    participant_id = record['participant_id']
    name = record.get('name', '')
    name = name.strip() if isinstance(name, str) else ''
    return '{} ({})'.format(name, participant_id) if name else participant_id


# ── Collection settings ────────────────────────────────────────────────────

def load_collection_settings(data_dir):
    """Return the last experiment name; reject damaged settings files."""
    path = os.path.join(data_dir, COLLECTION_SETTINGS_FILE)
    if not os.path.exists(path):
        return {'experiment_name': ''}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            settings = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError('{} cannot be read: {}'.format(path, exc))
    if not isinstance(settings, dict) or not isinstance(
            settings.get('experiment_name'), str):
        raise ValueError('{} must contain a string experiment_name'.format(path))
    return {'experiment_name': settings['experiment_name']}


def save_collection_settings(data_dir, experiment_name):
    """Atomically remember an experiment name without replacing corrupt input."""
    if not isinstance(experiment_name, str):
        raise ValueError('experiment_name must be a string')
    load_collection_settings(data_dir)
    os.makedirs(data_dir, exist_ok=True)
    settings = {'experiment_name': experiment_name.strip()}
    fd, temporary_path = tempfile.mkstemp(
        prefix='.collection_settings.', suffix='.tmp', dir=data_dir)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(settings, f, indent=2)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary_path, os.path.join(data_dir, COLLECTION_SETTINGS_FILE))
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise
    return settings


# ── Recorded characters ─────────────────────────────────────────────────────

def character_of(label):
    """'digit_3' -> '3', 'letter_A' -> 'A'; None for anything outside the dataset."""
    kind, _, char = str(label).partition('_')
    if kind == 'digit' and char in DIGITS:
        return char
    if kind == 'letter' and char in LETTERS:
        return char
    return None


def collection_data_dir(data_dir, demo=False):
    """Select the board dataset or its separate trackpad rehearsal root."""
    return os.path.join(data_dir, DEMO_DIR) if demo else data_dir


def session_output_dir(participant_id, session_id, demo=False):
    """Recorder output folder for one session, relative to the repo root."""
    return os.path.join(collection_data_dir(DATA_DIR_NAME, demo),
                        CHARACTERS_DIR, participant_id, session_id)


def next_session_id(data_dir, participant_id):
    """Return the next free session folder name for a participant (S01, S02, ...)."""
    participant_dir = os.path.join(data_dir, CHARACTERS_DIR, participant_id)
    numbers = [0]
    if os.path.isdir(participant_dir):
        for name in os.listdir(participant_dir):
            if SESSION_ID.match(name):
                numbers.append(int(name[1:]))
    return 'S{:02d}'.format(max(numbers) + 1)


def scan_samples(data_dir, input_source='serial'):
    """Read one dataset root's takes, accepting only the selected input source.

    Returns (samples, problems). Each sample is a dict with participant_id,
    session_id, height_mm and char. problems counts what was skipped, so the
    Data window can say why a take is missing from the table. Pass the demo
    root and input_source='touchpad' for rehearsal coverage; the default root
    only descends characters/, so demo recordings never affect board coverage.
    """
    samples = []
    problems = {}
    seen_basenames = set()

    def skip(reason):
        problems[reason] = problems.get(reason, 0) + 1

    root = os.path.join(data_dir, CHARACTERS_DIR)
    for folder, _, filenames in os.walk(root):
        if os.path.basename(os.path.dirname(folder)) != 'samples':
            continue
        for filename in sorted(filenames):
            if not filename.endswith('.json'):
                continue
            json_path = os.path.join(folder, filename)
            try:
                with open(json_path, 'r', encoding='utf-8') as f:
                    take = json.load(f)
            except (OSError, json.JSONDecodeError):
                skip('unreadable')  # also happens while a take is being saved
                continue
            char = character_of(take.get('label'))
            if char is None:
                skip('other label')
                continue
            if take.get('input_source') != input_source:
                skip('not from the sensor board' if input_source == 'serial'
                     else 'not from {}'.format(input_source))
                continue
            if not take.get('sample_count'):
                skip('empty')
                continue
            if not os.path.exists(os.path.splitext(json_path)[0] + '.csv'):
                skip('no CSV')
                continue
            basename = take.get('basename') or filename
            if basename in seen_basenames:
                skip('duplicate')
                continue
            seen_basenames.add(basename)
            height_mm = take.get('height_mm')
            if height_mm is None:
                skip('without height, shown at 0 mm')
                height_mm = 0
            elif not isinstance(height_mm, (int, float)) or height_mm not in HEIGHTS_MM:
                skip('at a height outside {} mm'.format(
                    '/'.join(str(h) for h in HEIGHTS_MM)))
                continue
            samples.append({
                'participant_id': take.get('participant_id') or 'unassigned',
                'session_id': take.get('session_id'),
                'height_mm': height_mm,
                'char': char,
            })
    return samples, problems


def count_coverage(samples, chars, height_mm):
    """Return {participant_id: {char: number of takes}} at one height."""
    coverage = {}
    for sample in samples:
        if sample['char'] not in chars or float(sample['height_mm']) != float(height_mm):
            continue
        counts = coverage.setdefault(sample['participant_id'], {})
        counts[sample['char']] = counts.get(sample['char'], 0) + 1
    return coverage


def progress(samples, participant_ids, height_mm):
    """Takes that count toward the target: at most TARGET_REPS per character."""
    coverage = count_coverage(samples, DIGITS + LETTERS, height_mm)
    return sum(min(n, TARGET_REPS)
               for participant_id, counts in coverage.items()
               if participant_id in participant_ids
               for n in counts.values())


# ── Teleoperation experiments ──────────────────────────────────────────────

def _teleoperation_json(path):
    def reject_constant(value):
        raise ValueError('nonfinite JSON number')

    with open(path, encoding='utf-8') as stream:
        return json.load(stream, parse_constant=reject_constant)


def _teleoperation_text(value, default=''):
    if value is None:
        return default
    if not isinstance(value, str):
        raise ValueError('invalid text metadata')
    return value.strip()


def _teleoperation_source(value):
    source = _teleoperation_text(value, default='unknown')
    return 'trackpad' if source == 'touchpad' else source


def _teleoperation_target_valid(target):
    if not isinstance(target, dict):
        return False
    point, repetition = target.get('position_m'), target.get('repetition')
    return (isinstance(target.get('target_id'), str) and bool(target['target_id']) and
            isinstance(repetition, int) and not isinstance(repetition, bool) and repetition > 0 and
            isinstance(point, (list, tuple)) and len(point) == 3 and all(
                isinstance(value, (int, float)) and not isinstance(value, bool) and
                math.isfinite(value) for value in point))


def _teleoperation_artifact(folder, filename):
    """Only inspect the recorder's own files, never an absolute or escaped path."""
    if (not isinstance(filename, str) or not filename or
            os.path.basename(filename) != filename or filename.startswith('.')):
        raise ValueError('invalid artefact filename')
    path = os.path.realpath(os.path.join(folder, filename))
    if os.path.dirname(path) != os.path.realpath(folder):
        raise ValueError('artefact outside session')
    return path


@lru_cache(maxsize=512)
def _teleoperation_trace(path, mode, modified_ns, size):
    """Validate once per immutable file revision; repeated GUI refreshes stay cheap."""
    axes = ('actual_x_m', 'actual_y_m', 'actual_z_m') if mode == 'mujoco' else (
        'x_m', 'y_m', 'z_m')
    fields = ('wall_elapsed_s', 'sim_elapsed_s') + axes
    count, endpoint = 0, None
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not set(fields).issubset(reader.fieldnames):
            raise ValueError('invalid trajectory header')
        for sample in reader:
            if None in sample or any(value is None for value in sample.values()):
                raise ValueError('incomplete trajectory row')
            values = [float(sample[field]) for field in fields]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('nonfinite trajectory row')
            endpoint = tuple(values[-3:])
            count += 1
    return count, endpoint


def _teleoperation_trial(folder, row, manifest, mode, participant_id, session_id,
                         experiment_name, input_source):
    """A manifest row is committed only when its matching artefacts are readable."""
    if not isinstance(row, dict):
        raise ValueError('invalid trial record')
    number, samples = row.get('trial'), row.get('samples')
    if (isinstance(number, bool) or not isinstance(number, int) or number < 1 or
            isinstance(samples, bool) or not isinstance(samples, int) or samples < 0):
        raise ValueError('invalid trial count')
    statuses = ('completed', 'cancelled', 'timed_out', 'feedback_lost',
                'clock_reset', 'simulation_stopped')
    if row.get('status') not in statuses:
        raise ValueError('unfinished or invalid trial')
    if row.get('participant_id') != participant_id:
        raise ValueError('inconsistent participant')
    if mode == 'mujoco':
        if (row.get('session_id') != session_id or
                _teleoperation_text(row.get('experiment_name')) != experiment_name or
                _teleoperation_source(row.get('input_source')) != input_source):
            raise ValueError('inconsistent trial metadata')
        result = _teleoperation_json(_teleoperation_artifact(folder, row.get('result_json')))
        if not isinstance(result, dict) or any(
                key not in result or result[key] != value for key, value in row.items()):
            raise ValueError('inconsistent result')
        if result.get('protocol') != manifest['protocol']:
            raise ValueError('inconsistent result protocol')
        for key in ('end_effector_frame', 'end_effector_site', 'end_effector_reference'):
            # Protocol v1's original result writer stored the flange identity
            # only in the session manifest. Later v2 results repeat it per trial.
            if (key in manifest and
                    (manifest['protocol'].endswith('_v2') or key in result) and
                    result.get(key) != manifest[key]):
                raise ValueError('inconsistent result reference')
    elif row.get('condition') != manifest.get('condition'):
        raise ValueError('inconsistent condition')
    trace_path = _teleoperation_artifact(folder, row.get('trajectory_csv'))
    stat = os.stat(trace_path)
    count, endpoint = _teleoperation_trace(trace_path, mode, stat.st_mtime_ns, stat.st_size)
    if count != samples:
        raise ValueError('inconsistent trajectory count')
    if endpoint is not None:
        end_values = [row.get('end_{}_m'.format(axis)) for axis in 'xyz']
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
               not math.isfinite(value) or not math.isclose(value, actual, abs_tol=1e-9)
               for value, actual in zip(end_values, endpoint)):
            raise ValueError('inconsistent trajectory endpoint')
    if row['status'] == 'completed':
        duration = row.get('completion_time_s')
        if (not count or isinstance(duration, bool) or
                not isinstance(duration, (int, float)) or not math.isfinite(duration) or
                duration <= 0):
            raise ValueError('empty or invalid completed trial')
    return number, row['status']


def scan_teleoperation_runs(data_dir):
    """Return (sessions, problems) for committed local MuJoCo and Gazebo records.

    Successful samples require a completed manifest row and a matching, nonempty
    trajectory. MuJoCo additionally requires its matching result JSON. The older
    Gazebo writer keeps its result in the manifest itself. Pending directories,
    orphan files and plan-only runs never contribute. Bad files are reported and
    skipped without preventing other participants' progress from being shown.
    """
    runs, problems, seen_sessions = [], {}, set()

    def skip(reason):
        problems[reason] = problems.get(reason, 0) + 1

    for mode, root_name in (('mujoco', TELEOPERATION_DIR), ('gazebo', VIRTUAL_TASK_DIR)):
        root = os.path.join(data_dir, root_name)
        for folder, directories, filenames in os.walk(root):
            directories[:] = sorted(name for name in directories if not name.startswith('.'))
            if 'manifest.json' not in filenames:
                continue
            try:
                manifest = _teleoperation_json(os.path.join(folder, 'manifest.json'))
                if not isinstance(manifest, dict) or manifest.get('schema_version') != 1:
                    raise ValueError('invalid manifest')
                if manifest.get('source') == 'plan_only':
                    continue
                metadata, settings = manifest.get('metadata', {}), manifest.get('settings', {})
                targets, trials = manifest.get('targets'), manifest.get('trials')
                if (not isinstance(metadata, dict) or not isinstance(settings, dict) or
                        not isinstance(targets, list) or not all(_teleoperation_target_valid(t) for t in targets) or
                        not isinstance(trials, list)):
                    raise ValueError('invalid manifest shape')
                participant_id = _teleoperation_text(manifest.get('participant_id'))
                session_id = _teleoperation_text(manifest.get(
                    'session_id' if mode == 'mujoco' else 'run_id'))
                if not participant_id or not session_id:
                    raise ValueError('missing participant or session')
                if mode == 'mujoco':
                    protocol = manifest.get('protocol')
                    if protocol not in ('mujoco_random_target_reaching_v1',
                                        'mujoco_random_target_reaching_v2'):
                        raise ValueError('unknown MuJoCo protocol')
                    expected_source = ('mujoco_measured_flange' if protocol.endswith('_v1')
                                       else 'mujoco_measured_gripper_center')
                    if manifest.get('source') != expected_source:
                        raise ValueError('invalid measured source')
                    references = (dict(end_effector_frame='fr3_link8', end_effector_reference='flange')
                                  if protocol.endswith('_v1') else dict(
                                      end_effector_frame='gripper_center', end_effector_site='gripper_center',
                                      end_effector_reference='fingertip_midpoint'))
                    if any(manifest.get(key) != value for key, value in references.items()):
                        raise ValueError('invalid measured reference')
                    name_data = manifest
                else:
                    if manifest.get('source') != 'gazebo_measured_tf':
                        raise ValueError('invalid measured source')
                    protocol = manifest.get('protocol', 'gazebo_target_reaching_pilot_v1')
                    name_data = metadata
                experiment_name = _teleoperation_text(name_data.get('experiment_name'))
                participant_name = _teleoperation_text(name_data.get('participant_name'))
                input_source = _teleoperation_source(name_data.get('input_source'))
                if input_source not in ('trackpad', 'serial', 'unknown'):
                    raise ValueError('invalid input source')
                if mode == 'mujoco' and input_source == 'unknown':
                    raise ValueError('missing input source')
                condition = _teleoperation_text(manifest.get('condition'))
                identity = mode, participant_id, session_id
                if identity in seen_sessions:
                    skip('duplicate session')
                    continue
                config = dict(mode=mode, protocol=protocol, input_source=input_source,
                              condition=condition, settings=settings,
                              seed=manifest.get('seed'), targets=targets,
                              magnet_count=manifest.get('magnet_count', metadata.get('magnet_count')),
                              end_effector_reference=manifest.get('end_effector_reference'),
                              end_effector_frame=manifest.get('end_effector_frame'),
                              end_effector_site=manifest.get('end_effector_site'),
                              mapping={key: value for key, value in metadata.items() if
                                       key.startswith(('pointer_', 'board_', 'gripper_', 'flange_', 'wheel_')) or
                                       key in ('camera', 'position_control', 'height_controls',
                                               'height_slider_resolution_m', 'backend')})
                configuration_id = hashlib.sha256(json.dumps(
                    config, sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()[:16]
            except (OSError, ValueError, TypeError):
                skip('unreadable or invalid session')
                continue
            seen_sessions.add(identity)
            status_counts, trial_ids = {}, set()
            for row in trials:
                try:
                    number, status = _teleoperation_trial(
                        folder, row, manifest, mode, participant_id, session_id,
                        experiment_name, input_source)
                    if number in trial_ids:
                        skip('duplicate trial')
                        continue
                    trial_ids.add(number)
                    status_counts[status] = status_counts.get(status, 0) + 1
                except (OSError, ValueError, TypeError, csv.Error):
                    skip('incomplete or inconsistent trial')
            runs.append(dict(
                participant_id=participant_id, participant_name=participant_name,
                session_id=session_id, experiment_name=experiment_name, mode=mode,
                input_source=input_source, condition=condition, protocol=protocol,
                configuration_id=configuration_id, configuration=config,
                completed=status_counts.get('completed', 0), attempts=sum(status_counts.values()),
                status_counts=status_counts, planned=len(targets), target_count=len(targets),
                created_utc=manifest.get('created_utc', ''), folder=folder))
    return runs, problems


def count_teleoperation_coverage(runs, experiment_name=None, mode=None,
                                 input_source=None, condition=None):
    """Aggregate per participant across sessions matching the selected experiment.

    None accepts every value; '' selects exactly the unnamed experiment. Names
    and conditions are trimmed, with case preserved. Planned counts come from
    saved session manifests, rather than the launcher's next-session settings.
    """
    experiment_name = (None if experiment_name is None else experiment_name.strip())
    condition = None if condition is None else condition.strip()
    coverage, seen_sessions = {}, set()
    for run in runs:
        if any(expected is not None and run[key] != expected for key, expected in (
                ('experiment_name', experiment_name), ('mode', mode),
                ('input_source', input_source), ('condition', condition))):
            continue
        identity = run['mode'], run['participant_id'], run['session_id']
        if identity in seen_sessions:
            continue
        seen_sessions.add(identity)
        counts = coverage.setdefault(run['participant_id'], dict(
            completed=0, attempts=0, sessions=0, planned=0,
            status_counts={}, input_counts={}, protocols=[], configurations=[]))
        for key in ('completed', 'attempts', 'planned'):
            counts[key] += run[key]
        counts['sessions'] += 1
        for status, count in run['status_counts'].items():
            counts['status_counts'][status] = counts['status_counts'].get(status, 0) + count
        source = run['input_source']
        counts['input_counts'][source] = counts['input_counts'].get(source, 0) + run['completed']
        for key, value in (('protocols', run['protocol']), ('configurations', run['configuration_id'])):
            if value not in counts[key]:
                counts[key].append(value)
                counts[key].sort()
    return coverage


# ── Tracking error runs ─────────────────────────────────────────────────────

def tracking_output_dir(run_id):
    """Tracking recorder output folder for one run, relative to the repo root."""
    return os.path.join(DATA_DIR_NAME, TRACKING_DIR, run_id)


def list_tracking_runs(data_dir):
    """Return one summary dict per tracking-error run, newest first."""
    runs = []
    root = os.path.join(data_dir, TRACKING_DIR)
    if not os.path.isdir(root):
        return runs
    for name in sorted(os.listdir(root), reverse=True):
        manifest_path = os.path.join(root, name, 'manifest.json')
        try:
            with open(manifest_path, 'r', encoding='utf-8') as f:
                manifest = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        settings = manifest.get('settings') or {}
        # A redone target is saved twice, so count distinct targets.
        captured = {s.get('target_index') for s in manifest.get('sessions', [])
                    if s.get('sample_count')}
        runs.append({
            'run_id': manifest.get('run_id') or name,
            'magnet': settings.get('magnet'),
            'magnet_offset_mm': settings.get('magnet_offset_mm'),
            'heights_mm': settings.get('heights_mm') or [],
            'targets': manifest.get('target_count') or 0,
            'captured': len(captured),
        })
    return runs
