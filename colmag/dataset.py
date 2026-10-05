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

import json
import os
import re
import tempfile
from datetime import date

from colmag.collection_protocol import DIGITS, LETTERS, TARGET_REPS


DATA_DIR_NAME = 'data_collection'
DEMO_DIR = 'demo'
CHARACTERS_DIR = 'characters'
TRACKING_DIR = 'tracking_error'
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
