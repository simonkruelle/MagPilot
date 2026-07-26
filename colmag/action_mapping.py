"""Shared, validated mapping from recognized characters to robot actions."""

import json
import os
import tempfile
from collections import OrderedDict


SCHEMA_VERSION = 1
DIGIT_LABELS = tuple('0123456789')
LETTER_LABELS = tuple('ABCDEFGHIJKLMNOPQRSTUVWXYZ')
ALL_LABELS = DIGIT_LABELS + LETTER_LABELS

# Ordered for the launcher dropdown and deliberately limited to motions already
# implemented and tested by colmag_robot_node.
ACTION_CATALOG = (
    ('none', 'Do nothing'),
    ('home', 'Home / reset'),
    ('wave', 'Wave'),
    ('bow', 'Bow'),
    ('nod_yes', 'Nod yes'),
    ('shake_no', 'Shake no'),
    ('fist_pumps', 'Fist pumps'),
    ('cheer', 'Cheer'),
    ('dab', 'Dab'),
    ('stretch_up', 'Stretch up'),
    ('point_left', 'Point left'),
    ('point_right', 'Point right'),
    ('cube_1', 'Cube corner 1'),
    ('cube_2', 'Cube corner 2'),
    ('cube_3', 'Cube corner 3'),
    ('cube_4', 'Cube corner 4'),
    ('cube_5', 'Cube corner 5'),
    ('cube_6', 'Cube corner 6'),
    ('cube_7', 'Cube corner 7'),
    ('cube_8', 'Cube corner 8'),
    ('cube_9', 'Cube center'),
)
ACTION_NAMES = OrderedDict(ACTION_CATALOG)
ACTION_IDS = tuple(ACTION_NAMES)
ACTION_METHODS = OrderedDict((
    ('home', '_home'),
    ('wave', '_wave'),
    ('bow', '_bow'),
    ('nod_yes', '_nod_yes'),
    ('shake_no', '_shake_no'),
    ('fist_pumps', '_celebrate'),
    ('cheer', '_cheer'),
    ('dab', '_dab'),
    ('stretch_up', '_stretch_up'),
    ('point_left', '_point_left'),
    ('point_right', '_point_right'),
))


class ActionMappingError(ValueError):
    """Raised when an action mapping cannot be used safely."""


def default_action_mapping():
    """Return a fresh mapping with the behavior used before customization."""
    mapping = OrderedDict()
    for label in DIGIT_LABELS:
        mapping[label] = 'home' if label == '0' else 'cube_{}'.format(label)
    for label in LETTER_LABELS:
        mapping[label] = 'nod_yes'
    mapping.update({
        'A': 'wave',
        'B': 'bow',
        'C': 'fist_pumps',
        'D': 'dab',
        'L': 'point_left',
        'R': 'point_right',
        'U': 'stretch_up',
        'X': 'home',
    })
    return mapping


def default_action_map_path():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(project_root, 'config', 'robot_actions.json')


DEFAULT_ACTION_MAP_PATH = default_action_map_path()


def bind_action_handlers(robot):
    """Bind every curated action id to its existing robot-node method."""
    handlers = OrderedDict((('none', None),))
    for action_id, method_name in ACTION_METHODS.items():
        handlers[action_id] = getattr(robot, method_name)
    for digit in range(1, 10):
        handlers['cube_{}'.format(digit)] = (
            lambda digit=digit: robot._go_to_digit(digit))
    if tuple(handlers) != ACTION_IDS:
        raise ActionMappingError(
            'action catalog and robot handler registry are out of sync')
    return handlers


def validate_action_mapping(mapping):
    """Validate and return a label-ordered mapping."""
    if not isinstance(mapping, dict):
        raise ActionMappingError('bindings must be a JSON object')

    missing = [label for label in ALL_LABELS if label not in mapping]
    extra = [label for label in mapping if label not in ALL_LABELS]
    if missing:
        raise ActionMappingError(
            'missing character bindings: {}'.format(', '.join(missing)))
    if extra:
        raise ActionMappingError(
            'unknown character bindings: {}'.format(', '.join(extra)))

    result = OrderedDict()
    for label in ALL_LABELS:
        action_id = mapping[label]
        if not isinstance(action_id, str) or action_id not in ACTION_NAMES:
            raise ActionMappingError(
                'unknown action {!r} for {}'.format(action_id, label))
        result[label] = action_id
    return result


def load_action_mapping(path=DEFAULT_ACTION_MAP_PATH):
    with open(path, 'r', encoding='utf-8') as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ActionMappingError('mapping file must contain a JSON object')
    if payload.get('schema_version') != SCHEMA_VERSION:
        raise ActionMappingError(
            'unsupported schema_version {!r}; expected {}'.format(
                payload.get('schema_version'), SCHEMA_VERSION))
    if 'bindings' not in payload:
        raise ActionMappingError('mapping file has no bindings object')
    return validate_action_mapping(payload['bindings'])


def save_action_mapping(path, mapping):
    """Validate and atomically replace an action mapping file."""
    validated = validate_action_mapping(mapping)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    payload = {
        'schema_version': SCHEMA_VERSION,
        'bindings': validated,
    }
    fd, temporary_path = tempfile.mkstemp(
        prefix='.robot_actions.', suffix='.tmp', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise
    return validated


def mapping_file_signature(path):
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return (stat.st_ino, stat.st_mtime_ns, stat.st_size)


class ReloadableActionMapping:
    """Keep the last valid mapping while watching an atomically replaced file."""

    _UNCHECKED = object()

    def __init__(self, path=DEFAULT_ACTION_MAP_PATH):
        self.path = path
        self.mapping = default_action_mapping()
        self.last_error = None
        self._signature = self._UNCHECKED

    def refresh(self):
        """Reload a changed file; return True only if active bindings changed."""
        signature = mapping_file_signature(self.path)
        if signature == self._signature:
            return False
        first_check = self._signature is self._UNCHECKED
        self._signature = signature

        if signature is None:
            self.last_error = None if first_check else (
                'mapping file is missing: {}'.format(self.path))
            return False

        try:
            mapping = load_action_mapping(self.path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.last_error = str(exc)
            return False

        changed = mapping != self.mapping
        self.mapping = mapping
        self.last_error = None
        return changed


def _compact_labels(labels):
    """Compact sorted one-character labels into ranges such as E-K, M-Q."""
    if not labels:
        return ''
    groups = []
    start = previous = labels[0]
    for label in labels[1:]:
        if ord(label) == ord(previous) + 1:
            previous = label
            continue
        groups.append((start, previous))
        start = previous = label
    groups.append((start, previous))
    return ', '.join(
        start if start == end else '{}-{}'.format(start, end)
        for start, end in groups
    )


def action_legend_rows(mapping, mode):
    """Return compact (labels, action name) rows for the active OCR mode."""
    validated = validate_action_mapping(mapping)
    labels = DIGIT_LABELS if mode == 'digits' else LETTER_LABELS
    remaining = list(labels)
    rows = []

    if mode == 'digits':
        cube_digits = [
            label for label in DIGIT_LABELS
            if label in '12345678'
            and validated[label] == 'cube_{}'.format(label)
        ]
        if cube_digits:
            rows.append((_compact_labels(cube_digits), 'Cube corners'))
            remaining = [label for label in remaining if label not in cube_digits]

    grouped = OrderedDict()
    for label in remaining:
        grouped.setdefault(validated[label], []).append(label)
    grouped_rows = [
        (_compact_labels(group_labels), ACTION_NAMES[action_id])
        for action_id, group_labels in grouped.items()
    ]
    rows.extend(grouped_rows)

    order = {label: index for index, label in enumerate(labels)}
    rows.sort(key=lambda row: min(order[label] for part in row[0].split(', ')
                                  for label in (
                                      (part,) if '-' not in part
                                      else (part.split('-', 1)[0],))))
    return tuple(rows)
