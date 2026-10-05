"""Shared labels and repetition target for character data collection.

Control recordings remain available for checking the board, but are separate
from the twenty characters collected for the recognition dataset.
"""

DIGITS = tuple('0123456789')
LETTERS = tuple('ABCDEFGHIJ')
CHARACTER_LABELS = tuple('digit_{}'.format(char) for char in DIGITS) + tuple(
    'letter_{}'.format(char) for char in LETTERS)
CONTROL_LABELS = ('control_blank', 'control_still')
TARGET_REPS = 10
PROTOCOL_ID = 'digits_0_9_letters_A_J_v1'


def protocol_metadata():
    """Return an independent JSON-serializable description of the protocol."""
    return {
        'protocol_id': PROTOCOL_ID,
        'character_labels': list(CHARACTER_LABELS),
        'control_labels': list(CONTROL_LABELS),
        'target_reps': TARGET_REPS,
    }
