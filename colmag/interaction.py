"""Virtual joystick and app-mode logic for the COLMAG interface."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class InputMode(str, Enum):
    MENU = 'menu'
    LETTERS = 'letters'
    DIGITS = 'digits'
    ROBOT = 'robot'


@dataclass(frozen=True)
class VirtualButton:
    name: str
    x: float
    y: float
    radius: float
    action: str

    def contains(self, cursor_x, cursor_y):
        dx = float(cursor_x) - self.x
        dy = float(cursor_y) - self.y
        return dx * dx + dy * dy <= self.radius * self.radius


@dataclass(frozen=True)
class JoystickEvent:
    button: VirtualButton
    timestamp: float
    mode: InputMode
    classifier_labels: Optional[str]
    command: Optional[str]


class VirtualJoystick:
    """Two virtual joysticks in the projected XY workspace."""

    def __init__(self, buttons):
        self.buttons = tuple(buttons)

    @classmethod
    def default(cls, extent):
        radius = extent * 0.15
        left_x = -extent * 0.55
        right_x = extent * 0.55
        horizontal = extent * 0.30
        vertical = extent * 0.52

        return cls(
            [
                VirtualButton('U', left_x, vertical, radius, 'robot:up'),
                VirtualButton('D', left_x, -vertical, radius, 'robot:down'),
                VirtualButton('L', left_x - horizontal, 0.0, radius, 'mode:letters'),
                VirtualButton('R', left_x + horizontal, 0.0, radius, 'mode:digits'),
                VirtualButton('X', right_x, vertical, radius, 'robot:x'),
                VirtualButton('B', right_x, -vertical, radius, 'robot:b'),
                VirtualButton('A', right_x - horizontal, 0.0, radius, 'robot:a'),
                VirtualButton('C', right_x + horizontal, 0.0, radius, 'robot:c'),
            ]
        )

    def button_at(self, cursor_x, cursor_y):
        for button in self.buttons:
            if button.contains(cursor_x, cursor_y):
                return button
        return None


class DwellPressDetector:
    """Turns cursor dwell inside a virtual button into one press event."""

    def __init__(self, dwell_seconds=2.0):
        self.dwell_seconds = float(dwell_seconds)
        self.active_button = None
        self.entered_at = None
        self.fired_button = None

    def update(self, button, timestamp):
        timestamp = float(timestamp)
        if button is None:
            self.active_button = None
            self.entered_at = None
            self.fired_button = None
            return False, 0.0

        if self.active_button is None or self.active_button.name != button.name:
            self.active_button = button
            self.entered_at = timestamp
            self.fired_button = None
            return False, 0.0

        elapsed = max(0.0, timestamp - self.entered_at)
        progress = min(1.0, elapsed / self.dwell_seconds) if self.dwell_seconds > 0 else 1.0
        if progress >= 1.0 and self.fired_button != button.name:
            self.fired_button = button.name
            return True, progress
        return False, progress


class AppController:
    """State machine for virtual joystick modes and future robot commands."""

    def __init__(
        self,
        joystick,
        dwell_seconds=2.0,
        letter_labels='letters',
        digit_labels='digits',
    ):
        self.joystick = joystick
        self.detector = DwellPressDetector(dwell_seconds=dwell_seconds)
        self.letter_labels = letter_labels
        self.digit_labels = digit_labels
        self.mode = InputMode.MENU
        self.classifier_labels = None
        self.last_command = None
        self.last_event = None
        self.active_button = None
        self.dwell_progress = 0.0

    def update_cursor(self, cursor_x, cursor_y, timestamp):
        button = self.joystick.button_at(cursor_x, cursor_y)
        fired, progress = self.detector.update(button, timestamp)
        self.active_button = button.name if button else None
        self.dwell_progress = progress
        if not fired:
            return None

        event = self._handle_button(button, timestamp)
        self.last_event = event
        return event

    def _handle_button(self, button, timestamp):
        classifier_labels = None
        command = None

        if button.action == 'mode:letters':
            self.mode = InputMode.LETTERS
            classifier_labels = self.letter_labels
            self.classifier_labels = classifier_labels
            command = 'letter_detection'
        elif button.action == 'mode:digits':
            self.mode = InputMode.DIGITS
            classifier_labels = self.digit_labels
            self.classifier_labels = classifier_labels
            command = 'number_detection'
        elif button.action.startswith('robot:'):
            self.mode = InputMode.ROBOT
            command = button.action
            self.last_command = command

        return JoystickEvent(
            button=button,
            timestamp=float(timestamp),
            mode=self.mode,
            classifier_labels=classifier_labels,
            command=command,
        )
