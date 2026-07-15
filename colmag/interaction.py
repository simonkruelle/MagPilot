"""Virtual joystick and app-mode logic for the COLMAG interface."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class InputMode(str, Enum):
    MENU = 'menu'
    LETTERS = 'letters'
    DIGITS = 'digits'
    SIGNS = 'signs'
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
        left_x = -extent * 0.50
        right_x = extent * 0.50
        horizontal = extent * 0.18
        vertical = extent * 0.36

        return cls(
            [
                VirtualButton('MagPilot', left_x, vertical, radius, 'robot:teleop'),
                VirtualButton('Reset', left_x, -vertical, radius, 'canvas:reset'),
                VirtualButton('Letters', left_x - horizontal, 0.0, radius, 'mode:letters'),
                VirtualButton('Digits', left_x + horizontal, 0.0, radius, 'mode:digits'),
                VirtualButton('1', right_x, vertical, radius, 'choice:0'),
                VirtualButton('2', right_x + horizontal, 0.0, radius, 'choice:1'),
                VirtualButton('3', right_x, -vertical, radius, 'choice:2'),
                VirtualButton('4', right_x - horizontal, 0.0, radius, 'choice:3'),
            ]
        )

    @classmethod
    def teleop(cls, extent, include_layer=True):
        """Taskbar layout used while teleoperating: a row of buttons along the
        top strip so the rest of the canvas stays a clean teleop area. 'Draw'
        exits back to letter mode; the 'cmd:' buttons publish their command
        verbatim without changing the app mode."""
        radius = extent * 0.10
        bar_y = extent * 0.84
        buttons = [
            VirtualButton('Draw', -extent * 0.60, bar_y, radius, 'mode:letters'),
            VirtualButton('Gripper', 0.0, bar_y, radius, 'cmd:gripper:toggle'),
        ]
        if include_layer:
            buttons.append(VirtualButton(
                'Layer', extent * 0.60, bar_y, radius, 'cmd:plane:toggle'))
        return cls(buttons)

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
        teleop_joystick=None,
    ):
        self.joystick = joystick
        # Alternate button layout used while in ROBOT (teleop) mode; falls back
        # to the default layout when not provided.
        self.teleop_joystick = teleop_joystick or joystick
        self.detector = DwellPressDetector(dwell_seconds=dwell_seconds)
        self.letter_labels = letter_labels
        self.digit_labels = digit_labels
        self.mode = InputMode.MENU
        self.classifier_labels = None
        self.last_command = None
        self.last_event = None
        self.active_button = None
        self.dwell_progress = 0.0

    def active_joystick(self):
        """The button layout for the current mode (taskbar while teleoperating)."""
        return self.teleop_joystick if self.mode == InputMode.ROBOT else self.joystick

    def update_cursor(self, cursor_x, cursor_y, timestamp):
        button = self.active_joystick().button_at(cursor_x, cursor_y)
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
        elif button.action == 'mode:signs':
            self.mode = InputMode.SIGNS
            self.classifier_labels = None
            command = 'symbol_detection'
        elif button.action.startswith('robot:'):
            self.mode = InputMode.ROBOT
            command = button.action
            self.last_command = command
        elif button.action.startswith('cmd:'):
            # Passthrough button: publish the embedded command verbatim without
            # changing the app mode (used by the teleop taskbar).
            command = button.action[len('cmd:'):]
            self.last_command = command
        elif button.action.startswith('choice:'):
            command = button.action
            self.last_command = command
        elif button.action.startswith('canvas:'):
            command = button.action
            self.last_command = command

        return JoystickEvent(
            button=button,
            timestamp=float(timestamp),
            mode=self.mode,
            classifier_labels=classifier_labels,
            command=command,
        )
