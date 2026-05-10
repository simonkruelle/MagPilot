#!/usr/bin/env python3
"""Smoke test the virtual joystick mode controller."""

from colmag.interaction import AppController, InputMode, VirtualJoystick


def main():
    joystick = VirtualJoystick.default(extent=0.05)
    controller = AppController(
        joystick,
        dwell_seconds=1.0,
        letter_labels='ABCX',
        digit_labels='0123',
    )

    left_button = next(button for button in joystick.buttons if button.name == 'L')
    event = controller.update_cursor(left_button.x, left_button.y, 0.0)
    assert event is None
    event = controller.update_cursor(left_button.x, left_button.y, 1.1)
    assert event is not None
    assert controller.mode == InputMode.LETTERS
    assert event.classifier_labels == 'ABCX'

    right_button = next(button for button in joystick.buttons if button.name == 'R')
    controller.update_cursor(right_button.x, right_button.y, 2.0)
    event = controller.update_cursor(right_button.x, right_button.y, 3.1)
    assert event is not None
    assert controller.mode == InputMode.DIGITS
    assert event.classifier_labels == '0123'

    a_button = next(button for button in joystick.buttons if button.name == 'A')
    controller.update_cursor(a_button.x, a_button.y, 4.0)
    event = controller.update_cursor(a_button.x, a_button.y, 5.1)
    assert event is not None
    assert controller.mode == InputMode.ROBOT
    assert event.command == 'robot:a'

    print("Virtual joystick smoke test passed")


if __name__ == "__main__":
    main()
