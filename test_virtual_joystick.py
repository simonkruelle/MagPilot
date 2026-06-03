#!/usr/bin/env python3
"""Smoke test the virtual joystick mode controller."""

from colmag.interaction import AppController, InputMode, VirtualJoystick


def make_controller():
    joystick = VirtualJoystick.default(extent=0.05)
    controller = AppController(
        joystick,
        dwell_seconds=1.0,
        letter_labels='ABCX',
        digit_labels='0123',
    )
    return joystick, controller


def button_named(joystick, name):
    return next(button for button in joystick.buttons if button.name == name)


def fire_button(controller, button, start_time):
    event = controller.update_cursor(button.x, button.y, start_time)
    assert event is None

    event = controller.update_cursor(button.x, button.y, start_time + 0.5)
    assert event is None

    event = controller.update_cursor(button.x, button.y, start_time + 1.1)
    assert event is not None

    repeat = controller.update_cursor(button.x, button.y, start_time + 1.2)
    assert repeat is None
    return event


def test_mode_buttons():
    joystick, controller = make_controller()

    event = fire_button(controller, button_named(joystick, 'Letters'), 0.0)
    assert controller.mode == InputMode.LETTERS
    assert event.command == 'letter_detection'
    assert event.classifier_labels == 'ABCX'

    controller.update_cursor(1.0, 1.0, 1.3)
    event = fire_button(controller, button_named(joystick, 'Digits'), 2.0)
    assert controller.mode == InputMode.DIGITS
    assert event.command == 'number_detection'
    assert event.classifier_labels == '0123'


def test_signs_button():
    joystick, controller = make_controller()

    event = fire_button(controller, button_named(joystick, 'Signs'), 0.0)
    assert controller.mode == InputMode.SIGNS
    assert event.command == 'symbol_detection'
    assert event.classifier_labels is None
    assert controller.classifier_labels is None


def test_reset_button_keeps_classification_mode():
    joystick, controller = make_controller()
    controller.mode = InputMode.LETTERS

    event = fire_button(controller, button_named(joystick, 'Reset'), 0.0)
    assert controller.mode == InputMode.LETTERS
    assert event.command == 'canvas:reset'
    assert event.classifier_labels is None
    assert controller.last_command == 'canvas:reset'


def test_choice_buttons_keep_classification_mode():
    expected_commands = {
        '1': 'choice:0',
        '2': 'choice:1',
        '3': 'choice:2',
        '4': 'choice:3',
    }

    for offset, (name, command) in enumerate(expected_commands.items()):
        joystick, controller = make_controller()
        controller.mode = InputMode.LETTERS
        event = fire_button(controller, button_named(joystick, name), float(offset) * 3.0)
        assert controller.mode == InputMode.LETTERS
        assert event.command == command
        assert event.classifier_labels is None
        assert controller.last_command == command


def test_moving_away_resets_dwell():
    joystick, controller = make_controller()
    button = button_named(joystick, 'Signs')

    assert controller.update_cursor(button.x, button.y, 0.0) is None
    assert controller.update_cursor(button.x, button.y, 0.6) is None
    assert controller.update_cursor(1.0, 1.0, 0.7) is None
    assert controller.update_cursor(button.x, button.y, 0.8) is None
    assert controller.update_cursor(button.x, button.y, 1.4) is None

    event = controller.update_cursor(button.x, button.y, 1.9)
    assert event is not None
    assert event.command == 'symbol_detection'


def main():
    test_mode_buttons()
    test_signs_button()
    test_reset_button_keeps_classification_mode()
    test_choice_buttons_keep_classification_mode()
    test_moving_away_resets_dwell()
    print("Virtual joystick smoke test passed")


if __name__ == "__main__":
    main()
