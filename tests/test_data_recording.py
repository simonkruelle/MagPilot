#!/usr/bin/env python3
"""Smoke test structured run/session data recording without sensor hardware."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import glob
import csv
import io
import json
import os
import shlex
import tempfile
from contextlib import contextmanager, redirect_stdout
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest.mock import patch

from magnetometer_reader import MagnetometerReader
from colmag.collection_protocol import CHARACTER_LABELS, TARGET_REPS


def make_row(timestamp, x, y, z):
    return [timestamp.isoformat(), *([0.0] * 48), x, y, z, 0.0, 0.0, 0.0]


def make_rows():
    start = datetime(2026, 5, 20, 12, 0, 0)
    return [
        make_row(start + timedelta(seconds=0.0), 0.000, 0.000, 0.02),
        make_row(start + timedelta(seconds=0.1), 0.005, 0.000, 0.03),
        make_row(start + timedelta(seconds=0.2), 0.010, 0.004, 0.04),
        make_row(start + timedelta(seconds=0.3), 0.014, 0.008, 0.05),
    ]


def load_manifest(output_dir):
    with open(os.path.join(output_dir, 'manifest.json'), 'r', encoding='utf-8') as f:
        return json.load(f)


def make_demo_reader(output_dir):
    return MagnetometerReader(
        enable_classifier=False, record_data=True, input_source='touchpad',
        touchpad_ink_mode='pen', writing_min_velocity=0.0, clean_view=True,
        output_dir=output_dir, run_id='callback_demo',
    )


@contextmanager
def collection_canvas(reader):
    """Drive the real canvas callbacks and its regular animation sampler."""
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.backend_bases import KeyEvent, MouseEvent

    reader.prepare_recording_layout(raw_csv_enabled=False)
    with patch.object(plt, 'show'):
        reader.plot_data()
    fig = reader._figure
    canvas = fig.canvas
    canvas.draw()  # initialize animation before checking its rendered image
    reader._animation.event_source.stop()
    ax = reader._teleop_ui['ax5']
    clock = [100.0]
    wall_start = datetime.now().timestamp() + 1.0

    def key(key_name, kind='key_press_event'):
        pixel_x, pixel_y = ax.transData.transform(
            (reader.touchpad_state['x'], reader.touchpad_state['y']))
        canvas.callbacks.process(kind, KeyEvent(
            kind, canvas, key=key_name, x=pixel_x, y=pixel_y))

    def tap(key_name):
        key(key_name)
        key(key_name, 'key_release_event')

    def pointer(x, y, kind='motion_notify_event', button=None):
        pixel_x, pixel_y = ax.transData.transform((x, y))
        event = MouseEvent(kind, canvas, pixel_x, pixel_y, button=button)
        assert event.inaxes is ax
        canvas.callbacks.process(kind, event)

    def frame():
        clock[0] += 0.02
        reader._animation._func(0)

    def pen_status():
        return next(artist for artist in ax.texts
                    if artist.get_text().startswith('Recording'))

    def outside():
        event = MouseEvent('axes_leave_event', canvas, -10, -10)
        canvas.callbacks.process('axes_leave_event', event)

    with patch('magnetometer_reader.time.monotonic', side_effect=lambda: clock[0]), \
            patch('magnetometer_reader.time.time',
                  side_effect=lambda: wall_start + clock[0] - 100.0):
        try:
            frame()
            yield SimpleNamespace(
                key=key, tap=tap, pointer=pointer, frame=frame, pen_status=pen_status,
                outside=outside,
                image=lambda: np.asarray(ax.images[0].get_array()).copy(),
            )
        finally:
            plt.close(fig)


def assert_saved_preview_matches(reader, output_dir, preview):
    import matplotlib.pyplot as plt
    import numpy as np

    entry = load_manifest(output_dir)['sessions'][-1]
    saved = plt.imread(os.path.join(output_dir, entry['paths']['png']))
    encoded = io.BytesIO()
    plt.imsave(encoded, preview, format='png', cmap='gray', vmin=0.0, vmax=1.0)
    encoded.seek(0)
    assert np.array_equal(saved, plt.imread(encoded))
    assert reader.last_capture_rows


def test_live_mode_does_not_create_layout():
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = os.path.join(tmpdir, 'live')
        MagnetometerReader(
            enable_classifier=False,
            output_dir=output_dir,
            run_id='live_test',
        )
        assert not os.path.exists(output_dir)


def test_dry_run_layout():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            dry_run=True,
            output_dir=tmpdir,
            run_id='dry_test',
        )

        assert not os.path.exists(os.path.join(tmpdir, 'manifest.json'))
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)

        assert os.path.isdir(os.path.join(tmpdir, 'raw'))
        assert os.path.isdir(os.path.join(tmpdir, 'samples'))
        manifest = load_manifest(tmpdir)
        assert manifest['run_id'] == 'dry_test'
        assert manifest['dry_run'] is True
        assert manifest['raw_log']['enabled'] is False
        assert manifest['sessions'] == []


def test_recording_artifacts_and_manifest():
    with tempfile.TemporaryDirectory() as tmpdir:
        raw_path = os.path.join(tmpdir, 'raw', 'unit_run_raw.csv')
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='unit_run',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=raw_path, raw_csv_enabled=True)

        reader.current_session = 'letter_A'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()

        label_dir = os.path.join(tmpdir, 'samples', 'letter_A')
        csv_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.csv'))
        png_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.png'))
        json_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.json'))
        assert len(csv_files) == len(png_files) == len(json_files) == 1

        manifest = load_manifest(tmpdir)
        assert manifest['raw_log']['enabled'] is True
        assert manifest['raw_log']['path'] == os.path.join('raw', 'unit_run_raw.csv')
        assert len(manifest['sessions']) == 1
        entry = manifest['sessions'][0]
        assert entry['label'] == 'letter_A'
        assert entry['repetition'] == 1
        assert entry['paths']['csv'].startswith(os.path.join('samples', 'letter_A'))
        assert entry['paths']['png'].endswith('.png')
        assert entry['paths']['json'].endswith('.json')

        reader.current_session = 'letter_A'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()
        assert glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep002_*.csv'))


def test_control_labels_are_separate():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='controls',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)

        reader.handle_keypress('-')
        assert reader.current_session is None
        reader.handle_keypress('enter')
        assert reader.current_session == 'control_blank'
        reader.session_data = make_rows()
        reader.handle_keypress('s')

        reader.handle_keypress('=')
        reader.handle_keypress('enter')
        assert reader.current_session == 'control_still'
        reader.session_data = make_rows()
        reader.handle_keypress('s')

        assert os.path.isdir(os.path.join(tmpdir, 'samples', 'control_blank'))
        assert os.path.isdir(os.path.join(tmpdir, 'samples', 'control_still'))
        manifest = load_manifest(tmpdir)
        labels = [entry['label'] for entry in manifest['sessions']]
        assert labels == ['control_blank', 'control_still']


def test_participant_and_session_ids():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='pilot',
            participant_id='P 01',
            session_id='S01',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)
        manifest = load_manifest(tmpdir)
        assert manifest['participant_id'] == 'P_01'
        assert manifest['session_id'] == 'S01'

        for _ in range(2):
            reader.current_session = 'digit_3'
            reader.current_session_started_at = make_rows()[0][0]
            reader.session_data = make_rows()
            reader.stop_session()

        label_dir = os.path.join(tmpdir, 'samples', 'digit_3')
        assert glob.glob(os.path.join(label_dir, 'pilot_P_01_S01_digit_3_rep001_*.csv'))
        assert glob.glob(os.path.join(label_dir, 'pilot_P_01_S01_digit_3_rep002_*.png'))
        sidecars = glob.glob(os.path.join(label_dir, 'pilot_P_01_S01_digit_3_rep001_*.json'))
        with open(sidecars[0], 'r', encoding='utf-8') as f:
            sidecar = json.load(f)
        assert sidecar['participant_id'] == 'P_01'
        assert sidecar['session_id'] == 'S01'

        manifest = load_manifest(tmpdir)
        assert [e['participant_id'] for e in manifest['sessions']] == ['P_01', 'P_01']
        assert [e['session_id'] for e in manifest['sessions']] == ['S01', 'S01']

        # A second participant in the same output dir starts its own repetition count.
        other = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='pilot',
            participant_id='P02',
            session_id='S01',
            writing_min_velocity=0.0,
        )
        other.current_session = 'digit_3'
        other.current_session_started_at = make_rows()[0][0]
        other.session_data = make_rows()
        other.stop_session()
        assert glob.glob(os.path.join(label_dir, 'pilot_P02_S01_digit_3_rep001_*.csv'))


def test_without_ids_keeps_legacy_names():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='legacy',
            writing_min_velocity=0.0,
        )
        assert reader.file_prefix() == 'legacy'
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)
        reader.current_session = 'letter_B'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()
        label_dir = os.path.join(tmpdir, 'samples', 'letter_B')
        assert glob.glob(os.path.join(label_dir, 'legacy_letter_B_rep001_*.csv'))
        entry = load_manifest(tmpdir)['sessions'][0]
        assert entry['participant_id'] is None
        assert entry['session_id'] is None


def test_height_is_stored_in_manifest_and_sidecar():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='heights',
            participant_id='P01',
            session_id='S02',
            height_mm=50.0,
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)
        reader.current_session = 'letter_C'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()

        manifest = load_manifest(tmpdir)
        assert manifest['height_mm'] == 50.0
        assert manifest['sessions'][0]['height_mm'] == 50.0
        label_dir = os.path.join(tmpdir, 'samples', 'letter_C')
        sidecar_path = glob.glob(os.path.join(label_dir, 'heights_P01_S02_letter_C_rep001_*.json'))[0]
        with open(sidecar_path, 'r', encoding='utf-8') as f:
            assert json.load(f)['height_mm'] == 50.0


def test_board_capture_ink_stays_dark_when_height_estimate_changes():
    import numpy as np

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, input_source='serial',
            writing_min_velocity=0, writing_max_z=0.05, clean_view=True,
            output_dir=tmpdir, run_id='stable_board_ink',
        )
        start = datetime(2026, 10, 5, 12, 0, 0)
        rows = [make_row(start + timedelta(seconds=i * 0.1),
                         -0.02 + i * 0.001, 0.0, 0.025)
                for i in range(31)]
        before = reader.rows_to_digit_image(rows[:25], trail_length=0)
        # A new height extreme must not renormalize previously drawn strokes.
        # Raw Z remains within the separate calibrated 50 mm pen-up gate.
        rows[-1][-4] = 0.055
        original = [row[:] for row in rows]
        after = reader.rows_to_digit_image(rows, trail_length=0)
        assert after.min() < 0.2  # sub-pixel brush centre lies between image rows
        assert np.all(after <= before)
        assert rows == original
        assert reader.manifest_settings()['stroke_strength_mode'] == 'uniform_capture'

        # Lifting beyond the height gate does not draw another segment.
        lifted = make_row(start + timedelta(seconds=3.1), 0.025, 0.015, 0.09)
        assert np.array_equal(
            reader.rows_to_digit_image(rows + [lifted], trail_length=0), after)
        reader.prepare_recording_layout(raw_csv_enabled=False)
        reader.start_session('digit_1')
        reader.session_data = rows
        assert reader.stop_session()
        entry = load_manifest(tmpdir)['sessions'][0]
        with open(os.path.join(tmpdir, entry['paths']['csv']), newline='') as f:
            saved_rows = list(csv.reader(f))[1:]
        assert [float(row[-4]) for row in saved_rows] == [row[-4] for row in original]
        assert_saved_preview_matches(reader, tmpdir, after)
        # Exercise the actual animation branch with serial-shaped rows. The
        # collection view must display the frozen full take at full opacity.
        import matplotlib.pyplot as plt
        reader.data_buffer.extend(rows)
        with patch.object(plt, 'show'):
            reader.plot_data()
        try:
            reader._figure.canvas.draw()
            reader._animation.event_source.stop()
            reader._animation._func(0)
            image_artist = reader._teleop_ui['ax5'].images[0]
            assert image_artist.get_alpha() == 1
            assert np.array_equal(np.asarray(image_artist.get_array()), after)
        finally:
            plt.close(reader._figure)


def test_board_capture_keeps_explicit_height_calibration_and_live_defaults():
    import numpy as np
    import sys
    import magnetometer_reader as reader_module

    calibrated = MagnetometerReader(enable_classifier=False, record_data=True,
                                   z_near=0.007, z_far=0.05)
    assert np.array_equal(calibrated.z_to_closeness([0.007, 0.05]), [1, 0])
    assert calibrated.manifest_settings()['stroke_strength_mode'] == 'calibrated_z'
    live = MagnetometerReader(enable_classifier=False)
    assert np.array_equal(live.z_to_closeness([0.01, 0.03]), [0, 1])
    assert live.manifest_settings()['stroke_strength_mode'] == 'relative_z'

    for record, explicit, expected in ((True, False, 0), (False, False, 0.035),
                                       (True, True, 0.002)):
        args = ['magnetometer_reader.py', '--no-classifier', '--input-source', 'serial']
        if record:
            args += ['--record-data']
        if explicit:
            args += ['--writing-min-velocity', '0.002']
        settings = []
        with patch.object(sys, 'argv', args), \
                patch.object(MagnetometerReader, 'run',
                             lambda self, **kwargs: settings.append(self.writing_min_velocity)):
            reader_module.main()
        assert settings == [expected]


def test_explicit_character_boundaries_and_protocol_metadata():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, output_dir=tmpdir,
            run_id='explicit', experiment_name='New pen — pilot',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        reader.handle_keypress('K')
        assert reader.selected_recording_label is None
        reader.handle_keypress('A')
        assert reader.current_session is None  # choosing the prompt records nothing
        reader.handle_keypress('enter')
        assert reader.current_session == 'letter_A'
        reader.session_data = make_rows()
        reader.handle_keypress('B')
        assert reader.selected_recording_label == 'letter_A'  # label cannot change mid-take
        reader.handle_keypress('enter')
        assert reader.current_session is None
        assert reader.session_data == []
        manifest = load_manifest(tmpdir)
        assert manifest['experiment_name'] == 'New pen — pilot'
        assert manifest['protocol']['character_labels'] == list(CHARACTER_LABELS)
        assert manifest['protocol']['target_reps'] == TARGET_REPS == 10
        entry = manifest['sessions'][0]
        assert entry['experiment_name'] == manifest['experiment_name']
        with open(os.path.join(tmpdir, entry['paths']['json'])) as f:
            metadata = json.load(f)
        assert metadata['experiment_name'] == manifest['experiment_name']
        assert metadata['capture_started_at']
        assert metadata['capture_stopped_at']
        assert metadata['sample_count'] == len(make_rows())
        reader.handle_keypress('enter')  # same character, next repetition
        assert reader.current_session == 'letter_A'
        reader.session_data = make_rows()
        reader.handle_keypress('escape')
        assert reader.current_session is None
        assert len(load_manifest(tmpdir)['sessions']) == 1


def test_save_uses_frozen_rows_while_producer_continues():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, output_dir=tmpdir,
            run_id='snapshot', writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        reader.start_session('digit_3')
        original_rows = make_rows()
        reader.session_data = original_rows
        expected_rows = [list(row) for row in original_rows]
        real_save_csv = reader.save_session_csv

        def producer_during_save(filename, rows=None):
            # This is the same conditional append the real acquisition thread
            # performs. It runs while artifact generation is in progress.
            with reader.data_lock:
                assert reader.current_session is None
                late_row = make_row(datetime.now(), 0.09, 0.09, 0.09)
                reader.data_buffer.append(late_row)
                if reader.current_session:
                    reader.session_data.append(late_row)
            original_rows.append(late_row)
            original_rows[0][-6] = 99.0  # mutable input cannot change frozen artifacts
            return real_save_csv(filename, rows=rows)

        with patch.object(reader, 'save_session_csv', side_effect=producer_during_save):
            assert reader.stop_session() is True
        entry = load_manifest(tmpdir)['sessions'][0]
        with open(os.path.join(tmpdir, entry['paths']['csv']), newline='') as f:
            csv_rows = list(csv.reader(f))[1:]
        with open(os.path.join(tmpdir, entry['paths']['json'])) as f:
            metadata = json.load(f)
        assert len(csv_rows) == entry['sample_count'] == metadata['sample_count'] == 4
        assert float(csv_rows[0][-6]) == expected_rows[0][-6]
        assert entry['ended_at'] == metadata['ended_at'] == expected_rows[-1][0]
        assert reader.last_capture_rows == tuple(tuple(row) for row in expected_rows)
        import matplotlib.pyplot as plt
        import numpy as np
        saved_image = plt.imread(os.path.join(tmpdir, entry['paths']['png']))
        expected_image = reader.rows_to_digit_image(expected_rows, trail_length=0)
        encoded_expected = io.BytesIO()
        plt.imsave(encoded_expected, expected_image, format='png', cmap='gray', vmin=0.0, vmax=1.0)
        encoded_expected.seek(0)
        assert np.array_equal(saved_image, plt.imread(encoded_expected))


def test_empty_and_failed_takes_do_not_count_and_failed_save_can_retry():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, output_dir=tmpdir,
            run_id='retry', writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        reader.start_session('letter_A')
        assert reader.stop_session() is False
        assert reader.reps_by_label() == {}
        assert load_manifest(tmpdir)['sessions'] == []
        reader.start_session('letter_A')
        reader.session_data = make_rows()
        with patch.object(reader, 'save_session_metadata', return_value=None):
            assert reader.stop_session() is False
        assert reader.reps_by_label() == {}
        assert reader.pending_capture is not None
        assert not glob.glob(os.path.join(tmpdir, 'samples', '*', '*.json'))
        assert reader.start_session('letter_B') is False
        assert reader.stop_session() is True
        assert reader.reps_by_label() == {'letter_A': 1}
        assert reader.pending_capture is None
        assert len(load_manifest(tmpdir)['sessions']) == 1
        assert len(glob.glob(os.path.join(tmpdir, 'samples', '*', '*.csv'))) == 1


def test_controls_are_outside_the_twenty_character_target():
    reader = MagnetometerReader(enable_classifier=False)
    reader.recording_sessions = [('control_blank', '', '', 5)]
    output = io.StringIO()
    with redirect_stdout(output):
        reader.print_recording_checklist()
    assert 'Done: 0/20 labels | 0/200 reps total' in output.getvalue()
    assert 'K[' not in output.getvalue()


def test_discarding_a_failed_save_removes_its_incomplete_manifest_entry():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, output_dir=tmpdir,
            run_id='discard', writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        reader.start_session('letter_A')
        reader.session_data = make_rows()
        with patch.object(reader, 'save_session_metadata', return_value=None):
            assert reader.stop_session() is False
        reader.handle_keypress('escape')
        assert reader.pending_capture is None
        assert reader.reps_by_label() == {}
        assert load_manifest(tmpdir)['sessions'] == []


def test_touchpad_capture_respects_boundaries_and_flushes_the_endpoint():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, input_source='touchpad',
            output_dir=tmpdir, run_id='touch', writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        clock = [100.0]
        wall_start = datetime(2026, 10, 4, 12, 0, 0).timestamp()
        reader.touchpad_state.update({
            'last_sample_at': 99.9, 'last_sample_x': -0.02, 'last_sample_y': 0.0,
            'x': 0.0, 'y': 0.0,
            'inside': True,
        })
        with patch('magnetometer_reader.time.monotonic', side_effect=lambda: clock[0]), \
                patch('magnetometer_reader.time.time', side_effect=lambda: wall_start + clock[0] - 100):
            reader.start_session('digit_3')
            # Use matching controlled wall time for the explicit capture boundary.
            reader.current_session_started_at = datetime.fromtimestamp(wall_start).isoformat()
            clock[0] = 100.02
            reader.touchpad_state['x'] = 0.01
            reader.append_touchpad_sample()
            clock[0] = 100.025  # before the next normal 60 Hz sample
            reader.touchpad_state['x'] = 0.015
            assert reader.stop_session() is True
        rows = reader.last_capture_rows
        assert rows
        assert all(datetime.fromisoformat(row[0]).timestamp() >= wall_start for row in rows)
        assert rows[-1][-6] == 0.015  # final movement was flushed into this take
        assert rows[0][-6] >= 0.0  # no interpolation back to pre-capture -0.02


def test_collection_canvas_enter_ignores_key_repeat():
    import matplotlib.pyplot as plt
    from matplotlib.backend_bases import KeyEvent
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False, record_data=True, output_dir=tmpdir,
            run_id='canvas', clean_view=True, writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_enabled=False)
        with patch.object(plt, 'show'):
            reader.plot_data()
        canvas = plt.gcf().canvas

        def key_event(kind, key):
            canvas.callbacks.process(kind, KeyEvent(kind, canvas, key=key))

        key_event('key_press_event', 'J')
        assert reader.current_session is None
        key_event('key_press_event', 'enter')
        assert reader.current_session == 'letter_J'
        key_event('key_press_event', 'enter')
        assert reader.current_session == 'letter_J'  # held key cannot stop it
        key_event('key_release_event', 'enter')
        reader.session_data = make_rows()
        key_event('key_press_event', 'enter')
        assert reader.current_session is None
        assert len(load_manifest(tmpdir)['sessions']) == 1
        plt.close('all')


def test_demo_enter_alone_controls_ink_and_isolates_takes():
    import numpy as np

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = make_demo_reader(tmpdir)
        with collection_canvas(reader) as ui:
            ui.tap('A')
            for x in np.linspace(-0.03, -0.02, 8):
                ui.pointer(x, -0.02)
                ui.frame()
            assert reader.current_session is None
            assert ui.image().min() == 1.0
            assert reader.data_buffer[-1][-3] == 0.0
            ui.tap('enter')
            for fraction in np.linspace(0.0, 1.0, 31):
                ui.pointer(-0.02 + 0.04 * fraction, -0.02 + 0.01 * fraction)
                ui.frame()
            assert 'move mouse to draw' in ui.pen_status().get_text()
            assert ui.image().min() < 0.5
            assert sum(row[-3] == 1.0 for row in reader.session_data) >= 30
            assert reader.session_data[-1][-3] == 1.0
            assert reader.touchpad_state['pen_down'] is False
            # Releasing either control cannot switch off experimenter-controlled ink.
            ui.pointer(0.02, -0.01, 'button_release_event', button=1)
            ui.key('space', 'key_release_event')
            ui.frame()
            assert reader.session_data[-1][-3] == 1.0
            ui.tap('enter')
            ui.frame()
            preview = ui.image()
            assert reader.reps_by_label() == {'letter_A': 1}
            assert_saved_preview_matches(reader, tmpdir, preview)
            frozen = reader.last_capture_rows
            for x in np.linspace(0.02, -0.03, 16):
                ui.pointer(x, 0.025)
                ui.frame()
            assert np.array_equal(ui.image(), preview)
            assert reader.last_capture_rows == frozen
            assert reader.data_buffer[-1][-3] == 0.0
            assert not ui.pen_status().get_visible()
            ui.tap('enter')
            for x in np.linspace(-0.03, -0.02, 11):
                ui.pointer(x, 0.025)
                ui.frame()
            ui.tap('enter')
            ui.frame()
            assert reader.reps_by_label() == {'letter_A': 2}
            assert all(row[-5] > 0.02 for row in reader.last_capture_rows)
            assert_saved_preview_matches(reader, tmpdir, ui.image())
            assert reader.manifest_settings()['touchpad']['recording_ink_gate'] == 'capture_interval'


def test_demo_outside_canvas_take_is_rejected_and_enter_can_retry():
    import numpy as np

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = make_demo_reader(tmpdir)
        with collection_canvas(reader) as ui:
            ui.tap('4')
            ui.outside()
            ui.tap('enter')
            for _ in range(8):
                ui.frame()
            assert reader.session_data
            assert all(row[-3] == 0.0 for row in reader.session_data)
            assert ui.image().min() == 1.0
            ui.tap('enter')
            ui.frame()
            assert reader.current_session is None
            assert reader.pending_capture is None
            assert reader.last_capture_rows == ()
            assert reader.selected_recording_label == 'digit_4'
            assert reader.reps_by_label() == {}
            assert load_manifest(tmpdir)['sessions'] == []
            assert not glob.glob(os.path.join(tmpdir, 'samples', '*', '*'))
            assert 'No canvas samples' in reader.action_feedback_text
            ui.tap('enter')
            for fraction in np.linspace(0.0, 1.0, 21):
                ui.pointer(0.025 - 0.05 * fraction, 0.02 - 0.04 * fraction)
                ui.frame()
            ui.tap('enter')
            assert reader.reps_by_label() == {'digit_4': 1}
            assert len(load_manifest(tmpdir)['sessions']) == 1


def test_demo_quick_exit_and_reentry_do_not_connect_strokes():
    import numpy as np

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = make_demo_reader(tmpdir)
        with collection_canvas(reader) as ui:
            ui.pointer(-0.005, 0.0)
            ui.frame()
            ui.tap('A')
            ui.tap('enter')
            for y in np.linspace(0.0, 0.005, 12):
                ui.pointer(-0.005, y)
                ui.frame()
            before = ui.image()
            # Leave and reenter between frames, with a gap small enough that
            # the renderer would otherwise connect the two endpoints.
            ui.outside()
            ui.pointer(0.005, 0.005)
            ui.frame()
            assert reader.session_data[-1][-3] == 0.0
            for y in np.linspace(0.005, 0.01, 12):
                ui.pointer(0.005, y)
                ui.frame()
            after = ui.image()
            scale = (reader.image_size - 1) / (2 * reader.projection_extent)
            middle_x = int(round(reader.projection_extent * scale))
            middle_y = int(round((reader.projection_extent - 0.005) * scale))
            assert abs(after[middle_y, middle_x] - before[middle_y, middle_x]) < 0.01
            assert after[:, middle_x + 3:].min() < 0.5
            ui.tap('enter')
            ui.frame()
            assert_saved_preview_matches(reader, tmpdir, ui.image())


def test_live_touchpad_pen_controls_still_use_mouse_and_space():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(enable_classifier=False, record_data=False,
            input_source='touchpad', touchpad_ink_mode='pen', writing_min_velocity=0,
            clean_view=True, output_dir=tmpdir)
        with collection_canvas(reader) as ui:
            ui.pointer(-0.01, 0.0)
            ui.frame()
            assert reader.data_buffer[-1][-3] == 0.0
            ui.pointer(-0.01, 0.0, 'button_press_event', button=1)
            ui.pointer(0.0, 0.0)
            ui.frame()
            assert reader.data_buffer[-1][-3] == 1.0
            ui.pointer(0.0, 0.0, 'button_release_event', button=1)
            ui.frame()
            assert reader.data_buffer[-1][-3] == 0.0
            ui.key('space')
            ui.pointer(0.01, 0.0)
            ui.frame()
            assert reader.data_buffer[-1][-3] == 1.0
            ui.key('space', 'key_release_event')
            ui.frame()
            assert reader.data_buffer[-1][-3] == 0.0
            assert reader.manifest_settings()['touchpad']['recording_ink_gate'] is None


def test_demo_optional_controls_allow_hover_only_takes():
    import matplotlib.pyplot as plt

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = make_demo_reader(tmpdir)
        with collection_canvas(reader) as ui:
            for key in ('-', '='):
                ui.tap(key)
                ui.tap('enter')
                ui.pointer(0.01, 0.02)
                ui.frame()
                ui.tap('enter')
            assert reader.reps_by_label() == {'control_blank': 1, 'control_still': 1}
            entries = load_manifest(tmpdir)['sessions']
            assert len(entries) == 2
            for entry in entries:
                image = plt.imread(os.path.join(tmpdir, entry['paths']['png']))
                assert image[:, :, :3].min() == 1.0


def test_demo_explicit_pen_stroke_remains_visible_near_canvas_edge():
    import numpy as np

    with tempfile.TemporaryDirectory() as tmpdir:
        reader = make_demo_reader(tmpdir)
        with collection_canvas(reader) as ui:
            x = reader.projection_extent * 0.96
            ui.tap('1')
            ui.pointer(x, -0.03)
            ui.tap('enter')
            for y in np.linspace(-0.03, 0.03, 31):
                ui.pointer(x, y)
                ui.frame()
            assert ui.image()[:, -6:].min() < 0.5
            ui.tap('enter')
            ui.frame()
            assert reader.reps_by_label() == {'digit_1': 1}
            assert ui.image()[:, -6:].min() < 0.5
            assert_saved_preview_matches(reader, tmpdir, ui.image())


def test_launcher_demo_command_records_a_separate_visible_character():
    import sys
    import magnetometer_reader as reader_module
    import matplotlib.pyplot as plt
    from colmag import dataset
    from colmag_launcher import build_record_command

    with tempfile.TemporaryDirectory() as data_dir:
        command = build_record_command('', 'P01', 'S01', 0,
                                       experiment_name='Lab meeting demo', demo=True)
        args = shlex.split(command.partition(' && ')[2])[2:]
        demo_root = dataset.collection_data_dir(data_dir, demo=True)
        output_dir = os.path.join(demo_root, 'characters', 'P01', 'S01')
        args[args.index('--output-dir') + 1] = output_dir

        def rehearse(reader, csv_filename=None, baudrate=921600):
            assert reader.input_source == 'touchpad'
            assert reader.touchpad_ink_mode == 'pen'
            assert reader.writing_min_velocity == 0
            assert reader.enable_classifier is False
            assert reader.ros_bridge is None
            import numpy as np
            with collection_canvas(reader) as ui:
                ui.tap('A')
                ui.pointer(-0.02, -0.02)
                ui.tap('enter')
                for start, end in (((-0.02, -0.02), (0.0, 0.02)),
                                   ((0.0, 0.02), (0.02, -0.02))):
                    for fraction in np.linspace(0.0, 1.0, 31):
                        ui.pointer(start[0] + fraction * (end[0] - start[0]),
                                   start[1] + fraction * (end[1] - start[1]))
                        ui.frame()
                assert ui.image().min() < 0.5
                ui.tap('enter')

        with patch.object(sys, 'argv', ['magnetometer_reader.py'] + args), \
                patch.object(MagnetometerReader, 'run', rehearse):
            reader_module.main()
        manifest = load_manifest(output_dir)
        assert manifest['input_source'] == 'touchpad'
        assert manifest['synthetic_data'] is True
        entry = manifest['sessions'][0]
        with open(os.path.join(output_dir, entry['paths']['json'])) as f:
            metadata = json.load(f)
        assert metadata['synthetic_data'] is True
        assert metadata['experiment_name'] == 'Lab meeting demo'
        image = plt.imread(os.path.join(output_dir, entry['paths']['png']))
        assert image[:, :, :3].min() < 0.5
        board_samples, _ = dataset.scan_samples(data_dir)
        demo_samples, _ = dataset.scan_samples(demo_root, input_source='touchpad')
        assert board_samples == []
        assert dataset.count_coverage(demo_samples, dataset.LETTERS, 0) == {'P01': {'A': 1}}


def main():
    test_live_mode_does_not_create_layout()
    test_dry_run_layout()
    test_recording_artifacts_and_manifest()
    test_control_labels_are_separate()
    test_participant_and_session_ids()
    test_without_ids_keeps_legacy_names()
    test_height_is_stored_in_manifest_and_sidecar()
    test_board_capture_ink_stays_dark_when_height_estimate_changes()
    test_board_capture_keeps_explicit_height_calibration_and_live_defaults()
    test_explicit_character_boundaries_and_protocol_metadata()
    test_save_uses_frozen_rows_while_producer_continues()
    test_empty_and_failed_takes_do_not_count_and_failed_save_can_retry()
    test_controls_are_outside_the_twenty_character_target()
    test_discarding_a_failed_save_removes_its_incomplete_manifest_entry()
    test_touchpad_capture_respects_boundaries_and_flushes_the_endpoint()
    test_collection_canvas_enter_ignores_key_repeat()
    test_demo_enter_alone_controls_ink_and_isolates_takes()
    test_demo_outside_canvas_take_is_rejected_and_enter_can_retry()
    test_demo_quick_exit_and_reentry_do_not_connect_strokes()
    test_live_touchpad_pen_controls_still_use_mouse_and_space()
    test_demo_optional_controls_allow_hover_only_takes()
    test_demo_explicit_pen_stroke_remains_visible_near_canvas_edge()
    test_launcher_demo_command_records_a_separate_visible_character()
    print("Structured data recording smoke test passed")


if __name__ == "__main__":
    main()
