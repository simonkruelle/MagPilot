"""Physical protocol geometry, durable progress and truthful diagnostics."""

import contextlib
import csv
import fcntl
import io
import json
import math
import os
import tempfile
import unittest

from colmag import magnet_evaluation as evaluation
from tools import record_magnet_evaluation as recorder
from tools.record_tracking_error import SimulatedSource, csv_header


class MagnetEvaluationTests(unittest.TestCase):
    def config(self):
        return evaluation.settings('Baseline', 'Approx. Ø9.5–10 × 5 mm disks',
                                   'coaxial stack, upright', 5, [2.5, 5, 7.5], .1, .1)

    def save(self, directory, manifest, stage, error_mm=0):
        item = evaluation.plan()[0]
        folder = os.path.join(directory, 'runs', item['run_id'])
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, stage['stage_id']+'.csv')
        rows = [['2026-10-05T12:00:00.000000', *([1.0]*48),
                 (stage.get('x_mm', 0)+error_mm)/1000, stage.get('y_mm', 0)/1000,
                 .0175, 0, 0, 1] for _ in range(5)]
        with open(path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(csv_header())
            writer.writerows(rows)
        entry = dict(item, **stage, csv=os.path.relpath(path, directory), sample_count=5,
                     metrics=evaluation.metrics(rows, stage, manifest['settings']))
        manifest['captures'].append(entry)
        evaluation.write_json(os.path.join(directory, 'manifest.json'), manifest)
        return entry, rows

    def test_nine_runs_rotate_order_and_keep_cover_separate(self):
        plan = evaluation.plan()
        self.assertEqual([item['magnet_count'] for item in plan], [1, 2, 3, 2, 3, 1, 3, 1, 2])
        self.assertEqual(len({item['run_id'] for item in plan}), 9)
        for count in (1, 2, 3):
            self.assertEqual([item['repetition'] for item in plan if item['magnet_count'] == count], [1, 2, 3])
            stages = evaluation.stages(self.config(), count)
            self.assertEqual(len(stages), 7)
            stationary = [stage for stage in stages if stage['kind'] == 'static']
            self.assertEqual(len(stationary), 5)
            self.assertTrue(all(stage['nominal_height_mm'] == 0 for stage in stationary))
            self.assertTrue(all(stage['magnet_centre_z_mm'] == 5+2.5*count for stage in stationary))
            self.assertEqual([stage['position_name'] for stage in stationary],
                             ['Top-left', 'Top-right', 'Bottom-left', 'Bottom-right', 'Centre'])
            self.assertEqual((stationary[-1]['x_mm'], stationary[-1]['y_mm']), (0, 0))

    def test_unknown_centre_omits_z_error_and_bad_numbers_are_rejected(self):
        config = evaluation.settings('x')
        stage = evaluation.stages(config, 1)[1]
        rows = [['2026-10-05T12:00:00', *([0.0]*48), -.035, .035, .018, 0, 0, 1]]*5
        self.assertIsNone(evaluation.metrics(rows, stage, config)['z_rmse_mm'])
        for kwargs in (dict(spacer_mm=float('nan')), dict(centre_offsets_mm=[1, 2]),
                       dict(centre_offsets_mm=[1, float('inf'), 3]), dict(capture_s=0)):
            with self.assertRaises(ValueError):
                evaluation.settings('x', **kwargs)

    def test_invalid_poses_remain_in_error_and_raw_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'serial')
            stage = evaluation.stages(self.config(), 1)[1]
            entry, rows = self.save(folder, manifest, stage, error_mm=200)
            values = entry['metrics']
            self.assertAlmostEqual(values['xy_rmse_mm'], 200)
            self.assertEqual(values['in_teleop_range_percent'], 0)
            self.assertEqual(evaluation.capture_rows(folder, entry), rows)
            rows[0][49] = float('nan')
            values = evaluation.metrics(rows, stage, self.config())
            self.assertEqual(values['finite_pose_percent'], 80)
            self.assertAlmostEqual(values['xy_rmse_mm'], 200)

    def test_aggregate_error_is_rms_and_signal_diagnostics_survive_report(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'serial')
            stages = evaluation.stages(self.config(), 1)
            self.save(folder, manifest, stages[0])
            self.save(folder, manifest, stages[1], 3)
            self.save(folder, manifest, stages[2], 4)
            summary = evaluation.report(folder, manifest)[0]
            self.assertAlmostEqual(summary['xy_rmse_mm'], math.sqrt(12.5))
            self.assertEqual(summary['field_signal_rms_raw'], 0)
            self.assertEqual(summary['baseline_noise_rms_raw'], 0)
            self.assertEqual(summary['captured'], 3)
            self.assertFalse(summary['completed'])

    def test_raw_signal_uses_the_magnet_away_baseline(self):
        config = self.config()
        stage = evaluation.stages(config, 1)[1]
        rows = [['2026-10-05T12:00:00', *([3.0]*48), -.035, .035, .018, 0, 0, 1]]*5
        result = evaluation.metrics(rows, stage, config, {'field_mean_raw': [1.0]*48})
        self.assertEqual(result['field_signal_rms_raw'], 2)
        self.assertEqual(result['field_noise_rms_raw'], 0)
        self.assertEqual(result['field_peak_raw'], 3)

    def test_complete_simulation_and_progress_do_not_mix_with_board(self):
        with tempfile.TemporaryDirectory() as data:
            folder = os.path.join(data, 'magnet_evaluation', 'demo')
            manifest = recorder.prepare(folder, self.config(), 'simulated')
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.run(folder, manifest, SimulatedSource(rate_hz=50), auto=True)
            self.assertEqual(evaluation.progress(folder), {1: 3, 2: 3, 3: 3})
            self.assertEqual(len(manifest['captures']), 63)
            self.assertEqual(evaluation.list_comparisons(data, 'Other'), [])
            self.assertEqual(evaluation.list_comparisons(data, 'Baseline')[0]['input_source'], 'simulated')
            for name in ('report.md', 'comparison.svg', 'summary.json'):
                self.assertTrue(os.path.isfile(os.path.join(folder, name)))
            with open(os.path.join(folder, 'report.md')) as stream:
                self.assertIn('SIMULATED', stream.read())

    def test_quit_and_resume_preserve_completed_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'simulated')
            answers = iter(['', '', 'q'])  # install stack; baseline; stop before first grid point
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.run(folder, manifest, SimulatedSource(rate_hz=50), prompt=lambda _: next(answers))
            self.assertEqual(len(manifest['captures']), 1)
            previous = manifest['captures'][0].copy()
            self.assertEqual(evaluation.progress(folder), {1: 0, 2: 0, 3: 0})
            resumed = recorder.prepare(folder, self.config(), 'simulated', resume=True)
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.run(folder, resumed, SimulatedSource(rate_hz=50), auto=True)
            self.assertEqual(len(resumed['captures']), 63)
            self.assertEqual(resumed['captures'][0], previous)
            with self.assertRaises(ValueError):
                recorder.prepare(folder, self.config(), 'serial', resume=True)
            changed = self.config()
            changed['spacer_mm'] = 10
            with self.assertRaises(ValueError):
                recorder.prepare(folder, changed, 'simulated', resume=True)

    def test_partial_corrupt_and_external_csvs_do_not_count(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'simulated')
            for stage in evaluation.stages(self.config(), 1):
                entry, _ = self.save(folder, manifest, stage)
            self.assertEqual(evaluation.progress(folder)[1], 1)
            path = os.path.join(folder, entry['csv'])
            with open(path, 'a') as stream:
                stream.write('partial,row\n')
            self.assertEqual(evaluation.progress(folder)[1], 0)
            with open(path, 'w') as stream:
                stream.write('wrong,header\n')
            self.assertEqual(evaluation.progress(folder)[1], 0)
            self.assertFalse(evaluation.capture_valid(folder, dict(entry, csv='../outside.csv')))

    def test_damaged_cached_metrics_are_recomputed_from_the_original_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'simulated')
            stages = evaluation.stages(self.config(), 1)
            baseline, rows = self.save(folder, manifest, stages[0])
            self.save(folder, manifest, stages[1], 3)
            baseline['metrics'] = {'sample_count': 5}
            manifest['captures'][1]['metrics'] = {'field_peak_raw': float('nan')}
            summary = evaluation.report(folder, manifest)[0]
            self.assertEqual(summary['baseline_noise_rms_raw'], 0)
            self.assertAlmostEqual(summary['xy_rmse_mm'], 3)

    def test_damaged_manifest_is_skipped_without_resetting_data(self):
        with tempfile.TemporaryDirectory() as data:
            folder = os.path.join(data, 'magnet_evaluation', 'bad')
            manifest = recorder.prepare(folder, self.config(), 'serial')
            for payload in ([], dict(manifest, captures=[1]), dict(manifest, settings={})):
                evaluation.write_json(os.path.join(folder, 'manifest.json'), payload)
                self.assertEqual(evaluation.list_comparisons(data), [])
                with self.assertRaises(ValueError):
                    recorder.prepare(folder, self.config(), 'serial', resume=True)

    def test_legacy_grid_remains_resumable_with_its_original_ten_positions(self):
        config = evaluation.settings('Legacy', 'disk', 'upright', 5, [2.5, 5, 7.5],
                                     .1, .1, evaluation.LEGACY_GRID)
        self.assertNotIn('position_protocol', config)
        stages = evaluation.stages(config, 1)
        self.assertEqual(len(stages), 12)
        self.assertEqual(stages[1]['stage_id'], 'x-35_y+35')
        self.assertEqual(stages[-2]['stage_id'], 'above_sensor')
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, config, 'simulated')
            answers = iter(['', '', 'q'])
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.run(folder, manifest, SimulatedSource(rate_hz=50), prompt=lambda _: next(answers))
            self.assertEqual(evaluation.read_manifest(folder)['settings'], config)
            resumed = recorder.prepare(folder, config, 'simulated', resume=True)
            with contextlib.redirect_stdout(io.StringIO()):
                recorder.run(folder, resumed, SimulatedSource(rate_hz=50), auto=True)
            self.assertEqual(len(resumed['captures']), 108)
            self.assertEqual(evaluation.progress(folder), {1: 3, 2: 3, 3: 3})
            with open(os.path.join(folder, 'report.md')) as stream:
                self.assertIn('grid_v1', stream.read())
            with self.assertRaises(ValueError):
                recorder.prepare(folder, evaluation.settings('Legacy', 'disk', 'upright', 5,
                                 [2.5, 5, 7.5], .1, .1), 'simulated', resume=True)

    def test_named_prompts_match_the_printable_guide_and_hide_coordinates(self):
        import xml.etree.ElementTree as ET
        root = ET.parse(os.path.join(os.path.dirname(__file__), '..', 'docs', 'magnet_evaluation_grid.svg')).getroot()
        namespace = {'svg': 'http://www.w3.org/2000/svg'}
        targets = [element for element in root.findall('.//svg:use', namespace)
                   if element.get('href') == '#target']
        expected = [(90+x, 90-y) for _, _, x, y in evaluation.POSITIONS]
        self.assertEqual([(float(target.get('x')), float(target.get('y'))) for target in targets], expected)
        with tempfile.TemporaryDirectory() as folder:
            manifest = recorder.prepare(folder, self.config(), 'simulated')
            prompts = io.StringIO()
            with contextlib.redirect_stdout(prompts):
                recorder.run(folder, manifest, SimulatedSource(rate_hz=50), auto=True)
            self.assertNotIn('X=', prompts.getvalue())
            self.assertNotIn('Y=', prompts.getvalue())
            for name in ('Top-left', 'Top-right', 'Bottom-left', 'Bottom-right', 'Centre'):
                self.assertIn(name, prompts.getvalue())

    def test_unknown_placement_protocol_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluation.settings('x', position_protocol='unknown')

    def test_automatic_real_capture_is_blocked_and_double_resume_is_locked(self):
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                recorder.main(['--auto', '--port', 'fake', '--output-dir', folder])
            self.assertEqual(error.exception.code, 2)
            self.assertFalse(os.path.exists(os.path.join(folder, 'manifest.json')))
            with open(os.path.join(folder, '.capture.lock'), 'a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaises(SystemExit) as error:
                    recorder.main(['--dry-run', '--output-dir', folder])
                self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
