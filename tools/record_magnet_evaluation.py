#!/usr/bin/env python3
"""Compare 1/2/3 magnets, three complete near-surface runs per stack."""

import argparse
import csv
import fcntl
import os
import sys
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from colmag import magnet_evaluation as evaluation
from tools.record_tracking_error import SerialSource, SimulatedSource, csv_header, collect_git_metadata


def offsets(text):
    if not text.strip():
        return [None]*3
    return [None if not value.strip() or value.strip() == '?' else float(value) for value in text.split(',')]


def prepare(directory, config, source, resume=False):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, 'manifest.json')
    if os.path.exists(path):
        if not resume:
            raise ValueError('This comparison already exists. Use Resume or a new output folder.')
        manifest = evaluation.read_manifest(directory)
        if manifest['settings'] != config or manifest['input_source'] != source:
            raise ValueError('Resume requires the original setup and input source.')
        return manifest
    if resume:
        raise ValueError('No comparison exists to resume.')
    manifest = dict(schema_version=evaluation.SCHEMA_VERSION, task=evaluation.TASK,
                    created_at=datetime.now().isoformat(), input_source=source,
                    settings=config, plan=evaluation.plan(), captures=[], git=collect_git_metadata())
    evaluation.write_json(path, manifest)
    evaluation.report(directory, manifest)
    return manifest


def capture(source, stage, config):
    if isinstance(source, SimulatedSource):
        source.target = dict(x_mm=stage.get('x_mm', 0), y_mm=stage.get('y_mm', 0),
                             z_mm=stage.get('magnet_centre_z_mm') or config['spacer_mm'],
                             tilt_deg=0, azimuth_deg=0)
    source.flush()
    list(source.read_rows(config['settle_s']))
    return list(source.read_rows(stage['duration_s']))


def run(directory, manifest, source, prompt=input, auto=False):
    config = manifest['settings']
    print('MAGNET BASELINE · {} · {}'.format(config['experiment_name'], manifest['input_source'].upper()))
    print('0 mm above cardboard; ~{:g} mm cardboard above sensors.'.format(config['spacer_mm']))
    print('Use identical magnets, arrangement, orientation and placement guide. Keep the stack upright.')
    print('Three full runs per count. Count order rotates between repeats. Enter captures; q saves and quits.')
    try:
        for index, item in enumerate(evaluation.plan(), 1):
            saved = evaluation.saved_stages(directory, manifest, item['run_id'])
            stages = evaluation.stages(config, item['magnet_count'])
            if {value['stage_id'] for value in stages}.issubset(saved):
                continue
            print('\nRUN {}/9 · {} magnet(s) · repeat {}/3'.format(index, item['magnet_count'], item['repetition']))
            if not auto and prompt('Install {} magnet(s), {}, then Enter (q quits) > '.format(
                    item['magnet_count'], config['arrangement'])).strip().lower() == 'q':
                return
            for stage in stages:
                if stage['stage_id'] in saved:
                    continue
                if stage['kind'] == 'baseline':
                    instruction = 'Move ALL magnets at least 30 cm away from the board'
                elif stage['kind'] == 'static':
                    instruction = 'Rest stack on cardboard at X={:+g}, Y={:+g} mm; keep it still'.format(stage['x_mm'], stage['y_mm'])
                else:
                    instruction = 'Trace the printed square, then its diagonals, slowly for {:g} s; keep the same surface gap'.format(stage['duration_s'])
                while True:
                    print(instruction)
                    if not auto and prompt('Enter records; q quits > ').strip().lower() == 'q':
                        return
                    print('Recording {:g} s…'.format(stage['duration_s']), flush=True)
                    rows = capture(source, stage, config)
                    if len(rows) >= 5:
                        break
                    print('Too few board packets. Check connection; this capture does not count.')
                    if auto:
                        raise ValueError('Too few packets received.')
                folder = os.path.join(directory, 'runs', item['run_id'])
                os.makedirs(folder, exist_ok=True)
                path = os.path.join(folder, stage['stage_id']+'.csv')
                with open(path+'.tmp', 'w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(csv_header())
                    writer.writerows(rows)
                os.replace(path+'.tmp', path)
                baseline_entry = next((entry for entry in manifest['captures']
                                 if entry['run_id'] == item['run_id'] and entry['stage_id'] == 'baseline'), None)
                baseline = evaluation.capture_metrics(directory, baseline_entry, stages[0], config) if baseline_entry else None
                entry = dict(item, **stage, csv=os.path.relpath(path, directory),
                             sample_count=len(rows), saved_at=datetime.now().isoformat(),
                             metrics=evaluation.metrics(rows, stage, config, baseline))
                manifest['captures'] = [old for old in manifest['captures'] if
                    (old['run_id'], old['stage_id']) != (item['run_id'], stage['stage_id'])] + [entry]
                evaluation.write_json(os.path.join(directory, 'manifest.json'), manifest)
                evaluation.report(directory, manifest)
                print('Saved {} packets. {}'.format(len(rows),
                    '' if stage['kind'] == 'baseline' else '{:.0f}% poses inside board range'.format(entry['metrics']['in_board_range_percent'])))
            print('Run complete. Progress: {}'.format(evaluation.progress(directory, manifest)))
    finally:
        source.close()
        evaluation.report(directory, manifest)
        print('\nReport: {}'.format(os.path.join(directory, 'report.md')))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--experiment-name', default='Magnet baseline')
    parser.add_argument('--magnet', default='small magnets; dimensions not measured')
    parser.add_argument('--arrangement', default='coaxial stack')
    parser.add_argument('--spacer-mm', type=float, default=5)
    parser.add_argument('--centre-offsets-mm', default='', help='1/2/3 magnet centre heights above cardboard in mm; blank or ? means unknown')
    parser.add_argument('--capture-s', type=float, default=2)
    parser.add_argument('--sweep-s', type=float, default=10)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--auto', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args(argv)
    try:
        config = evaluation.settings(args.experiment_name, args.magnet, args.arrangement,
                                     args.spacer_mm, offsets(args.centre_offsets_mm), args.capture_s, args.sweep_s)
        if args.auto and not args.simulate:
            raise ValueError('Automatic capture is only available with simulated input; board tests need physical placement.')
        if not args.dry_run and not args.simulate and not args.port:
            raise ValueError('Select the board serial port.')
        os.makedirs(args.output_dir, exist_ok=True)
        with open(os.path.join(args.output_dir, '.capture.lock'), 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ValueError('This comparison is already recording in another window.') from exc
            manifest = prepare(args.output_dir, config, 'simulated' if args.simulate else 'serial', args.resume)
            if args.dry_run:
                print('Plan: 1/2/3 magnets × 3 runs · 9 grid positions + above-sensor point + baseline + sweep')
                print('Physical cover: {} mm; nominal writing condition: 0 mm'.format(args.spacer_mm))
                print('Report: {}'.format(os.path.join(args.output_dir, 'report.md')))
                return 0
            if sum(evaluation.progress(args.output_dir, manifest).values()) == 9:
                print('Comparison already complete. Report: {}'.format(os.path.join(args.output_dir, 'report.md')))
                return 0
            source = SimulatedSource() if args.simulate else SerialSource(args.port, args.baudrate)
            run(args.output_dir, manifest, source, auto=args.auto)
        return 0
    except (ValueError, OSError) as exc:
        parser.exit(2, '{}\n'.format(exc))


if __name__ == '__main__':
    sys.exit(main())
