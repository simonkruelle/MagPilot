"""Protocol and reports for the near-surface, three-stack board comparison.

Only the standard library is used, so the launcher can inspect progress without
opening the serial port. A run means five named placements and a motion capture,
rather than three consecutive samples at one position. Existing grid runs keep
their original placement protocol.
"""

import csv
import json
import math
import os
import statistics
from datetime import datetime
from functools import lru_cache
from html import escape

TASK = 'magnet_stack_baseline'
SCHEMA_VERSION = 1
COUNTS = (1, 2, 3)
REPEATS = 3
FIVE_POSITIONS = 'five_positions_v2'
LEGACY_GRID = 'grid_v1'
POSITIONS = (
    ('top_left', 'Top-left', -35.0, 35.0),
    ('top_right', 'Top-right', 35.0, 35.0),
    ('bottom_left', 'Bottom-left', -35.0, -35.0),
    ('bottom_right', 'Bottom-right', 35.0, -35.0),
    ('centre', 'Centre', 0.0, 0.0),
)


def settings(experiment_name, magnet='small magnets', arrangement='coaxial stack',
             spacer_mm=5, centre_offsets_mm=None, capture_s=2, sweep_s=10,
             position_protocol=FIVE_POSITIONS):
    """Keep nominal surface condition separate from physical magnet-centre Z."""
    spacer_mm, capture_s, sweep_s = map(float, (spacer_mm, capture_s, sweep_s))
    if not all(math.isfinite(v) for v in (spacer_mm, capture_s, sweep_s)):
        raise ValueError('Thickness and recording times must be finite numbers.')
    if spacer_mm < 0 or capture_s <= 0 or sweep_s <= 0:
        raise ValueError('Thickness cannot be negative; recording times must be positive.')
    offsets = list(centre_offsets_mm or (None, None, None))
    if len(offsets) != 3:
        raise ValueError('Enter three centre offsets, one for each magnet count.')
    for index, value in enumerate(offsets):
        if value is not None:
            value = float(value)
            if not math.isfinite(value) or value < 0:
                raise ValueError('Centre offsets must be finite, nonnegative mm.')
            offsets[index] = value
    if not str(magnet).strip() or not str(arrangement).strip():
        raise ValueError('Describe the magnets and their arrangement.')
    if position_protocol not in (FIVE_POSITIONS, LEGACY_GRID):
        raise ValueError('Unknown placement protocol.')
    config = dict(experiment_name=str(experiment_name).strip(), magnet=str(magnet).strip(),
                arrangement=str(arrangement).strip(), nominal_height_mm=0.0,
                spacer_mm=spacer_mm, centre_offsets_mm=offsets,
                magnet_counts=list(COUNTS), repeats=REPEATS,
                grid_xy_mm=[-35.0, 0.0, 35.0], tilt_deg=0.0,
                settle_s=0.5, capture_s=capture_s, sweep_s=sweep_s,
                baseline_s=2.0, sensor_bias_m=0.01,
                calibration_note='Existing playground bias: abs(raw Z) minus 10 mm; not recalibrated by this comparison.',
                centre_offset_note='Operator-supplied approximate geometric offsets; verify stack geometry before treating Z as ground truth.',
                frame='board centre; X/Y in mm, physical Z above sensor plane',
                height_note='0 mm above cardboard; spacer thickness is approximate; '
                            'magnet-centre offsets are above cardboard and may be approximate')
    if position_protocol == FIVE_POSITIONS:
        config['position_protocol'] = FIVE_POSITIONS
    return config


def position_protocol(config):
    # Published v1 settings have no protocol field. Never shorten their plan.
    return config.get('position_protocol', LEGACY_GRID)


def plan():
    """Rotate count order between repeats to reduce a simple order effect."""
    return [dict(run_id='mag{}_rep{}'.format(count, repetition),
                 magnet_count=count, repetition=repetition)
            for repetition, order in enumerate(((1, 2, 3), (2, 3, 1), (3, 1, 2)), 1)
            for count in order]


def stages(config, count):
    result = [dict(stage_id='baseline', kind='baseline', duration_s=config['baseline_s'])]
    axis = config['grid_xy_mm']
    offset = config['centre_offsets_mm'][count - 1]
    if position_protocol(config) == FIVE_POSITIONS:
        placements = POSITIONS
    else:
        placements = []
        names = {(x, y): '{}{}'.format('Top' if y > 0 else 'Bottom' if y < 0 else 'Middle',
                                     '-left' if x < 0 else '-right' if x > 0 else '-centre')
                 for y in axis for x in axis}
        names[(0.0, 0.0)] = 'Centre'
        for row, y in enumerate(reversed(axis)):
            for x in (axis if row % 2 == 0 else list(reversed(axis))):
                placements.append(('x{:+g}_y{:+g}'.format(x, y), names[(x, y)], x, y))
        placements.append(('above_sensor', 'Extra mark above a sensor', 17.5, 17.5))
    for number, (stage_id, name, x, y) in enumerate(placements, 1):
        result.append(dict(stage_id=stage_id, kind='static', position_name=name, position_number=number,
                           x_mm=x, y_mm=y, nominal_height_mm=0.0,
                           magnet_centre_z_mm=None if offset is None else config['spacer_mm'] + offset,
                           duration_s=config['capture_s']))
    result.append(dict(stage_id='sweep', kind='sweep', duration_s=config['sweep_s']))
    return result


def read_manifest(directory):
    with open(os.path.join(directory, 'manifest.json'), encoding='utf-8') as stream:
        value = json.load(stream)
    if not isinstance(value, dict) or value.get('task') != TASK or value.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('This folder is not a magnet-stack comparison.')
    config = value.get('settings')
    if not isinstance(config, dict) or not isinstance(value.get('captures'), list):
        raise ValueError('Damaged comparison settings or capture list.')
    if any(not isinstance(entry, dict) or not isinstance(entry.get('run_id'), str) or
           not isinstance(entry.get('stage_id'), str) or not isinstance(entry.get('csv'), str)
           for entry in value['captures']):
        raise ValueError('Damaged capture metadata; restore it before resuming.')
    try:
        expected = settings(config['experiment_name'], config['magnet'], config['arrangement'],
                            config['spacer_mm'], config['centre_offsets_mm'],
                            config['capture_s'], config['sweep_s'], position_protocol(config))
    except (KeyError, ValueError, TypeError) as exc:
        raise ValueError('Damaged comparison setup.') from exc
    if config != expected or value.get('input_source') not in ('serial', 'simulated'):
        raise ValueError('Unknown comparison protocol or input source.')
    return value


def write_json(path, value):
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
    os.replace(temporary, path)


def capture_rows(directory, entry):
    """Require a saved CSV with enough rows; reject missing/outside paths."""
    if not isinstance(entry, dict) or not isinstance(entry.get('csv'), str):
        return []
    path = os.path.realpath(os.path.join(directory, entry['csv']))
    root = os.path.realpath(directory) + os.sep
    if not path.startswith(root) or not path.endswith('.csv'):
        return []
    try:
        with open(path, newline='') as stream:
            reader = csv.reader(stream)
            header = next(reader)
            expected = ['timestamp'] + ['Sensor{}_B{}'.format(sensor, axis)
                         for sensor in range(1, 17) for axis in ('x', 'y', 'z')] + [
                         'Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz']
            if header != expected:
                return []
            rows = []
            for row in reader:
                if len(row) != 55:
                    return []
                rows.append([row[0], *map(float, row[1:])])
        return rows if len(rows) >= 5 and len(rows) == entry.get('sample_count') else []
    except (OSError, ValueError, StopIteration, TypeError):
        return []


@lru_cache(maxsize=512)
def _capture_valid(path, sample_count, size, modified_ns):
    # File stat keys invalidate the cache after a partial/corrupt file changes.
    # Cache validity only, rather than retaining every experiment's raw rows.
    return bool(capture_rows(os.path.dirname(path), dict(csv=os.path.basename(path),
                                                       sample_count=sample_count)))


def capture_valid(directory, entry):
    if not isinstance(entry, dict) or not isinstance(entry.get('csv'), str):
        return False
    try:
        path = os.path.realpath(os.path.join(directory, entry['csv']))
        if not path.startswith(os.path.realpath(directory) + os.sep):
            return False
        stat = os.stat(path)
        return _capture_valid(path, entry.get('sample_count'), stat.st_size, stat.st_mtime_ns)
    except (OSError, TypeError):
        return False


def saved_stages(directory, manifest, run_id):
    saved = {}
    for entry in manifest.get('captures', []):
        if isinstance(entry, dict) and isinstance(entry.get('stage_id'), str) and entry.get('run_id') == run_id and capture_valid(directory, entry):
            saved[entry['stage_id']] = entry
    return saved


def progress(directory, manifest=None):
    manifest = manifest or read_manifest(directory)
    counts = {count: 0 for count in COUNTS}
    for run in plan():
        expected = {stage['stage_id'] for stage in stages(manifest['settings'], run['magnet_count'])}
        if expected.issubset(saved_stages(directory, manifest, run['run_id'])):
            counts[run['magnet_count']] += 1
    return counts


def list_comparisons(data_dir, experiment_name=None):
    root = os.path.join(data_dir, 'magnet_evaluation')
    result = []
    if not os.path.isdir(root):
        return result
    for name in sorted(os.listdir(root), reverse=True):
        directory = os.path.join(root, name)
        try:
            manifest = read_manifest(directory)
            config = manifest['settings']
            if experiment_name is not None and config['experiment_name'] != experiment_name.strip():
                continue
            result.append(dict(directory=directory, run_id=name,
                               settings=config, input_source=manifest['input_source'],
                               counts=progress(directory, manifest)))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return result


def _finite(rows, start, stop):
    return [row for row in rows if all(math.isfinite(v) for v in row[start:stop])]


def metrics(rows, stage, config, baseline=None):
    """Raw-unit field diagnostics and pose errors; no inferred saturation rail."""
    field_rows = _finite(rows, 1, 49)
    poses = _finite(rows, 49, 55)
    means = [statistics.mean(row[i] for row in field_rows) for i in range(1, 49)] if field_rows else []
    noise = [statistics.pstdev(row[i] for row in field_rows) for i in range(1, 49)] if field_rows else []
    valid = [row for row in poses if abs(row[49]) <= .075 and abs(row[50]) <= .075
             and 0 <= abs(row[51]) - config['sensor_bias_m'] <= .15]
    teleop_valid = [row for row in poses if abs(row[49]) <= .05 and abs(row[50]) <= .05
                    and abs(row[51]) - config['sensor_bias_m'] <= .15]
    result = dict(sample_count=len(rows), rate_hz=len(rows) / stage['duration_s'],
                  finite_pose_percent=100 * len(poses) / len(rows) if rows else 0,
                  in_board_range_percent=100 * len(valid) / len(rows) if rows else 0,
                  in_teleop_range_percent=100 * len(teleop_valid) / len(rows) if rows else 0,
                  field_peak_raw=max((abs(v) for row in field_rows for v in row[1:49]), default=None),
                  field_noise_rms_raw=math.sqrt(statistics.mean(v*v for v in noise)) if noise else None,
                  field_mean_raw=means,
                  xy_rmse_mm=None, z_rmse_mm=None, jitter_xyz_mm=None)
    if stage['kind'] == 'static' and poses:
        result['xy_rmse_mm'] = math.sqrt(statistics.mean(
            (row[49]*1000-stage['x_mm'])**2 + (row[50]*1000-stage['y_mm'])**2 for row in poses))
        result['jitter_xyz_mm'] = 1000 * math.sqrt(sum(statistics.pvariance(row[i] for row in poses)
                                                       for i in (49, 50, 51)))
        if stage['magnet_centre_z_mm'] is not None:
            result['z_rmse_mm'] = math.sqrt(statistics.mean(
                ((abs(row[51])-config['sensor_bias_m'])*1000-stage['magnet_centre_z_mm'])**2
                for row in poses))
        if baseline and baseline.get('field_mean_raw') and means:
            result['field_signal_rms_raw'] = math.sqrt(statistics.mean(
                (value-base)**2 for value, base in zip(means, baseline['field_mean_raw'])))
    if stage['kind'] == 'sweep' and field_rows:
        # Constant channels during deliberate motion warrant inspection. This
        # is NOT proof of clipping: rails and firmware gain are not yet known.
        result['constant_field_channels'] = sum(max(row[i] for row in field_rows) ==
                                                min(row[i] for row in field_rows) for i in range(1, 49))
    gaps = []
    for a, b in zip(rows, rows[1:]):
        try:
            gaps.append(max(0, (datetime.fromisoformat(b[0])-datetime.fromisoformat(a[0])).total_seconds()))
        except (ValueError, TypeError):
            pass
    result['max_packet_gap_ms'] = 1000 * max(gaps) if gaps else None
    return result


def capture_metrics(directory, entry, stage, config):
    """Use cached diagnostics only when complete; CSV remains the reference."""
    value = entry.get('metrics')
    required = {'sample_count', 'rate_hz', 'finite_pose_percent', 'in_board_range_percent',
                'in_teleop_range_percent', 'field_peak_raw', 'field_noise_rms_raw',
                'field_mean_raw', 'xy_rmse_mm', 'z_rmse_mm', 'jitter_xyz_mm', 'max_packet_gap_ms'}
    if isinstance(value, dict) and required.issubset(value):
        numeric = [value[key] for key in required if key != 'field_mean_raw']
        means = value['field_mean_raw']
        valid = (isinstance(means, list) and len(means) in (0, 48) and
                 all(isinstance(number, (int, float)) and math.isfinite(number) for number in means) and
                 all(number is None or isinstance(number, (int, float)) and math.isfinite(number)
                     for number in numeric))
        if valid:
            return dict(value)
    return metrics(capture_rows(directory, entry), stage, config)


def report(directory, manifest):
    """Regenerate a concise comparison table and an exportable SVG chart."""
    config = manifest['settings']
    summaries = []
    for run in plan():
        saved = saved_stages(directory, manifest, run['run_id'])
        expected = stages(config, run['magnet_count'])
        values = {stage['stage_id']: capture_metrics(directory, saved[stage['stage_id']], stage, config)
                    for stage in expected if stage['stage_id'] in saved}
        baseline = values.get('baseline')
        if baseline and baseline.get('field_mean_raw'):
            for stage in expected:
                value = values.get(stage['stage_id'], {})
                if stage['kind'] == 'static' and value.get('field_mean_raw'):
                    value['field_signal_rms_raw'] = math.sqrt(statistics.mean(
                        (signal-base)**2 for signal, base in zip(value['field_mean_raw'], baseline['field_mean_raw'])))
        stationary = [values[s['stage_id']] for s in expected
                      if s['kind'] == 'static' and s['stage_id'] in values]
        captured = len({stage['stage_id'] for stage in expected}.intersection(saved))
        summary = dict(run, completed=captured == len(expected), captured=captured, planned=len(expected))
        for key in ('xy_rmse_mm', 'z_rmse_mm', 'jitter_xyz_mm', 'in_board_range_percent',
                    'in_teleop_range_percent', 'field_signal_rms_raw'):
            finite = [value[key] for value in stationary if value.get(key) is not None]
            summary[key] = ((math.sqrt(statistics.mean(v*v for v in finite)) if key in
                            ('xy_rmse_mm', 'z_rmse_mm') else statistics.mean(finite)) if finite else None)
        summary['baseline_noise_rms_raw'] = baseline['field_noise_rms_raw'] if baseline else None
        summary['field_peak_raw'] = max((value['field_peak_raw'] for value in values.values()
                                        if value['field_peak_raw'] is not None), default=None)
        summary['min_rate_hz'] = min((value['rate_hz'] for value in values.values()), default=None)
        summary['max_packet_gap_ms'] = max((value['max_packet_gap_ms'] for value in values.values()
                                           if value['max_packet_gap_ms'] is not None), default=None)
        summary['sweep'] = values.get('sweep', {})
        summaries.append(summary)
    write_json(os.path.join(directory, 'summary.json'), dict(input_source=manifest['input_source'],
                settings=config, runs=summaries, counts=progress(directory, manifest)))
    columns = ['run_id', 'magnet_count', 'repetition', 'completed', 'captured', 'planned',
               'xy_rmse_mm', 'z_rmse_mm', 'jitter_xyz_mm', 'in_teleop_range_percent',
               'baseline_noise_rms_raw', 'field_signal_rms_raw', 'field_peak_raw',
               'min_rate_hz', 'max_packet_gap_ms']
    with open(os.path.join(directory, 'summary.csv'), 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(summaries)
    def fmt(value):
        return '—' if value is None else '{:.1f}'.format(value)
    lines = ['# Magnet baseline comparison', '',
             '- Experiment: {}'.format(config['experiment_name'] or 'Unnamed'),
             '- Source: **{}**'.format(manifest['input_source'].upper()),
             '- Magnets: {}; arrangement: {}.'.format(config['magnet'], config['arrangement']),
             '- Nominal height: **0 mm above cardboard**; cardboard: **{:g} mm (approx.)**.'.format(config['spacer_mm']),
             '- 1 / 2 / 3 magnets × 3 runs; each run: magnet-away baseline, {}, {:g} s sweep.'.format(
                 'five named placements' if position_protocol(config) == FIVE_POSITIONS else
                 'original nine grid positions + one above-sensor position', config['sweep_s']),
             '- Placement protocol: {}.'.format(position_protocol(config)),
             '- Use the marked guide positions inside the board; hold the magnet axis upright.', '',
             '| Magnets | Repeat | Captures | XY RMS error (mm) | Z RMS error (approx. mm) | XYZ jitter (mm) | Poses in teleop range (%) |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for value in summaries:
        lines.append('| {} | {} | {}/{} | {} | {} | {} | {} |'.format(value['magnet_count'], value['repetition'],
                     value['captured'], value['planned'], fmt(value['xy_rmse_mm']),
                     fmt(value['z_rmse_mm']),
                     fmt(value['jitter_xyz_mm']), fmt(value['in_teleop_range_percent'])))
    lines += ['', '| Magnets | Repeat | Baseline noise (raw) | Signal RMS (raw) | Field peak (raw) | Min rate (Hz) | Max delivery gap (ms) | Constant sweep channels |',
              '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for value in summaries:
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} / 48 |'.format(
            value['magnet_count'], value['repetition'], fmt(value['baseline_noise_rms_raw']),
            fmt(value['field_signal_rms_raw']), fmt(value['field_peak_raw']), fmt(value['min_rate_hz']),
            fmt(value['max_packet_gap_ms']), value['sweep'].get('constant_field_channels', '—')))
    lines += ['', '![Static tracking comparison](comparison.svg)', '',
              '- Raw CSVs retain all board estimates, including failed/out-of-range poses.',
              '- “In board range” checks ±75 mm X/Y and 0–150 mm bias-corrected Z; it is a geometry diagnostic, not accuracy.',
              '- “In teleop range” checks ±50 mm X/Y and ≤150 mm corrected Z, matching the control input gate; finite pose data are required.',
              '- Rates divide received packets by the recording window. Gaps measure host delivery, not firmware sample timestamps.',
              '- XY RMS includes finite outliers. Error requires correct physical target placement.',
              '- Guide coordinates remain saved internally. Named corner marks are inside the board, not its physical outer edges.',
              '- XY/Z errors combine per-position mean squared errors equally; jitter is the mean root-sum-square of raw XYZ standard deviations.',
              '- Z errors in summary.json use supplied centre offsets; approximate stack geometry produces approximate ground truth. Blank offsets omit Z errors.',
              '- Existing height bias is assumed: abs(raw Z) − 10 mm. Raw Z is preserved; this test does not recalibrate firmware.',
              '- Field values retain firmware units. Constant sweep channels suggest inspection; they do not prove saturation.',
              '- Compare all three repeats, motion continuity, field peaks and jitter before choosing the pen baseline.',
              '- Simulation verifies the workflow; it cannot identify the best physical magnet stack.', '']
    with open(os.path.join(directory, 'report.md'), 'w', encoding='utf-8') as stream:
        stream.write('\n'.join(lines))
    finite = [value['xy_rmse_mm'] for value in summaries if value['completed'] and value['xy_rmse_mm'] is not None]
    limit = max(5, max(finite, default=5) * 1.15)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="720" height="320" viewBox="0 0 720 320">',
           '<rect width="720" height="320" fill="#f5f7fb" rx="18"/>',
           '<g font-family="sans-serif" fill="#1d1d1f"><text x="32" y="36" font-size="22" font-weight="bold">Static XY tracking error</text>',
           '<text x="32" y="58" font-size="12">{} · 0 mm above ~{} mm cardboard · complete runs only</text>'.format(
               escape(manifest['input_source'].upper()), config['spacer_mm'])]
    for tick in range(5):
        y = 245-tick*40
        svg += ['<path d="M80 {} H680" stroke="#dde4ec"/>'.format(y),
                '<text x="20" y="{}" font-size="12">{:.1f} mm</text>'.format(y+4,limit*tick/4)]
    for count, x in zip(COUNTS, (180, 380, 580)):
        for value in summaries:
            if value['magnet_count'] == count and value['completed'] and value['xy_rmse_mm'] is not None:
                y = 245-value['xy_rmse_mm']/limit*160
                svg.append('<circle cx="{}" cy="{}" r="6" fill="#0a84ff"/>'.format(x+(value['repetition']-2)*16,y))
        svg.append('<text x="{}" y="278" text-anchor="middle" font-size="15">{} magnet{}</text>'.format(x,count,'' if count==1 else 's'))
    svg += ['<text x="32" y="307" font-size="12">Three repeats per stack. Lower error is better; inspect jitter and motion separately.</text></g></svg>']
    with open(os.path.join(directory, 'comparison.svg'), 'w', encoding='utf-8') as stream:
        stream.write('\n'.join(svg))
    return summaries
