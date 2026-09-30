#!/usr/bin/env python3
"""Offline checks for tools/check_board.py (no board needed)."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import io
import json
import math
import os
import random
import struct
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

import check_board as cb
import probe_board as pb

RATE_HZ = 20.0          # fewer packets than the real 60/s keep the tests fast
NOISE_UT = 1.0
S6, S7 = 5, 6           # sensor indices


# --------------------------------------------------------------------------
# Fake data
# --------------------------------------------------------------------------

def packet(row):
    return b'\xAA' + struct.pack('<54f', *row) + b'\xBB'


def ns(seconds):
    return int(seconds * 1e9)


def noise_rows(n, rng, pose=(-0.02, 0.03, 0.015, 0.707, 0.0, 0.707)):
    return [[rng.gauss(0.0, NOISE_UT) for _ in range(48)] + list(pose) for _ in range(n)]


def start_rows(rng):
    zeros = [[0.0] * 54 for _ in range(2)]
    return zeros + noise_rows(int(58 * RATE_HZ), rng)


def weak_rows(rng):
    rows = noise_rows(int(70 * RATE_HZ), rng)
    for i, row in enumerate(rows[:int(10 * RATE_HZ)]):
        for s in cb.MIDDLE_SENSORS:
            row[3 * s + 2] += 500.0 * math.sin(math.pi * i / (10 * RATE_HZ))
    return rows


def strong_rows(rng):
    """10 s on the cover (Sensor7_Bx stuck near -14.2 mT while its Bz swings), then 120 s away
    with an offset on S7 that fades from about 23 to 20 uT, and 2 uT on S6."""
    hold, away = int(10 * RATE_HZ), int(120 * RATE_HZ)
    rows = noise_rows(hold + away, rng)
    for i, row in enumerate(rows[:hold]):
        row[3 * S7] = -14200.0 + rng.uniform(-5, 5)
        row[3 * S7 + 2] += -3000.0 + 6000.0 * i / hold
        row[3 * S6] += 3000.0 * math.sin(math.pi * i / hold)
    for i, row in enumerate(rows[hold:]):
        fade = 1.0 - 0.15 * i / away
        for c, v in zip(range(3 * S7, 3 * S7 + 3), (20.0, -5.0, 10.0)):
            row[c] += v * fade
        row[3 * S6] += 2.0
    return rows


def roll_rows():
    """Upright on the centre, magnet 80 deg from vertical, rolled one turn in the middle 10 s."""
    rows = []
    n_hold, n_roll = int(5 * RATE_HZ), int(10 * RATE_HZ)
    for i in range(2 * n_hold + n_roll):
        turn = min(max(i - n_hold, 0), n_roll) / float(n_roll)
        az = 2 * math.pi * turn
        field = [0.0] * 48
        for k, s in enumerate(cb.MIDDLE_SENSORS):
            field[3 * s] = 3000.0 * math.cos(az + k * math.pi / 2)
            field[3 * s + 1] = 3000.0 * math.sin(az + k * math.pi / 2)
        tilt = math.radians(80)
        rows.append(field + [0.001, -0.002, -0.005, math.sin(tilt) * math.cos(az),
                             math.sin(tilt) * math.sin(az), math.cos(tilt)])
    return rows


def write_folder(folder, steps):
    """Save steps the way run_check does. Each step: name, rows, t0 (s), cues [(text, s)],
    optional open_s and prefix bytes (setup text) arriving at prefix_s."""
    meta = {'port': '/dev/ttyACM0', 'started_local': 'test', 'steps': [], 'openings': []}
    for st in steps:
        data, reads = b'', []
        if st.get('prefix'):
            data += st['prefix']
            reads.append((ns(st['prefix_s']), len(st['prefix'])))
        for i, row in enumerate(st['rows']):
            p = packet(row)
            data += p
            reads.append((ns(st['t0'] + i / RATE_HZ), len(p)))
        pb.save_step(folder, st['name'], {'data': data, 'reads': reads})
        segments, t = [], st['t0']
        for text, seconds in st['cues']:
            segments.append({'cue': text, 'start_ns': ns(t), 'end_ns': ns(t + seconds)})
            t += seconds
        meta['steps'].append({'name': st['name'], 'start_ns': segments[0]['start_ns'],
                              'end_ns': segments[-1]['end_ns'], 'open_ns': (
                                  ns(st['open_s']) if 'open_s' in st else None),
                              'opened_here': 'open_s' in st, 'segments': segments,
                              'packets': len(st['rows'])})
    pb.write_json(os.path.join(folder, 'meta.json'), meta)


def full_check(folder, replug_prefix=b'*** Setup ongoing ***\r\nEnd of Program Setup\r\n',
               replug_zeros=1):
    rng = random.Random(1)
    banner = b'*** Setup ongoing ***\r\nMux 0 detected\r\nEnd of Program Setup\r\n'
    replug_rows = [[0.0] * 54 for _ in range(replug_zeros)] + noise_rows(int(29 * RATE_HZ), rng)
    write_folder(folder, [
        {'name': 'start', 'rows': start_rows(rng), 't0': 2.0, 'open_s': 0.0, 'prefix': banner,
         'prefix_s': 0.9, 'cues': [('away', 60.0)]},
        {'name': 'weak', 'rows': weak_rows(rng), 't0': 80.0,
         'cues': [('hold 3 cm', 10.0), ('away', 60.0)]},
        {'name': 'strong', 'rows': strong_rows(rng), 't0': 160.0,
         'cues': [('hold on the cover', 10.0), ('away', 120.0)]},
        {'name': 'replug', 'rows': replug_rows, 't0': 402.0, 'open_s': 400.0,
         'prefix': replug_prefix, 'prefix_s': 400.8, 'cues': [('away', 30.0)]},
        {'name': 'roll', 'rows': roll_rows(), 't0': 450.0,
         'cues': [('hold', 5.0), ('roll', 10.0), ('hold', 5.0)]},
    ])


class FakeSerial:
    """Serves prepared chunks; any write, DTR, RTS or break call fails the test."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        if not self.chunks:
            time.sleep(0.001)
            return b''
        return self.chunks.pop(0)

    def reset_input_buffer(self):
        pass                     # host-side flush only; nothing reaches the board

    def close(self):
        self.closed = True

    def write(self, data):
        raise AssertionError('check wrote to the port')

    def send_break(self, *args):
        raise AssertionError('check sent a break')

    def __setattr__(self, name, value):
        if name in ('dtr', 'rts', 'break_condition'):
            raise AssertionError('check changed %s' % name)
        object.__setattr__(self, name, value)


class FakePorts:
    """list_ports() that plays a script of port lists (the last one repeats)."""

    def __init__(self, *script):
        self.script = list(script)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.script.pop(0) if len(self.script) > 1 else self.script[0]


def board(device):
    return [{'device': device, 'description': 'USB Serial', 'vid': 0x16C0, 'pid': 0x0483,
             'serial_number': '18143760', 'by_id': []}]


SHORT_STEPS = tuple((label, name, action, text, tuple((0.05, cue) for _, cue in cues))
                    for label, name, action, text, cues in cb.STEPS)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------

class Helpers(unittest.TestCase):

    def test_sensor_positions_follow_the_measured_grid(self):
        self.assertEqual(cb.sensor_xy_mm(0), (-52.5, 52.5))     # S1
        self.assertEqual(cb.sensor_xy_mm(3), (-52.5, -52.5))    # S4
        self.assertEqual(cb.sensor_xy_mm(6), (-17.5, -17.5))    # S7, a middle sensor
        self.assertEqual(cb.sensor_xy_mm(12), (52.5, 52.5))     # S13: 105 mm from S1
        self.assertEqual(sorted(cb.sensor_xy_mm(s) for s in cb.MIDDLE_SENSORS),
                         [(-17.5, -17.5), (-17.5, 17.5), (17.5, -17.5), (17.5, 17.5)])

    def test_unwrap_adds_up_a_full_turn(self):
        angles = [math.degrees(math.atan2(math.sin(a), math.cos(a)))
                  for a in [2 * math.pi * i / 50 for i in range(51)]]
        unwrapped = cb.unwrap_degrees(angles)
        self.assertAlmostEqual(unwrapped[-1] - unwrapped[0], 360.0, places=6)


class Analysis(unittest.TestCase):

    def answers(self, **kwargs):
        with tempfile.TemporaryDirectory() as folder:
            full_check(folder, **kwargs)
            r = cb.analyze_folder(folder)
            cb.write_report(r, folder)
            report = open(os.path.join(folder, 'report.md'), encoding='utf-8').read()
            json.load(open(os.path.join(folder, 'report.json'), encoding='utf-8'))
        return r, dict(r['answers']), report

    def test_offset_after_a_strong_field_only(self):
        r, answers, report = self.answers()
        start = answers['Stylus away, fresh baseline']
        self.assertIn('fresh start-up at step 1: yes (setup text, 2 zero packet(s))', start)
        self.assertRegex(start, r'noise (0\.9\d|1(\.0\d)?) uT \(X/Y\)')
        weak = answers['After a weak field (3 cm above the cover)']
        self.assertIn('no clear offset (every sensor under 3 uT)', weak)
        strong = answers['After a strong field (on the cover)']
        self.assertIn('flat top) on Sensor7_Bx', strong)
        self.assertIn('yes, 1 sensor(s) at 3 uT or more', strong)
        self.assertRegex(strong, r'largest S7 2\d, S6 2(\.\d)?, ')
        self.assertRegex(strong, r'fading: S7 2[1-3](\.\d+)? uT 10-30 s after, '
                                 r'(19|20)(\.\d+)? uT in the last 20 s \(-1\d %\)')
        self.assertEqual(r['strong']['reference'], 'weak')
        replug = answers['Replug (new baseline)']
        self.assertIn('fresh start-up: yes (setup text, 1 zero packet(s))', replug)
        roll = answers['Magnet direction (roll)']
        self.assertIn('the magnet direction is 80 deg from vertical and turned by 360 deg', roll)
        self.assertIn('middle sensors turned by 360 deg; the stylus position moved 0 mm', roll)
        self.assertIn('the magnet points across the stylus', roll)
        self.assertNotIn('Not recorded', answers)
        self.assertIn('| S7 | -17.5, -17.5 |', report)
        s7 = [line for line in report.splitlines() if line.startswith('| S7 |')][0]
        self.assertIn('| yes |', s7)

    def test_replug_without_a_restart_is_flagged(self):
        _, answers, _ = self.answers(replug_prefix=b'', replug_zeros=0)
        self.assertIn('fresh start-up: NO: no setup text and no zero packets',
                      answers['Replug (new baseline)'])

    def test_upright_magnet_points_along_the_stylus(self):
        rows = roll_rows()
        for row in rows:
            row[48 + 3:] = [0.0, 0.05, 0.9987]
        with tempfile.TemporaryDirectory() as folder:
            write_folder(folder, [{'name': 'roll', 'rows': rows, 't0': 0.0,
                                   'cues': [('hold', 5.0), ('roll', 10.0), ('hold', 5.0)]}])
            answers = dict(cb.analyze_folder(folder)['answers'])
        self.assertIn('3 deg from vertical: it points along the stylus', answers[
            'Magnet direction (roll)'])
        self.assertIn('start, weak, strong, replug', answers['Not recorded'])


def run_main(argv, open_port, list_ports, prompt=lambda _: ''):
    with mock.patch.object(cb, 'STEPS', SHORT_STEPS), redirect_stdout(io.StringIO()) as out:
        code = cb.main(argv, prompt=prompt, open_port=open_port, list_ports=list_ports,
                       sleep=lambda _: None)
    return code, out.getvalue()


class Recorder(unittest.TestCase):

    def test_full_run_with_two_replugs_never_writes(self):
        rng = random.Random(2)
        banner = b'*** Setup ongoing ***\r\n'
        first = [banner] + [packet(row) for row in start_rows(rng)[:40]]
        second = [banner] + [packet(row) for row in noise_rows(40, rng)]
        opened, prompts = [], []

        def open_port(device, baudrate):
            opened.append((device, FakeSerial(first if not opened else second)))
            return opened[-1][1]

        ports = FakePorts(board('/dev/ttyACM0'), [], board('/dev/ttyACM1'), [],
                          board('/dev/ttyACM0'))
        with tempfile.TemporaryDirectory() as out_dir:
            code, output = run_main(['--output-dir', out_dir], open_port, ports,
                                    prompt=lambda text: prompts.append(text) or '')
            run = os.path.join(out_dir, os.listdir(out_dir)[0])
            meta = json.load(open(os.path.join(run, 'meta.json'), encoding='utf-8'))
            files = sorted(os.listdir(run))
        self.assertEqual(code, 0, output)
        self.assertEqual([d for d, _ in opened], ['/dev/ttyACM1', '/dev/ttyACM0'])
        self.assertTrue(all(port.closed for _, port in opened))
        self.assertEqual([s['name'] for s in meta['steps']],
                         ['start', 'weak', 'strong', 'replug', 'roll'])
        self.assertEqual([len(s['segments']) for s in meta['steps']], [1, 2, 2, 1, 3])
        self.assertIn('Unplug the board', output)
        self.assertIn('Plugged in as /dev/ttyACM1', prompts[0])
        self.assertIn('\a>>> Take the stylus away', output)
        self.assertIn('report.md', files)
        self.assertIn('strong.bin', files)
        self.assertIn('Please zip this folder and send it', output)

    def test_replug_that_never_happens(self):
        ports = FakePorts(board('/dev/ttyACM0'))
        with tempfile.TemporaryDirectory() as out_dir, \
                mock.patch.object(cb, 'PLUG_WAIT_S', 0.01):
            code, output = run_main(['--output-dir', out_dir],
                                    lambda device, baudrate: FakeSerial([]), ports)
        self.assertEqual(code, 1)
        self.assertIn('The board was not unplugged within', output)
        self.assertIn('run the check on the host', output)
        self.assertIn('Nothing was recorded: The board was not unplugged', output)

    def test_port_that_opens_on_the_second_try(self):
        attempts = []

        def open_port(device, baudrate):
            attempts.append(device)
            if len(attempts) == 1:
                raise RuntimeError('Could not open %s: Permission denied' % device)
            return FakeSerial([])

        port, open_ns = cb.open_after_plug(open_port, '/dev/ttyACM0', 921600, lambda _: None)
        self.assertEqual(len(attempts), 2)
        self.assertIsInstance(open_ns, int)

    def test_replay_of_a_wrong_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(SystemExit):
                cb.main(['--replay', folder])


if __name__ == '__main__':
    unittest.main()
