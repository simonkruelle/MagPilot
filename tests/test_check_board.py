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


def full_check(folder, already_running=False):
    """The board starts when the check opens the port; or it was already running (no setup
    text, no zero packets) and carries a 20 uT offset on S5's Bx from before, in every step."""
    rng = random.Random(1)
    if already_running:
        start = {'rows': noise_rows(int(60 * RATE_HZ), rng), 't0': 0.01}
    else:
        start = {'rows': start_rows(rng), 't0': 2.0, 'prefix_s': 0.9,
                 'prefix': b'*** Setup ongoing ***\r\nMux 0 detected\r\nEnd of Program Setup\r\n'}
    start.update(name='start', open_s=0.0, cues=[('away', 60.0)])
    steps = [
        start,
        {'name': 'weak', 'rows': weak_rows(rng), 't0': 80.0,
         'cues': [('hold 3 cm', 10.0), ('away', 60.0)]},
        {'name': 'strong', 'rows': strong_rows(rng), 't0': 160.0,
         'cues': [('hold on the cover', 10.0), ('away', 120.0)]},
        {'name': 'roll', 'rows': roll_rows(), 't0': 300.0,
         'cues': [('hold', 5.0), ('roll', 10.0), ('hold', 5.0)]},
    ]
    if already_running:
        for step in steps:
            for row in step['rows']:
                row[3 * 4] += 20.0
    write_folder(folder, steps)


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


SHORT_STEPS = tuple((label, name, text, tuple((0.05, cue) for _, cue in cues))
                    for label, name, text, cues in cb.STEPS)


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
        start = answers['Stylus away (zero reference)']
        self.assertIn('the board started when the check opened it (setup text, 2 zero '
                      'packet(s)), so its baseline is fresh', start)
        self.assertRegex(start, r'stylus-away readings: largest 0\.\d+ uT \(S\d+\), so they '
                                r'are at zero')
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
        roll = answers['Magnet direction (roll)']
        self.assertIn('the magnet direction is 80 deg from vertical and turned by 360 deg', roll)
        self.assertIn('middle sensors turned by 360 deg; the stylus position moved 0 mm', roll)
        self.assertIn('the magnet points across the stylus', roll)
        self.assertNotIn('Not recorded', answers)
        self.assertIn('| S7 | -17.5, -17.5 |', report)
        s7 = [line for line in report.splitlines() if line.startswith('| S7 |')][0]
        self.assertIn('| yes |', s7)

    def test_board_that_was_already_running(self):
        _, answers, _ = self.answers(already_running=True)
        start = answers['Stylus away (zero reference)']
        self.assertIn('the board was already running, so its baseline is from when it started',
                      start)
        self.assertRegex(start, r'largest (19|20|21)(\.\d)? uT \(S5\), so the board carries an '
                                r'offset from before')
        self.assertIn('no clear offset', answers['After a weak field (3 cm above the cover)'])

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
        self.assertEqual('start, weak, strong', answers['Not recorded'])


def run_main(argv, open_port, list_ports, prompt=lambda _: '', steps=SHORT_STEPS, readers=()):
    with mock.patch.object(cb, 'STEPS', steps), \
            mock.patch.object(cb, 'other_readers', lambda: list(readers)), \
            redirect_stdout(io.StringIO()) as out:
        code = cb.main(argv, prompt=prompt, open_port=open_port, list_ports=list_ports)
    return code, out.getvalue()


# Simon's run of 30 Sep: after a replug the firmware gave up on its multiplexer
MUX_FAILURE = (b'\n*** Setup ongoing ***\r\n\n*** Initialize I2C Network ***\r\n'
               b'\n*** done. ***\r\n\n*** Mux 0 not detected. Program freezing... '
               b'Check your Wiring. ***\r\n')


class Recorder(unittest.TestCase):

    def test_full_run_opens_once_and_never_writes(self):
        rng = random.Random(2)
        chunks = [b'*** Setup ongoing ***\r\n'] + [packet(row) for row in start_rows(rng)[:40]]
        opened, prompts = [], []

        def open_port(device, baudrate):
            opened.append((device, FakeSerial(chunks)))
            return opened[-1][1]

        with tempfile.TemporaryDirectory() as out_dir:
            code, output = run_main(['--output-dir', out_dir], open_port,
                                    FakePorts(board('/dev/ttyACM0')),
                                    prompt=lambda text: prompts.append(text) or '')
            run = os.path.join(out_dir, os.listdir(out_dir)[0])
            meta = json.load(open(os.path.join(run, 'meta.json'), encoding='utf-8'))
            files = sorted(os.listdir(run))
        self.assertEqual(code, 0, output)
        self.assertEqual([d for d, _ in opened], ['/dev/ttyACM0'])
        self.assertTrue(opened[0][1].closed)
        self.assertEqual([s['name'] for s in meta['steps']], ['start', 'weak', 'strong', 'roll'])
        self.assertEqual([len(s['segments']) for s in meta['steps']], [1, 2, 2, 3])
        self.assertEqual(meta['steps'][0]['open_ns'], meta['open_ns'])
        self.assertIn('Put the stylus at least 1 m away', prompts[0])
        self.assertEqual(len(prompts), 4)       # before the open, then steps 2-4
        self.assertNotIn('nplug', output)
        self.assertIn('\a>>> Take the stylus away', output)
        self.assertIn('report.md', files)
        self.assertIn('strong.bin', files)
        self.assertIn('Please zip this folder and send it', output)

    def test_board_that_stops_during_its_start_up(self):
        label, name, text, cues = SHORT_STEPS[0]
        steps = ((label, name, text, ((1.5, cues[0][1]),)),) + SHORT_STEPS[1:]
        with tempfile.TemporaryDirectory() as out_dir, \
                mock.patch.object(cb, 'FIRST_PACKET_S', 0.2):
            code, output = run_main(['--output-dir', out_dir],
                                    lambda device, baudrate: FakeSerial([MUX_FAILURE]),
                                    FakePorts(board('/dev/ttyACM0')), steps=steps)
            run = os.path.join(out_dir, os.listdir(out_dir)[0])
            meta = json.load(open(os.path.join(run, 'meta.json'), encoding='utf-8'))
            answers = dict(cb.analyze_folder(run)['answers'])
        self.assertEqual(code, 1)
        self.assertEqual([s['name'] for s in meta['steps']], ['start'])
        self.assertTrue(meta['steps'][0]['stopped'].startswith('no packets'))
        self.assertLess(meta['steps'][0]['end_ns'] - meta['steps'][0]['start_ns'], 1.4e9)
        self.assertIn('Stopped: no packets since the port opened: the board stopped during its '
                      'start-up; its last text was "*** Mux 0 not detected. Program '
                      'freezing... Check your Wiring. ***". Unplug the board', output)
        self.assertNotIn('Step 2', output)
        self.assertIn('The board sent no packets, so there is nothing to analyse', output)
        self.assertIn('no packets: the board stopped during its start-up after "*** Mux 0 not '
                      'detected.', answers['Stylus away (zero reference)'])
        self.assertIn('the board is not sending',
                      cb.no_packets_yet(b'', 2 * cb.FIRST_PACKET_S * 1e9))
        self.assertIsNone(cb.no_packets_yet(MUX_FAILURE, 0.5 * cb.FIRST_PACKET_S * 1e9))

    def test_launcher_still_reading_stops_before_the_port_opens(self):
        def open_port(device, baudrate):
            raise AssertionError('opened the port')

        with tempfile.TemporaryDirectory() as out_dir:
            with self.assertRaises(SystemExit) as stop:
                run_main(['--output-dir', out_dir], open_port, FakePorts(board('/dev/ttyACM0')),
                         readers=['magnetometer_reader.py'])
            self.assertEqual(os.listdir(out_dir), [])
        self.assertIn('magnetometer_reader.py is running and reads the board', str(stop.exception))

    def test_port_that_cannot_be_opened(self):
        def open_port(device, baudrate):
            raise RuntimeError('Could not open %s: Permission denied' % device)

        with tempfile.TemporaryDirectory() as out_dir:
            with self.assertRaises(SystemExit) as stop:
                run_main(['--output-dir', out_dir], open_port, FakePorts(board('/dev/ttyACM0')))
            self.assertEqual(os.listdir(out_dir), [])
        self.assertIn('Cannot run the check: Could not open /dev/ttyACM0: Permission denied',
                      str(stop.exception))

    def test_other_readers_are_found_by_their_command_line(self):
        commands = {'101': b'python3\0magnetometer_reader.py\0--port\0/dev/ttyACM0\0',
                    '102': b'docker\0exec\0colmag_simon\0bash\0-lc\0cd /colmag && python3 '
                           b'tools/record_tracking_error.py --port /dev/ttyACM0\0',
                    '103': b'sg\0dialout\0-c\0python3 tools/check_board.py\0',
                    '104': b'bash\0-lc\0pgrep -af \'[m]agnetometer_reader.py\'\0',
                    '105': b'nano\0magnetometer_reader.py\0',
                    '106': b'python3\0-m\0pytest\0tests/test_magnetometer_reader.py\0',
                    'self': b'python3\0probe_board.py\0'}
        with tempfile.TemporaryDirectory() as proc:
            for pid, command in commands.items():
                os.makedirs(os.path.join(proc, pid))
                with open(os.path.join(proc, pid, 'cmdline'), 'wb') as handle:
                    handle.write(command)
            os.makedirs(os.path.join(proc, '107'))      # ended: no cmdline left
            self.assertEqual(cb.other_readers(proc),
                             ['magnetometer_reader.py', 'record_tracking_error.py'])
        self.assertEqual(cb.other_readers(os.path.join(proc, 'gone')), [])

    def test_replay_of_a_wrong_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(SystemExit):
                cb.main(['--replay', folder])


if __name__ == '__main__':
    unittest.main()
