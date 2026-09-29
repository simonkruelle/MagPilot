#!/usr/bin/env python3
"""Offline checks for tools/probe_board.py (no board needed)."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import io
import math
import os
import random
import struct
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

import probe_board as pb


# --------------------------------------------------------------------------
# Fake data
# --------------------------------------------------------------------------

EARTH_UT = (20.0, 5.0, -40.0)   # |B| = 45 uT
POSE = (0.0, 0.0, 0.017, 0.0, 0.0, 1.0)


def packet(field=None, pose=POSE):
    """One 218-byte packet; `field` is 48 floats (default: Earth field on every sensor)."""
    field = list(field) if field is not None else list(EARTH_UT) * 16
    return b'\xAA' + struct.pack('<54f', *(field + list(pose))) + b'\xBB'


def earth_rows(n, scale=1.0, noise=0.3, seed=0, offset=0.0):
    """n rows of 54 floats: Earth field plus noise on every sensor, default pose."""
    rng = random.Random(seed)
    return [[(v + offset + rng.gauss(0, noise)) * scale for v in EARTH_UT * 16] + list(POSE) for _ in range(n)]


def stream(rows, rate_hz=100.0, per_read=1, t0=0):
    """Bytes plus reads (t_ns, nbytes) for rows arriving at rate_hz, per_read packets per read."""
    data, reads = b'', []
    for i in range(0, len(rows), per_read):
        chunk = b''.join(packet(r[:48], *([r[48:]] if len(r) > 48 else [])) for r in rows[i:i + per_read])
        data += chunk
        reads.append((t0 + int((i + per_read - 1) * 1e9 / rate_hz), len(chunk)))
    return data, reads


def rows_of(data):
    return [list(f) for _, f in pb.split_packets(data)['packets']]


class FakeSerial:
    """Serves prepared bytes; any write, DTR, RTS or break call fails the test."""

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

    def close(self):
        self.closed = True

    def write(self, data):
        raise AssertionError('probe wrote to the port')

    def send_break(self, *args):
        raise AssertionError('probe sent a break')

    def __setattr__(self, name, value):
        if name in ('dtr', 'rts', 'break_condition'):
            raise AssertionError('probe changed %s' % name)
        object.__setattr__(self, name, value)


SHORT_STEPS = tuple((name, action, 0.2, text) for name, action, _, text in pb.STEPS)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------

class PacketSplitting(unittest.TestCase):

    def test_garbage_banner_and_resync(self):
        good = packet()
        bad_tail = b'\xAA' + b'\x00' * 216 + b'\x00'          # header, wrong tail
        data = b'MagPilot fw 1.2\r\n' + b'\x01\x02' + good + bad_tail + good + b'\xAA\x10'
        split = pb.split_packets(data)
        self.assertEqual(len(split['packets']), 2)
        self.assertEqual(split['bytes_before_first_packet'], 19)
        self.assertGreaterEqual(split['resyncs'], 1)
        self.assertEqual(split['leftover_bytes'], 2)
        self.assertEqual(split['skipped_bytes'] + 2 * 218 + 2, len(data))
        text = pb.find_text(data, split['skipped'])
        self.assertEqual(text[0], (0, 'MagPilot fw 1.2\r\n'))

    def test_header_byte_inside_floats(self):
        value = struct.unpack('<f', b'\xAA\xAA\xAA\x3F')[0]
        rows = [[value] * 48, [1.0] * 48]
        data = b''.join(packet(r) for r in rows)
        got = rows_of(data)
        self.assertEqual(len(got), 2)
        self.assertEqual(got[0][0], value)

    def test_arrival_is_the_read_with_the_last_byte(self):
        data = packet() + packet()
        reads = [(100, 200), (200, 100), (300, 136)]      # packet 1 ends in read 2, packet 2 in read 3
        ends = [end for end, _ in pb.split_packets(data)['packets']]
        self.assertEqual(pb.arrival_of(ends, reads), [(200, 1), (300, 2)])


class Timing(unittest.TestCase):

    def test_rate_and_batched_gaps(self):
        data, reads = stream(earth_rows(200), rate_hz=100.0, per_read=4)
        step, _ = pb.analyze_step(data, reads, open_ns=None, opened_here=False)
        t = step['timing']
        self.assertAlmostEqual(t['rate_hz'], 200 / 1.96 * (199 / 200), delta=2.0)
        self.assertAlmostEqual(t['gap_fractions']['<1 ms'], 150 / 199, places=3)
        self.assertAlmostEqual(t['gap_fractions']['20-50 ms'], 49 / 199, places=3)
        self.assertEqual(t['median_packets_per_read'], 4)
        self.assertEqual(t['per_second']['median'], 100)

    def test_open_delays(self):
        data, reads = stream(earth_rows(10), t0=int(0.8e9))
        step, _ = pb.analyze_step(b'boot ok\n' + data, [(int(0.7e9), 8)] + reads, open_ns=0, opened_here=True)
        self.assertAlmostEqual(step['timing']['open_to_first_byte_s'], 0.7)
        self.assertAlmostEqual(step['timing']['open_to_first_packet_s'], 0.8)
        self.assertAlmostEqual(step['text'][0]['after_open_s'], 0.7)
        self.assertTrue(pb.restart_evidence({'far': step})['restart'])


class Units(unittest.TestCase):

    def guess(self, rows):
        return pb.guess_units(pb.field_stats(rows))

    def test_units_from_earth_field(self):
        self.assertEqual(self.guess(earth_rows(50))['unit'], 'uT')
        self.assertEqual(self.guess(earth_rows(50, scale=0.01))['unit'], 'gauss')
        self.assertEqual(self.guess(earth_rows(50, scale=1e-6))['unit'], 'tesla')

    def test_baseline_and_counts(self):
        rng = random.Random(2)
        baseline = self.guess([[rng.gauss(0, 0.3) for _ in range(48)] + list(POSE) for _ in range(50)])
        self.assertTrue(baseline['baseline'])
        self.assertIn('baseline is probably subtracted', baseline['text'])
        counts = self.guess([[float(round(v / 0.15)) for v in row[:48]] + row[48:] for row in earth_rows(50)])
        self.assertEqual(counts['unit'], 'counts')


class Saturation(unittest.TestCase):

    far = pb.field_stats(earth_rows(50))

    def test_clipping_is_found(self):
        ramp = [float(v) for v in range(0, 200, 10)] + [200.0] * 6 + [float(v) for v in range(190, 0, -10)]
        rows = [[r] + list(EARTH_UT[1:]) + list(EARTH_UT) * 15 for r in ramp]
        sat = pb.detect_saturation({'sweep': rows}, self.far)
        self.assertEqual([(c['channel'], c['value'], c['run']) for c in sat['clipped']], [('Sensor1_Bx', 200.0, 6)])

    def test_still_magnet_is_not_clipping(self):
        rng = random.Random(1)
        rows = [[80.0 + 0.15 * rng.choice((0, 0, 0, 1))] + list(EARTH_UT[1:]) + list(EARTH_UT) * 15 for _ in range(100)]
        rows += [[80.15] + list(EARTH_UT[1:]) + list(EARTH_UT) * 15] * 4
        sat = pb.detect_saturation({'still': rows}, self.far)
        self.assertEqual(sat['clipped'], [])

    def test_wrap_around(self):
        values = [100.0, 150.0, 180.0, -185.0, -150.0, -100.0]
        rows = [[v] + list(EARTH_UT[1:]) + list(EARTH_UT) * 15 for v in values]
        sat = pb.detect_saturation({'closest': rows}, self.far)
        self.assertEqual([(w['from'], w['to']) for w in sat['wraps']], [(180.0, -185.0)])

    def test_full_scale_note(self):
        sat = {'max_abs_xy': 6000.0, 'max_abs_z': 100.0}
        self.assertIn('GAIN_SEL 7', pb.full_scale_note(sat, {'unit': 'uT', 'to_ut': 1.0}))
        self.assertIn('no setting clips', pb.full_scale_note({'max_abs_xy': 900.0, 'max_abs_z': 900.0}, {'to_ut': 1.0}))


class PoseAndMoment(unittest.TestCase):

    def rows(self, moments):
        return [list(EARTH_UT) * 16 + [0.01, -0.02, 0.017] + list(m) for m in moments]

    def test_moment_kinds(self):
        unit = [(math.sin(a), 0.0, math.cos(a)) for a in (0.0, 0.3, 0.6)]
        self.assertEqual(pb.interpret_moment({'sweep': self.rows(unit)})['kind'], 'unit')
        fixed = [(3 * x, 3 * y, 3 * z) for x, y, z in unit]
        self.assertEqual(pb.interpret_moment({'sweep': self.rows(fixed)})['kind'], 'fixed')
        fitted = [(0.0, 0.0, s) for s in (0.5, 1.0, 2.0)]
        self.assertEqual(pb.interpret_moment({'sweep': self.rows(fitted)})['kind'], 'fitted')

    def test_pose_units(self):
        self.assertEqual(pb.guess_pose_units(self.rows([(0, 0, 1)] * 5))['unit'], 'm')
        mm = [list(EARTH_UT) * 16 + [40.0, -60.0, 17.0, 0.0, 0.0, 1.0]] * 5
        self.assertEqual(pb.guess_pose_units(mm)['unit'], 'mm')


class PortOpen(unittest.TestCase):

    def test_no_change(self):
        far, again = earth_rows(100, seed=1), earth_rows(100, seed=2)
        self.assertEqual(pb.port_open_effect(far, far, again)['result'], 'none')

    def test_baseline_recaptured(self):
        far = earth_rows(100, noise=0.3, seed=1)
        magnet = [((i % 7) - 3) * 30.0 for i in range(48)]
        still = [[v + m for v, m in zip(row, magnet)] + row[48:] for row in earth_rows(100, seed=3)]
        reopen_far = [[v - m for v, m in zip(row, magnet)] + row[48:] for row in earth_rows(100, seed=4)]
        effect = pb.port_open_effect(far, still, reopen_far)
        self.assertEqual(effect['result'], 'baseline')
        self.assertIn('open the port with the stylus far away', effect['text'])


def write_fake_probe(folder, steps):
    """Save steps {name: (data, reads, opened_here)} the way run_probe does."""
    meta = {'port': '/dev/ttyACM0', 'baudrate': 921600, 'started_local': 'test', 'steps': [],
            'port_info': {'device': '/dev/ttyACM0', 'description': 'Teensy', 'vid': 0x16C0, 'pid': 0x0483}}
    for name, (data, reads, opened_here) in steps.items():
        pb.save_step(folder, name, {'data': data, 'reads': reads})
        meta['steps'].append({'name': name, 'start_ns': reads[0][0] - 10, 'end_ns': reads[-1][0], 'open_ns': 0,
                              'opened_here': opened_here})
    pb.write_json(os.path.join(folder, 'meta.json'), meta)


class Replay(unittest.TestCase):

    def test_replay_writes_expected_answers(self):
        magnet_pose = [0.02, -0.01, 0.017, 0.0, 0.0, 1.0]
        magnet = [[v + 300.0 * (i == 5) for i, v in enumerate(row[:48])] + magnet_pose for row in earth_rows(100, seed=7)]
        steps = {}
        for name, rows, opened in (('far', earth_rows(300, seed=1), True), ('still', magnet, False),
                                   ('sweep', magnet, False), ('closest', magnet, False),
                                   ('reopen_magnet', magnet, True), ('reopen_far', earth_rows(300, seed=2), False)):
            data, reads = stream(rows, t0=int(0.01e9))
            steps[name] = (data, reads, opened)
        with tempfile.TemporaryDirectory() as folder:
            write_fake_probe(folder, steps)
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(pb.main(['--replay', folder]), 0)
            report = open(os.path.join(folder, 'report.md'), encoding='utf-8').read()
            self.assertTrue(os.path.exists(os.path.join(folder, 'report.json')))
        answers = report.split('## Answers')[1].split('## Steps')[0]
        self.assertIn('**Board:** Teensy, VID:PID 16C0:0483 -> Teensy (PJRC)', answers)
        self.assertIn('**Sample rate:** 100.0 packets/s', answers)
        self.assertIn('**Units:** uT', answers)
        self.assertIn('no baseline is re-captured at port open', answers)
        self.assertIn('no clipped or wrapped channels found', answers)
        self.assertIn('**Pose units:** metres', answers)
        self.assertIn('unit direction vector', answers)
        self.assertIn('## Answers', out.getvalue())


class Recorder(unittest.TestCase):

    def test_full_run_never_writes(self):
        chunks = [b'hello from fw\n'] + [packet() for _ in range(30)]
        ports = [{'device': '/dev/ttyACM0', 'description': 'Fake', 'vid': 0x303A, 'pid': 0x1001}]
        opened = []

        def open_port(device, baudrate):
            opened.append(FakeSerial(chunks))
            return opened[-1]

        with tempfile.TemporaryDirectory() as folder, mock.patch.object(pb, 'STEPS', SHORT_STEPS):
            with redirect_stdout(io.StringIO()):
                code = pb.main(['--output-dir', folder], prompt=lambda _: '', open_port=open_port,
                               list_ports=lambda: ports)
            run = os.path.join(folder, os.listdir(folder)[0])
            files = sorted(os.listdir(run))
        self.assertEqual(code, 0)
        self.assertEqual(len(opened), 2)
        self.assertTrue(all(p.closed for p in opened))
        self.assertIn('reopen_far_reads.csv', files)
        self.assertIn('report.md', files)

    def test_pseudo_terminal_end_to_end(self):
        try:
            import pty  # noqa: F401
            import tty
            import serial  # noqa: F401
        except ImportError:
            self.skipTest('pyserial or pty not available')
        master, slave = os.openpty()
        tty.setraw(slave)
        port = pb.open_serial_port(os.ttyname(slave), 921600)
        stop = threading.Event()

        def board():
            os.write(master, b'MagPilot boot v0.1\r\n')
            while not stop.is_set():
                os.write(master, packet())
                time.sleep(0.01)

        writer = threading.Thread(target=board, daemon=True)
        writer.start()
        try:
            with redirect_stdout(io.StringIO()):
                rec = pb.record_step(port, 1.0, 'pty')
        finally:
            stop.set()
            writer.join(1.0)
            port.close()
            os.close(master)
            os.close(slave)
        step, rows = pb.analyze_step(rec['data'], rec['reads'], None, False)
        self.assertIsNone(rec['stopped'])
        self.assertGreater(len(rows), 50)
        self.assertEqual(list(rows[0][:3]), list(EARTH_UT))
        self.assertTrue(70 < step['timing']['rate_hz'] < 110, step['timing']['rate_hz'])
        self.assertIn('MagPilot boot', step['text'][0]['text'])


if __name__ == '__main__':
    unittest.main()
