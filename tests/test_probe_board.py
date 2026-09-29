#!/usr/bin/env python3
"""Offline checks for tools/probe_board.py (no board needed)."""

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
    return [[(v + offset + rng.gauss(0, noise)) * scale for v in EARTH_UT * 16] + list(POSE)
            for _ in range(n)]


STEP_XY, STEP_Z = 1.202, 1.936     # MLX90393 GAIN_SEL 1 + RES 1 in uT, as on the real board


def grid_rows(n, seed=0, pose=POSE):
    """Baseline-subtracted readings on the sensor's count grid: (count - baseline) * step."""
    rng = random.Random(seed)
    rows = []
    for _ in range(n):
        row = []
        for _ in range(16):
            row += [(rng.randint(-3, 3) - 0.37) * STEP_XY, (rng.randint(-3, 3) - 0.37) * STEP_XY,
                    (rng.randint(-3, 3) - 0.61) * STEP_Z]
        rows.append(row + list(pose))
    return rows


def stream(rows, rate_hz=100.0, per_read=1, t0=0):
    """Bytes plus reads (t_ns, nbytes) for rows arriving at rate_hz, per_read packets per read."""
    data, reads = b'', []
    for i in range(0, len(rows), per_read):
        chunk = b''.join(packet(r[:48], r[48:] or POSE) for r in rows[i:i + per_read])
        data += chunk
        reads.append((t0 + int((i + per_read - 1) * 1e9 / rate_hz), len(chunk)))
    return data, reads


def rows_of(data):
    return [list(f) for _, f in pb.split_packets(data)['packets']]


def with_sensor1(rows_or_values, axis=0):
    """Rows with Earth field everywhere except one channel of Sensor 1."""
    rows = []
    for v in rows_or_values:
        row = list(EARTH_UT) * 16
        row[axis] = v
        rows.append(row)
    return rows


class FakeSerial:
    """Serves prepared bytes; any write, DTR, RTS or break call fails the test."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.closed = False
        self.flushes = 0

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        if not self.chunks:
            time.sleep(0.001)
            return b''
        return self.chunks.pop(0)

    def reset_input_buffer(self):
        self.flushes += 1        # host-side flush only; nothing reaches the board

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


SHORT_STEPS = tuple((label, name, action, 0.2, text) for label, name, action, _, text in pb.STEPS)
FAKE_PORTS = [{'device': '/dev/ttyACM0', 'description': 'Fake', 'vid': 0x303A, 'pid': 0x1001}]


def run_main(argv, open_port, prompt=lambda _: '', ports=FAKE_PORTS):
    """Run main() with short steps and fake ports; return (exit code, stdout)."""
    with mock.patch.object(pb, 'STEPS', SHORT_STEPS), redirect_stdout(io.StringIO()) as out:
        code = pb.main(argv, prompt=prompt, open_port=open_port, list_ports=lambda: ports)
    return code, out.getvalue()


def fake_opener(chunks, opened):
    def open_port(device, baudrate):
        opened.append(FakeSerial(chunks))
        return opened[-1]
    return open_port


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

    def test_misaligned_packet_at_a_partial_start_is_rejected(self):
        # A 0xAA in the partial packet has 0xBB exactly 217 bytes later, inside the first
        # real packet (every field float there is BB BB BB 3F).
        value = struct.unpack('<f', b'\xBB\xBB\xBB\x3F')[0]
        partial = b'\x00' * 5 + b'\xAA' + b'\x00' * 94
        data = partial + packet([value] * 48) * 3
        self.assertEqual(data[5 + 217], 0xBB)
        got = rows_of(data)
        self.assertEqual(len(got), 3)
        self.assertTrue(all(row[:48] == [value] * 48 for row in got))

    def test_arrival_is_the_read_with_the_last_byte(self):
        data = packet() + packet()
        reads = [(100, 200), (200, 100), (300, 136)]      # packet 1 ends in read 2, packet 2 in read 3
        ends = [end for end, _ in pb.split_packets(data)['packets']]
        self.assertEqual(pb.arrival_of(ends, reads), [(200, 1), (300, 2)])


class FirmwareText(unittest.TestCase):

    def test_partial_packet_at_open_is_not_text(self):
        data, reads = stream(earth_rows(20, noise=3.0, seed=5), t0=int(0.01e9))
        for cut in range(1, 218):
            step, _ = pb.analyze_step(data[cut:], [(reads[0][0], reads[0][1] - cut)] + reads[1:],
                                      open_ns=0, opened_here=True)
            self.assertEqual(step['text'], [], cut)
            self.assertFalse(pb.restart_evidence({'far': step})['restart'], cut)

    def test_banner_after_a_partial_packet_is_kept(self):
        data = packet()[118:] + b'MagPilot fw 1.2\r\n' + packet() * 3
        split = pb.split_packets(data)
        self.assertEqual([t for _, t in pb.find_text(data, split['skipped'])], ['MagPilot fw 1.2\r\n'])


class Timing(unittest.TestCase):

    def test_rate_and_batched_gaps(self):
        data, reads = stream(earth_rows(200), rate_hz=100.0, per_read=4)
        step, _ = pb.analyze_step(data, reads, open_ns=None, opened_here=False)
        t = step['timing']
        self.assertAlmostEqual(t['rate_hz'], 199 / 1.96, places=3)  # first at 0.03 s, last 1.99 s
        self.assertAlmostEqual(t['gap_fractions']['<1 ms'], 150 / 199, places=3)
        self.assertAlmostEqual(t['gap_fractions']['20-50 ms'], 49 / 199, places=3)
        self.assertEqual(t['median_packets_per_read'], 4)
        self.assertEqual(t['per_second']['median'], 100)

    def test_open_delays(self):
        data, reads = stream(earth_rows(10), t0=int(0.8e9))
        step, _ = pb.analyze_step(b'boot ok\n' + data, [(int(0.7e9), 8)] + reads,
                                  open_ns=0, opened_here=True)
        self.assertAlmostEqual(step['timing']['open_to_first_byte_s'], 0.7)
        self.assertAlmostEqual(step['timing']['open_to_first_packet_s'], 0.8)
        self.assertAlmostEqual(step['text'][0]['after_open_s'], 0.7)
        self.assertTrue(pb.restart_evidence({'far': step})['restart'])

    def test_reboot_pause_after_open(self):
        # Seen with a fake board on a pty: 3 packets from before the reset slip through
        # the open flush, then the board boots for 1.5 s. The rate used to include the
        # pause (85.5/s for a 100/s board) and the restart went unnoticed.
        stale, stale_reads = stream(earth_rows(3), per_read=3, t0=int(0.002e9))
        data, reads = stream(earth_rows(850, seed=1), per_read=3, t0=int(1.52e9))
        step, _ = pb.analyze_step(stale + data, stale_reads + reads, open_ns=0, opened_here=True)
        t = step['timing']
        self.assertAlmostEqual(t['rate_hz'], 100.0, delta=1.0)
        self.assertEqual(len(t['pauses_s']), 1)
        self.assertAlmostEqual(t['pauses_s'][0], 1.5, delta=0.05)
        self.assertAlmostEqual(t['pause_after_open_s'], 1.5, delta=0.05)
        evidence = pb.restart_evidence({'far': step})
        self.assertTrue(evidence['restart'])
        self.assertIn('stopped for 1.5', evidence['text'])
        answer = dict((q, a) for q, a, _ in pb.rate_answers({'steps': {'far': step}, 'meta': {}}))
        self.assertIn('not counting 1 pause(s)', answer['Sample rate'])

    def test_slow_jittery_stream_has_no_pauses(self):
        times = [int(i * 0.5e9 + (0.1e9 if i % 2 else 0)) for i in range(20)]  # 2/s, gaps 0.4-0.6 s
        t = pb.timing_stats(times, list(range(20)))
        self.assertEqual(t['pauses_s'], [])
        self.assertAlmostEqual(t['rate_hz'], 19 / 9.6, places=6)

    def test_random_bytes_are_not_a_board(self):
        rng = random.Random(3)
        garbage = bytes(rng.getrandbits(8) for _ in range(218000))
        step, _ = pb.analyze_step(garbage, [(1, len(garbage))], open_ns=0, opened_here=True)
        self.assertFalse(pb.far_stream_ok(step))
        data, reads = stream(earth_rows(100))
        step, _ = pb.analyze_step(data[60:], [(1, len(data) - 60)], open_ns=0, opened_here=True)
        self.assertTrue(pb.far_stream_ok(step))


class Units(unittest.TestCase):

    def guess(self, rows):
        return pb.guess_units(pb.field_stats(rows))

    def test_units_from_earth_field(self):
        self.assertEqual(self.guess(earth_rows(50))['unit'], 'uT')
        self.assertEqual(self.guess(earth_rows(50, scale=0.01))['unit'], 'gauss')
        self.assertEqual(self.guess(earth_rows(50, scale=1e-6))['unit'], 'tesla')

    def test_baseline_and_counts(self):
        rng = random.Random(2)
        baseline = self.guess([[rng.gauss(0, 0.3) for _ in range(48)] + list(POSE)
                               for _ in range(50)])
        self.assertTrue(baseline['baseline'])
        self.assertIn('baseline is probably subtracted', baseline['text'])
        counts = self.guess([[float(round(v / 0.15)) for v in row[:48]] + row[48:]
                             for row in earth_rows(50)])
        self.assertEqual(counts['unit'], 'counts')
        self.assertFalse(counts['baseline'])

    def test_integer_counts_with_baseline(self):
        rng = random.Random(4)
        rows = [[float(round(rng.gauss(0, 3))) for _ in range(48)] + list(POSE) for _ in range(100)]
        counts = self.guess(rows)
        self.assertEqual(counts['unit'], 'counts')
        self.assertTrue(counts['baseline'])
        self.assertIn('a baseline is probably subtracted', counts['text'])

    def test_units_from_value_steps(self):
        rows = grid_rows(200)
        steps = pb.axis_steps(rows)
        self.assertAlmostEqual(steps['xy'], STEP_XY, places=6)
        self.assertAlmostEqual(steps['z'], STEP_Z, places=6)
        units = pb.guess_units(pb.field_stats(rows), steps)
        self.assertEqual((units['unit'], units['to_ut'], units['confidence']),
                         ('uT', 1.0, 'measured'))
        self.assertEqual(units['settings'], [(1, 1), (4, 2), (7, 3)])
        self.assertTrue(units['baseline'])
        self.assertIn('X/Y readings move in steps of 1.202 and Z in steps of 1.936', units['text'])
        self.assertIn('so a baseline is subtracted', units['text'])
        gauss = [[v * 0.01 for v in row[:48]] + row[48:] for row in rows]
        self.assertEqual(pb.guess_units(pb.field_stats(gauss), pb.axis_steps(gauss))['unit'],
                         'gauss')

    def test_smooth_or_sparse_readings_have_no_step(self):
        rng = random.Random(5)
        self.assertIsNone(pb.value_step([rng.gauss(0, 2) for _ in range(100)]))
        self.assertIsNone(pb.value_step([0.0, 1.202, 2.404, 0.0]))
        self.assertEqual(pb.axis_steps(earth_rows(50)), {'xy': None, 'z': None})
        self.assertEqual(pb.mlx_settings(1.5, 2.0), (None, None, []))

    def test_averaged_counts_are_not_claimed_as_mg(self):
        rows = [[v / 0.15 + 0.25 for v in row[:48]] + row[48:] for row in earth_rows(50)]
        guess = self.guess(rows)
        self.assertEqual(guess['unit'], 'mG')
        self.assertEqual(guess['confidence'], 'unclear')
        self.assertIn('or raw counts at GAIN_SEL 7, RES 0', guess['text'])


class Saturation(unittest.TestCase):

    far = pb.field_stats(earth_rows(50))

    def test_clipping_is_found(self):
        ramp = [float(v) for v in range(0, 200, 10)] + [200.0] * 6
        ramp += [float(v) for v in range(190, 0, -10)]
        sat = pb.detect_saturation({'sweep': with_sensor1(ramp)}, self.far)
        found = [(c['channel'], c['value'], c['run']) for c in sat['clipped']]
        self.assertEqual(found, [('Sensor1_Bx', 200.0, 6)])

    def test_still_magnet_is_not_clipping(self):
        rng = random.Random(1)
        values = [80.0 + 0.15 * rng.choice((0, 0, 0, 1)) for _ in range(100)] + [80.15] * 4
        sat = pb.detect_saturation({'still': with_sensor1(values)}, self.far)
        self.assertEqual(sat['clipped'], [])

    def test_channel_pinned_for_the_whole_step(self):
        rows = []
        for i in range(100):
            bx = 3000.0 * math.sin(i / 10.0)
            rows.append([bx, -bx, 7929.6] + list(EARTH_UT) * 15)
        sat = pb.detect_saturation({'closest': rows}, self.far)
        self.assertEqual([(c['channel'], c['run']) for c in sat['clipped']], [('Sensor1_Bz', 100)])

    def test_plus_and_minus_full_scale_tie(self):
        values = [0.0, 1000.0, 4915.0, 4915.0, 2000.0, -1000.0, -4915.0, -4915.0, -4915.0, 0.0]
        sat = pb.detect_saturation({'closest': with_sensor1(values)}, self.far)
        self.assertEqual([(c['value'], c['run']) for c in sat['clipped']], [(-4915.0, 3)])

    def test_wrap_around(self):
        values = [100.0, 150.0, 180.0, -185.0, -150.0, -100.0]
        sat = pb.detect_saturation({'closest': with_sensor1(values)}, self.far)
        self.assertEqual([(w['from'], w['to']) for w in sat['wraps']], [(180.0, -185.0)])

    def test_sign_flip_alone_is_unclear_not_yes(self):
        values = [100.0, 150.0, 180.0, -185.0, -150.0, -100.0]
        sat = pb.detect_saturation({'closest': with_sensor1(values)}, self.far)
        sat['full_scale'] = pb.full_scale_note(sat, {'unit': 'uT', 'to_ut': 1.0})
        text, confidence = pb.saturation_answer(sat)
        self.assertEqual(confidence, 'unclear')
        self.assertTrue(text.startswith('no clipped channels found; sign flip between samples'))

    def test_full_scale_note(self):
        uT = {'unit': 'uT', 'to_ut': 1.0}
        above = pb.full_scale_note({'max_abs_xy': 6000.0, 'max_abs_z': 100.0}, uT)
        self.assertIn('above the full scale of GAIN_SEL 7', above)
        self.assertIn('HALLCONF 0xC table', above)
        below = pb.full_scale_note({'max_abs_xy': 900.0, 'max_abs_z': 900.0}, uT)
        self.assertTrue(below.endswith('below the smallest MLX90393 full scale (4915/7930 uT)'))
        pinned = pb.full_scale_note({'max_abs_xy': 32767 * 0.150, 'max_abs_z': 32767 * 0.242}, uT)
        self.assertIn('|Bxy| = 4915 uT', pinned)
        self.assertIn('matches the full scale of GAIN_SEL 7, RES 0', pinned)
        self.assertIn('probably clipped or wrapped', pinned)


    def test_full_scale_of_the_matching_settings(self):
        units = {'unit': 'uT', 'to_ut': 1.0, 'settings': [(1, 1), (4, 2), (7, 3)]}
        sat = {'max_abs_xy': 14700.0, 'max_abs_z': 22200.0}
        note = pb.full_scale_note(sat, units)
        self.assertIn('GAIN_SEL 1 + RES 1: 37 % / 35 % of 39387 / 63439 uT', note)
        self.assertIn('GAIN_SEL 4 + RES 2: 56 % / 52 %', note)
        self.assertIn('ruled out, because these readings exceed its range: GAIN_SEL 7 + RES 3',
                      note)
        self.assertNotIn('probably clipped', note)
        self.assertEqual(pb.settings_in_range(units, sat), [(1, 1), (4, 2)])


class PoseAndMoment(unittest.TestCase):

    def rows(self, moments, poses=None):
        poses = poses or [(0.01, -0.02, 0.017)] * len(moments)
        return [list(EARTH_UT) * 16 + list(p) + list(m) for p, m in zip(poses, moments)]

    def test_moment_kinds(self):
        unit = [(math.sin(a), 0.0, math.cos(a)) for a in (0.0, 0.3, 0.6)]
        self.assertEqual(pb.interpret_moment({'sweep': self.rows(unit)})['kind'], 'unit')
        fixed = pb.interpret_moment({'sweep': self.rows([(3 * x, 3 * y, 3 * z) for x, y, z in unit])})
        self.assertEqual((fixed['kind'], fixed['confidence']), ('fixed', 'likely'))
        fitted = [(0.0, 0.0, s) for s in (0.5, 1.0, 2.0)]
        self.assertEqual(pb.interpret_moment({'sweep': self.rows(fitted)})['kind'], 'fitted')
        zero = pb.interpret_moment({'sweep': self.rows([(0.0, 0.0, 0.0)] * 3)})
        self.assertEqual((zero['kind'], zero['confidence']), ('zero', 'measured'))
        self.assertIn('does not compute it', zero['text'])

    def test_pose_units(self):
        sweep = [(-0.05 + 0.02 * i, 0.04 - 0.015 * i, 0.017) for i in range(6)]
        self.assertEqual(pb.guess_pose_units(self.rows([(0, 0, 1)] * 6, sweep))['unit'], 'm')
        mm = [(-50.0 + 20 * i, 40.0 - 15 * i, 17.0) for i in range(6)]
        self.assertEqual(pb.guess_pose_units(self.rows([(0, 0, 1)] * 6, mm))['unit'], 'mm')

    def test_frozen_pose_claims_no_unit(self):
        frozen = pb.guess_pose_units(self.rows([(0, 0, 1)] * 5))
        self.assertIsNone(frozen['unit'])
        self.assertEqual(frozen['confidence'], 'unclear')
        self.assertIn('does not follow the stylus', frozen['text'])


class PortOpen(unittest.TestCase):

    magnet = [((i % 7) - 3) * 30.0 for i in range(48)]

    def plus(self, rows, sign):
        return [[v + sign * m for v, m in zip(row, self.magnet)] + row[48:] for row in rows]

    def test_no_change(self):
        far, again = earth_rows(100, seed=1), earth_rows(100, seed=2)
        self.assertEqual(pb.port_open_effect(far, far, again)['result'], 'none')
        still = self.plus(earth_rows(100, seed=3), +1)
        self.assertEqual(pb.port_open_effect(far, still, again, still)['result'], 'none')

    def test_stylus_not_seen_at_reopen(self):
        far, again = earth_rows(100, seed=1), earth_rows(100, seed=2)
        still = self.plus(earth_rows(100, seed=3), +1)
        effect = pb.port_open_effect(far, still, again, earth_rows(100, seed=5))
        self.assertIsNone(effect['result'])
        self.assertEqual(effect['confidence'], 'unclear')
        self.assertIn('stylus was not seen during reopen_magnet', effect['text'])

    def test_stylus_still_near_after_the_reopen(self):
        far = earth_rows(100, seed=1)                    # no magnet: the pose stays frozen
        reopen_magnet = self.plus(earth_rows(100, seed=3), +1)
        near = [[v + 0.1 * m for v, m in zip(row, self.magnet)] + row[48:]
                for row in earth_rows(100, seed=4)]
        drift = pb.port_open_effect(far, reopen_magnet, near, reopen_magnet)
        self.assertEqual((drift['result'], drift['confidence']), ('none', 'measured'))
        self.assertIn('the stylus field was still visible', drift['text'])
        self.assertIn('drift, or the stylus was not fully away', drift['text'])
        tracked = [row[:48] + [0.001 * i] + row[49:] for i, row in enumerate(near)]
        effect = pb.port_open_effect(far, reopen_magnet, tracked, reopen_magnet)
        self.assertEqual(effect['result'], 'none')
        self.assertIn('the firmware still tracked a magnet (its pose moved by 0.099 m',
                      effect['text'])
        self.assertIn('stylus was probably not far enough away', effect['text'])

    def test_baseline_recaptured(self):
        far = earth_rows(100, noise=0.3, seed=1)
        still = self.plus(earth_rows(100, seed=3), +1)
        reopen_far = self.plus(earth_rows(100, seed=4), -1)
        effect = pb.port_open_effect(far, still, reopen_far, earth_rows(100, seed=5))
        self.assertEqual(effect['result'], 'baseline')
        self.assertIn('open the port with the stylus far away', effect['text'])


class StartUp(unittest.TestCase):

    def test_setup_text_and_zeros_only_at_the_first_open(self):
        banner = b'*** Setup ongoing ***\r\nMux 0 detected\r\n'
        zeros = [[0.0] * 48 + list(POSE)] * 3
        data, reads = stream(zeros + grid_rows(100), t0=int(1.2e9))
        far, _ = pb.analyze_step(banner + data, [(int(0.9e9), len(banner))] + reads,
                                 open_ns=0, opened_here=True)
        again, reads = stream(grid_rows(100, seed=1), t0=int(0.02e9))
        reopen, _ = pb.analyze_step(again, reads, open_ns=0, opened_here=True)
        self.assertEqual((far['leading_zero_packets'], reopen['leading_zero_packets']), (3, 0))
        evidence = pb.restart_evidence({'far': far, 'reopen_magnet': reopen})
        self.assertFalse(evidence['restart'])
        self.assertIn('start-up only at the first open (first packet 1.20 s after opening in far',
                      evidence['text'])
        self.assertIn('opening the port again does not restart the board', evidence['text'])
        both = pb.restart_evidence({'far': far, 'reopen_magnet': far})
        self.assertTrue(both['restart'])


class BoardIdentity(unittest.TestCase):

    def test_pc_serial_ports_are_hidden(self):
        def port(device, vid):
            return mock.Mock(device=device, description='d', manufacturer=None, product=None,
                             vid=vid, pid=0x0483 if vid else None, serial_number=None)
        fake_serial = mock.Mock()
        fake_serial.tools.list_ports.comports.return_value = [
            port('/dev/ttyS%d' % i, None) for i in range(32)] + [port('/dev/ttyACM0', 0x16C0)]
        with mock.patch.object(pb, 'import_serial', return_value=fake_serial), \
                mock.patch.object(pb.glob, 'glob', return_value=[]):
            ports = pb.list_port_info()
        self.assertEqual([p['device'] for p in ports], ['/dev/ttyACM0'])

    def test_permission_hint(self):
        hint = pb.explain_port_error(OSError(13, 'Permission denied'))
        self.assertIn('log in again', hint)
        self.assertIn('sg dialout -c', hint)

    def test_host_device_in_container_keeps_by_id_name(self):
        link = '/host/dev/serial/by-id/usb-Teensyduino_USB_Serial_123-if00'
        globs = {'/host/dev/serial/by-id/*': [link], '/host/dev/ttyACM*': ['/host/dev/ttyACM0']}
        fake_serial = mock.Mock()
        fake_serial.tools.list_ports.comports.return_value = []
        with mock.patch.object(pb, 'import_serial', return_value=fake_serial), \
                mock.patch.object(pb.glob, 'glob', side_effect=lambda p: globs.get(p, [])), \
                mock.patch.object(pb.os.path, 'realpath', return_value='/dev/ttyACM0'):
            ports = pb.list_port_info()
        self.assertEqual(ports[0]['by_id'], [link])
        answer = pb.board_answer(ports[0])['text']
        self.assertIn('usb-Teensyduino_USB_Serial_123-if00', answer)
        self.assertIn('ls -l /dev/serial/by-id/ or lsusb', answer)

    def test_by_id_port_argument_matches_its_tty(self):
        chunks = [packet() for _ in range(30)]
        ports = [{'device': '/dev/ttyACM0', 'description': 'Teensy', 'vid': 0x16C0, 'pid': 0x0483}]
        with tempfile.TemporaryDirectory() as folder:
            link = os.path.join(folder, 'usb-Teensyduino-if00')
            os.symlink('/dev/ttyACM0', link)
            out_dir = os.path.join(folder, 'out')
            code, _ = run_main(['--output-dir', out_dir, '--port', link, '--listen-only'],
                               fake_opener(chunks, []), ports=ports)
            run = os.path.join(out_dir, os.listdir(out_dir)[0])
            meta = json.load(open(os.path.join(run, 'meta.json')))
        self.assertEqual(code, 0)
        self.assertEqual(meta['port_info']['vid'], 0x16C0)

    def test_fmt_prints_thousands_in_full(self):
        self.assertEqual(pb.fmt(4915.05, 3), '4915')
        self.assertEqual(pb.fmt(12345.0), '12345')
        self.assertEqual(pb.fmt(0.01734, 3), '0.0173')
        self.assertEqual(pb.fmt(None), 'n/a')


def write_fake_probe(folder, steps):
    """Save steps {name: (data, reads, opened_here)} the way run_probe does."""
    meta = {'port': '/dev/ttyACM0', 'baudrate': 921600, 'started_local': 'test', 'steps': [],
            'port_info': {'device': '/dev/ttyACM0', 'description': 'Teensy', 'vid': 0x16C0,
                          'pid': 0x0483}}
    for name, (data, reads, opened_here) in steps.items():
        pb.save_step(folder, name, {'data': data, 'reads': reads})
        meta['steps'].append({'name': name, 'start_ns': reads[0][0] - 10, 'end_ns': reads[-1][0],
                              'open_ns': 0, 'opened_here': opened_here})
    pb.write_json(os.path.join(folder, 'meta.json'), meta)


class Replay(unittest.TestCase):

    def test_replay_writes_expected_answers(self):
        magnet_field = [((i % 7) - 3) * 30.0 for i in range(48)]
        magnet_pose = [0.02, -0.01, 0.017, 0.0, 0.0, 1.0]
        magnet = [[v + m for v, m in zip(row[:48], magnet_field)] + magnet_pose
                  for row in earth_rows(100, seed=7)]
        sweep = [row[:48] + [-0.05 + 0.001 * i, 0.04 - 0.0008 * i] + magnet_pose[2:]
                 for i, row in enumerate(magnet)]
        steps = {}
        for name, rows, opened in (('far', earth_rows(300, seed=1), True), ('still', magnet, False),
                                   ('sweep', sweep, False), ('closest', magnet, False),
                                   ('reopen_magnet', magnet, True),
                                   ('reopen_far', earth_rows(300, seed=2), False)):
            data, reads = stream(rows, t0=int(0.01e9))
            steps[name] = (data, reads, opened)
        with tempfile.TemporaryDirectory() as folder:
            write_fake_probe(folder, steps)
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(pb.main(['--replay', folder]), 0)
            report = open(os.path.join(folder, 'report.md'), encoding='utf-8').read()
            self.assertTrue(os.path.exists(os.path.join(folder, 'report.json')))
        answers = report.split('## Answers')[1].split('## Steps')[0]
        self.assertIn('**Board** (likely): Teensy, VID:PID 16C0:0483 -> Teensy (PJRC)', answers)
        self.assertIn('**Sample rate** (measured): 100.0 packets/s', answers)
        self.assertIn('native USB: the baud rate does not limit the rate', answers)
        self.assertIn('**Units** (likely): uT', answers)
        self.assertIn('no baseline is re-captured at port open', answers)
        self.assertIn('no clipped or wrapped channels found', answers)
        self.assertIn('**Pose units** (likely): metres', answers)
        self.assertIn('-> 0.007 m', answers)
        self.assertIn('unit direction vector', answers)
        self.assertNotIn('exact field unit', report)
        self.assertIn('\nAnswers\n- Board (likely): Teensy', out.getvalue())
        self.assertNotIn('**', out.getvalue())

    def test_replay_of_a_board_that_takes_its_baseline_at_start_up(self):
        # Like the real board on 29 Sep: setup text and zero packets only at the first
        # open, readings on the MLX90393 count grid with a baseline subtracted.
        banner = b'*** Setup ongoing ***\r\nEnd of Program Setup\r\n'
        magnet_field = [((i % 7) - 3) * 300.0 for i in range(48)]

        def magnet(seed):
            return [[v + m for v, m in zip(row[:48], magnet_field)]
                    + [-0.05 + 0.001 * i, 0.04 - 0.0008 * i, -0.006, 0.0, 0.0, 1.0]
                    for i, row in enumerate(grid_rows(100, seed))]
        zeros = [[0.0] * 48 + list(POSE)] * 2
        far_data, far_reads = stream(zeros + grid_rows(300, seed=1), t0=int(1.2e9))
        steps = {'far': (banner + far_data, [(int(0.9e9), len(banner))] + far_reads, True)}
        for name, rows, opened in (('still', magnet(2), False), ('sweep', magnet(3), False),
                                   ('closest', magnet(4), False),
                                   ('reopen_magnet', magnet(5), True),
                                   ('reopen_far', grid_rows(300, seed=6), False)):
            data, reads = stream(rows, t0=int(0.01e9))
            steps[name] = (data, reads, opened)
        with tempfile.TemporaryDirectory() as folder:
            write_fake_probe(folder, steps)
            with redirect_stdout(io.StringIO()):
                self.assertEqual(pb.main(['--replay', folder]), 0)
            report = open(os.path.join(folder, 'report.md'), encoding='utf-8').read()
        answers = report.split('## Answers')[1].split('## Steps')[0]
        self.assertIn('**Units** (measured): uT: X/Y readings move in steps of 1.202 and Z in '
                      'steps of 1.936', answers)
        self.assertIn('yes, taken at start-up: far |B| is at the noise level, and the first 2 '
                      'packet(s) after opening the port were exactly 0', answers)
        self.assertIn('start-up only at the first open', answers)
        self.assertIn('no baseline is re-captured at port open: right after the reopen the '
                      'stylus field was still visible', answers)
        self.assertIn('Which of GAIN_SEL 1 + RES 1, GAIN_SEL 4 + RES 2, GAIN_SEL 7 + RES 3 the '
                      'firmware uses', report)

    def test_replay_of_a_wrong_folder(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(SystemExit) as caught:
                pb.main(['--replay', folder])
        self.assertIn('not a probe folder', str(caught.exception))


class Recorder(unittest.TestCase):

    def test_full_run_never_writes(self):
        chunks = [b'hello from fw\n'] + [packet() for _ in range(30)]
        opened = []
        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder], fake_opener(chunks, opened))
            run = os.path.join(folder, os.listdir(folder)[0])
            files = sorted(os.listdir(run))
            meta = json.load(open(os.path.join(run, 'meta.json')))
        self.assertEqual(code, 0)
        self.assertEqual(len(opened), 2)
        self.assertTrue(all(p.closed for p in opened))
        self.assertEqual(sum(p.flushes for p in opened), 4)     # still, sweep, closest, reopen_far
        flushed = [s['name'] for s in meta['steps'] if s['stale_bytes'] is not None]
        self.assertEqual(flushed, ['still', 'sweep', 'closest', 'reopen_far'])
        self.assertIn('reopen_far_reads.csv', files)
        self.assertIn('report.md', files)
        self.assertIn('Step 5a reopen_magnet', out)
        self.assertIn('Step 5b reopen_far', out)
        self.assertIn('Ctrl-C stops at any time', out)

    def test_listen_only_does_not_blame_skipped_steps(self):
        chunks = [packet() for _ in range(30)]
        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder, '--listen-only'], fake_opener(chunks, []))
            run = os.path.join(folder, os.listdir(folder)[0])
            report = open(os.path.join(run, 'report.md'), encoding='utf-8').read()
        self.assertEqual(code, 0)
        self.assertIn('Saturation (unclear): ' + pb.NOT_MEASURED, out)
        self.assertIn('Pose units (unclear): ' + pb.NOT_MEASURED, out)
        self.assertNotIn('restarts when the port opens', report)
        self.assertNotIn('How Pose_x', report)

    def test_garbage_far_step_stops_the_run(self):
        rng = random.Random(9)
        chunks = [bytes(rng.getrandbits(8) for _ in range(4096)) for _ in range(40)]
        opened = []
        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder], fake_opener(chunks, opened))
        self.assertEqual(len(opened), 1)
        self.assertIn('valid packet(s)', out)
        self.assertIn('Sample rate (unclear):', out)

    def test_closed_stdin_still_writes_a_report(self):
        calls = []

        def prompt(_):
            calls.append(1)
            if len(calls) == 3:
                raise EOFError
            return ''

        chunks = [packet() for _ in range(30)]
        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder], fake_opener(chunks, []), prompt=prompt)
            run = os.path.join(folder, os.listdir(folder)[0])
            files = os.listdir(run)
        self.assertEqual(code, 0)
        self.assertIn('docker exec -it', out)
        self.assertIn('report.md', files)

    def test_failed_open_exits_with_the_real_error(self):
        def open_port(device, baudrate):
            raise RuntimeError('Could not open %s: [Errno 13] Permission denied -> add yourself '
                               'to the dialout group' % device)

        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder], open_port)
            files = os.listdir(os.path.join(folder, os.listdir(folder)[0]))
        self.assertEqual(code, 1)
        self.assertIn('Nothing was recorded: Could not open /dev/ttyACM0', out)
        self.assertNotIn(pb.WRONG_PORT_HINT, out)
        self.assertNotIn('report.md', files)

    def test_failed_reopen_is_the_first_answer(self):
        opened = []

        def open_port(device, baudrate):
            if opened:
                raise RuntimeError('Could not open %s: port is busy' % device)
            opened.append(FakeSerial([packet() for _ in range(30)]))
            return opened[-1]

        with tempfile.TemporaryDirectory() as folder:
            code, out = run_main(['--output-dir', folder], open_port)
        self.assertEqual(code, 1)
        self.assertIn('\nAnswers\n- Recording failed (measured): Could not open', out)

    def test_unwritable_output_dir(self):
        with tempfile.TemporaryDirectory() as folder:
            blocker = os.path.join(folder, 'file')
            open(blocker, 'w').close()
            with self.assertRaises(SystemExit) as caught:
                run_main(['--output-dir', blocker], fake_opener([], []))
        self.assertIn('pass --output-dir somewhere writable', str(caught.exception))

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


class Questions(unittest.TestCase):

    def test_firmware_questions_only_ask_what_was_not_answered(self):
        r = {'units': {'confidence': 'likely', 'unit': 'uT'},
             'saturation': {'clipped': [], 'wraps': [{'channel': 'Sensor2_Bx'}]},
             'port_open': {'result': None}, 'moment': {'confidence': 'unclear'},
             'pose_units': {'confidence': 'unclear'}, 'steps': {'far': {}}}
        questions = '\n'.join(pb.firmware_questions(r))
        self.assertNotIn('exact field unit', questions)
        self.assertIn('overflows', questions)                  # a sign flip is not an answer
        self.assertNotIn('port opens', questions)              # reopen was never recorded
        self.assertNotIn('Pose_x', questions)                  # sweep was never recorded
        r['saturation']['clipped'] = [{'channel': 'Sensor1_Bz'}]
        r['units']['confidence'] = 'unclear'
        questions = '\n'.join(pb.firmware_questions(r))
        self.assertIn('exact field unit', questions)
        self.assertNotIn('overflows', questions)


if __name__ == '__main__':
    unittest.main()
