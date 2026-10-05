"""Finite packet framing, serial lifecycle and real local PTY bridge checks."""

import io
import json
import os
import select
import signal
import struct
import subprocess
import sys
import threading
import time
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from colmag.teleoperation_input import (  # noqa: E402
    MAX_SERIAL_BACKLOG_BYTES, PacketFramer, SerialInput, finite_packet,
)
from tools.teleoperation_serial_stream import stream_serial, watch_stdin  # noqa: E402

try:
    import pty
    import serial
except ImportError:
    pty = serial = None


def packet(pose=None, first_field=0.0):
    fields = [first_field] + [value / 10.0 for value in range(1, 48)]
    pose = pose or [0.01, 0.02, 0.03, 0, 0, 1]
    return b'\xaa' + struct.pack('<54f', *(fields + pose)) + b'\xbb'


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    raise AssertionError('Expected asynchronous state was not reached.')


class PacketFramingTests(unittest.TestCase):
    def test_an_old_partial_packet_cannot_be_revived_after_a_feedback_pause(self):
        framer = PacketFramer()
        data = packet()
        self.assertEqual(framer.feed(data[:40], 10), [])
        self.assertEqual(framer.feed(b'', 10.1), [])
        self.assertEqual(framer.feed(data[40:], 11), [])
        self.assertEqual(len(framer.feed(data, 11.1)), 1)

    def test_garbage_partial_chunks_and_multiple_packets_resynchronise(self):
        framer = PacketFramer()
        first = packet()
        second = packet([0.11, 0.02, 0.03, 0, 0, 1])
        self.assertEqual(framer.feed(b'garbage' + first[:57]), [])
        recovered = framer.feed(first[57:] + b'noise' + second)
        self.assertEqual(len(recovered), 2)
        self.assertAlmostEqual(recovered[0]['pose'][0], 0.01)
        self.assertAlmostEqual(recovered[1]['pose'][0], 0.11)
        self.assertEqual(len(recovered[0]['magnetic_fields']), 48)
        self.assertEqual(len(framer.buffer), 0)

    def test_corrupt_tail_nan_and_infinity_do_not_become_control_samples(self):
        framer = PacketFramer()
        corrupt = b'\xaa' + b'\x00' * 216 + b'\x00'
        nonfinite_pose = packet([float('nan'), 0, 0.03, 0, 0, 1])
        nonfinite_fields = packet(first_field=float('inf'))
        recovered = framer.feed(corrupt + nonfinite_pose + nonfinite_fields + packet())
        self.assertEqual(len(recovered), 1)
        self.assertAlmostEqual(recovered[0]['pose'][2], 0.03)

    def test_long_unframed_garbage_does_not_accumulate(self):
        framer = PacketFramer()
        self.assertEqual(framer.feed(b'\x00' * 200000), [])
        self.assertEqual(len(framer.buffer), 0)

    def test_bridge_payload_must_have_complete_finite_pose_and_fields(self):
        message = dict(pose=[0] * 6, magnetic_fields=[0] * 48)
        self.assertIsNotNone(finite_packet(message))
        for malformed in (None, [], {}, dict(message, pose=[0] * 5),
                          dict(message, magnetic_fields=[0] * 47),
                          dict(message, pose=[float('nan')] * 6),
                          dict(message, magnetic_fields=[float('inf')] * 48)):
            self.assertIsNone(finite_packet(malformed))


class SerialInputLifecycleTests(unittest.TestCase):
    def test_docker_drops_old_buffered_packets_with_a_different_source_clock_epoch(self):
        base = dict(pose=[0] * 6, magnetic_fields=[1] * 48)
        messages = [dict(type='clock_sync', nonce=1, source_monotonic_s=1000),
                    dict(type='clock_sync', nonce=2, source_monotonic_s=1000),
                    dict(base, pose=[9] * 6, source_monotonic_s=1000.05),
                    dict(base, pose=[1] * 6, source_monotonic_s=1000.50)]
        process = mock.Mock()
        process.stdin = io.StringIO()
        process.stdout = io.StringIO('\n'.join(json.dumps(item) for item in messages) + '\n')
        process.stderr = io.StringIO()
        process.returncode = 0
        process.wait.return_value = 0
        with mock.patch('colmag.teleoperation_input.threading.Thread.start'):
            source = SerialInput('/host/dev/ttyACM0')
        with mock.patch('colmag.teleoperation_input.subprocess.Popen', return_value=process), \
                mock.patch('colmag.teleoperation_input.time.monotonic',
                           side_effect=[100, 100.01, 100.01, 100.02, 100.50, 100.52, 100.53]):
            with self.assertRaisesRegex(RuntimeError, 'disconnected'):
                source._docker_worker()
        sample = source.latest()
        self.assertEqual(sample['pose'], [1] * 6)
        self.assertEqual(sample['received_monotonic_s'], 100.52)
        self.assertAlmostEqual(sample['source_age_s'], .01)
        self.assertAlmostEqual(sample['clock_sync_uncertainty_s'], .01)

    def test_large_serial_backlog_is_flushed_and_only_newest_chunk_packet_is_sent(self):
        stop = threading.Event()

        class Connection:
            resets = 0
            closed = False

            @property
            def in_waiting(self):
                return MAX_SERIAL_BACKLOG_BYTES + 1 if self.resets == 1 else 436

            def reset_input_buffer(self):
                self.resets += 1

            def read(self, count):
                return packet() + packet([.11, .02, .03, 0, 0, 1])

            def close(self):
                self.closed = True

        class Output(io.StringIO):
            def write(self, text):
                result = super().write(text)
                stop.set()
                return result

        connection, output = Connection(), Output()
        self.assertEqual(stream_serial('dummy', 921600, stop, output,
                                       serial_factory=lambda **kwargs: connection), 0)
        message = json.loads(output.getvalue())
        self.assertAlmostEqual(message['pose'][0], .11)
        self.assertGreater(message['source_monotonic_s'], 0)
        self.assertEqual(connection.resets, 2)
        self.assertTrue(connection.closed)

    def test_docker_bridge_uses_argument_list_host_timestamp_and_reports_disconnect(self):
        message = dict(pose=[0] * 6, magnetic_fields=[1] * 48,
                       received_monotonic_s=-123, source_monotonic_s=-1000)
        process = mock.Mock()
        process.stdin = io.StringIO()
        process.stdout = io.StringIO('bad JSON\n' + '\n'.join(json.dumps(item) for item in (
            dict(type='clock_sync', nonce=1, source_monotonic_s=-1000),
            dict(type='clock_sync', nonce=2, source_monotonic_s=-1000), message)) + '\n')
        process.stderr = io.StringIO('board disconnected')
        process.returncode = 1
        process.wait.return_value = 1
        before = time.monotonic()
        with mock.patch('colmag.teleoperation_input.subprocess.Popen', return_value=process) as start:
            source = SerialInput('/host/dev/ttyACM0', container='test-container')
            try:
                sample = wait_for(source.latest)
                wait_for(lambda: source.error)
                self.assertGreaterEqual(sample['received_monotonic_s'], before)
                self.assertNotEqual(sample['received_monotonic_s'], -123)
                self.assertIn('board disconnected', source.error)
                command = start.call_args[0][0]
                self.assertEqual(command[:4], ['docker', 'exec', '-i', 'test-container'])
                self.assertIn('/colmag/tools/teleoperation_serial_stream.py', command)
                self.assertEqual(command[-4:], ['--port', '/host/dev/ttyACM0', '--baudrate', '921600'])
                self.assertGreaterEqual(sample['source_age_s'], 0)
                self.assertLessEqual(sample['clock_sync_uncertainty_s'], 0.125)
                self.assertNotIn('shell', start.call_args[1])
            finally:
                source.close()
                source.close()
            self.assertTrue(process.stdin.closed)
            self.assertFalse(source._thread.is_alive())

    def test_close_requests_docker_eof_without_killing_a_cooperative_bridge(self):
        ended = threading.Event()

        class InputPipe:
            closed = False

            def write(self, text):
                return len(text)

            def flush(self):
                pass

            def close(self):
                self.closed = True
                ended.set()

        class OutputPipe:
            closed = False

            def readline(self):
                ended.wait(2)
                return ''

            def close(self):
                self.closed = True

        process = mock.Mock()
        process.stdin = InputPipe()
        process.stdout = OutputPipe()
        process.stderr = io.StringIO()
        process.wait.return_value = 0
        with mock.patch('colmag.teleoperation_input.subprocess.Popen', return_value=process) as start:
            source = SerialInput('/host/dev/ttyACM0')
            wait_for(lambda: start.called)
            # Publishing the process and entering readline are asynchronous.
            wait_for(lambda: source._process is process)
            source.close()
            source.close()
        self.assertTrue(ended.is_set())
        self.assertEqual(source.error, '')
        self.assertFalse(source._thread.is_alive())
        process.terminate.assert_not_called()

    @unittest.skipUnless(serial, 'pyserial unavailable')
    def test_missing_local_port_is_reported_and_closing_is_idempotent(self):
        source = SerialInput('/dev/magpilot_missing_serial_test')
        try:
            self.assertTrue(wait_for(lambda: source.error))
            self.assertIsNone(source.latest())
        finally:
            source.close()
            source.close()
        self.assertFalse(source._thread.is_alive())

    def test_stream_q_and_eof_stop_and_broken_output_releases_serial_port(self):
        for text in ('q', ''):
            stop = threading.Event()
            watch_stdin(io.StringIO(text), stop)
            self.assertTrue(stop.is_set())
        stop = threading.Event()

        class Connection:
            in_waiting = 218
            closed = False

            def reset_input_buffer(self):
                pass

            def read(self, count):
                return packet()

            def close(self):
                self.closed = True

        class BrokenOutput:
            def write(self, text):
                raise BrokenPipeError('reader closed')

        connection = Connection()
        status = stream_serial('dummy', 115200, stop, BrokenOutput(),
                               serial_factory=lambda **kwargs: connection)
        self.assertEqual(status, 0)
        self.assertTrue(stop.is_set())
        self.assertTrue(connection.closed)


@unittest.skipUnless(pty and serial, 'Local POSIX PTY and pyserial required')
class ActualPtyTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)

    def tearDown(self):
        for descriptor in (self.master, self.slave):
            if descriptor is not None:
                os.close(descriptor)

    def test_local_serial_reads_packets_then_reports_physical_disconnect(self):
        source = SerialInput(self.port)
        try:
            wait_for(lambda: source._serial_port is not None)
            data = packet()
            os.write(self.master, b'garbage' + data[:30])
            time.sleep(0.03)
            self.assertIsNone(source.latest())
            os.write(self.master, data[30:])
            sample = wait_for(source.latest)
            self.assertAlmostEqual(sample['pose'][0], 0.01)
            self.assertEqual(len(sample['magnetic_fields']), 48)
            sample['pose'][0] = 999
            self.assertAlmostEqual(source.latest()['pose'][0], 0.01)
            os.close(self.master)
            self.master = None
            self.assertTrue(wait_for(lambda: source.error))
        finally:
            source.close()

    def test_adapter_clock_sync_with_a_real_bridge_and_pty(self):
        real_start = subprocess.Popen

        def launch_local_bridge(command, **kwargs):
            self.assertEqual(command[:3], ['docker', 'exec', '-i'])
            return real_start([sys.executable, '-u', os.path.join(ROOT, 'tools', 'teleoperation_serial_stream.py'),
                               '--port', self.port], **kwargs)

        with mock.patch('colmag.teleoperation_input.subprocess.Popen', side_effect=launch_local_bridge):
            source = SerialInput('/host/dev/test-pty')
            try:
                def provide_packet():
                    os.write(self.master, packet())
                    return source.latest()

                sample = wait_for(provide_packet)
                self.assertLessEqual(sample['source_age_s'], .25)
                self.assertLessEqual(sample['clock_sync_uncertainty_s'], .125)
                self.assertEqual(len(sample['pose']), 6)
                self.assertEqual(source.error, '')
            finally:
                source.close()
                source.close()
        self.assertFalse(source._thread.is_alive())
        self.assertEqual(source._process.returncode, 0)

    def bridge(self):
        process = subprocess.Popen(
            [sys.executable, '-u', os.path.join(ROOT, 'tools', 'teleoperation_serial_stream.py'),
             '--port', self.port], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True,
        )
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            os.write(self.master, packet())
            if select.select([process.stdout], [], [], 0.02)[0]:
                line = process.stdout.readline()
                if line:
                    self.assertEqual(len(json.loads(line)['pose']), 6)
                    return process
                break
        process.kill()
        process.wait(timeout=2)
        self.fail('Bridge did not read a valid PTY packet: ' + process.stderr.read())

    def test_bridge_stdin_eof_closes_and_exits_cleanly(self):
        process = self.bridge()
        try:
            process.stdin.close()
            self.assertEqual(process.wait(timeout=2), 0)
            self.assertEqual(process.stderr.read(), '')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            process.stdout.close()
            process.stderr.close()

    def test_bridge_sigterm_closes_and_exits_cleanly(self):
        process = self.bridge()
        try:
            process.send_signal(signal.SIGTERM)
            self.assertEqual(process.wait(timeout=2), 0)
            self.assertEqual(process.stderr.read(), '')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            process.stdin.close()
            process.stdout.close()
            process.stderr.close()

    def test_bridge_broken_stdout_closes_and_exits_cleanly(self):
        process = self.bridge()
        try:
            process.stdout.close()
            os.write(self.master, packet())
            self.assertEqual(process.wait(timeout=2), 0)
            self.assertEqual(process.stderr.read(), '')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            process.stdin.close()
            process.stderr.close()


if __name__ == '__main__':
    unittest.main()
