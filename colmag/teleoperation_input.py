"""Threaded sensor-board input for the standalone MuJoCo collection window."""

import copy
import json
import math
import subprocess
import threading
import time

from tools.record_tracking_error import (
    PACKET_HEADER, PACKET_SIZE, PACKET_TAIL, parse_packet,
)


MAX_PACKET_AGE_S = 0.25
MAX_SERIAL_BACKLOG_BYTES = PACKET_SIZE * 16


class PacketFramer:
    """Recover complete finite board packets from arbitrary serial read chunks."""

    def __init__(self):
        self.buffer = bytearray()
        self.last_chunk_monotonic_s = None

    def feed(self, chunk, received_monotonic_s=None):
        if chunk and received_monotonic_s is not None:
            received = float(received_monotonic_s)
            if not math.isfinite(received):
                raise ValueError('Chunk receipt time must be finite.')
            if (self.last_chunk_monotonic_s is not None and
                    (received - self.last_chunk_monotonic_s > MAX_PACKET_AGE_S or
                     received < self.last_chunk_monotonic_s)):
                self.buffer.clear()
            self.last_chunk_monotonic_s = received
        self.buffer.extend(chunk)
        packets = []
        while self.buffer:
            start = self.buffer.find(bytes((PACKET_HEADER,)))
            if start < 0:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < PACKET_SIZE:
                break
            if self.buffer[PACKET_SIZE - 1] != PACKET_TAIL:
                del self.buffer[0]
                continue
            parsed = parse_packet(self.buffer[:PACKET_SIZE])
            del self.buffer[:PACKET_SIZE]
            if parsed is None:
                continue
            fields, pose = parsed
            if not all(math.isfinite(value) for value in fields + pose):
                continue
            packets.append(dict(pose=list(pose), magnetic_fields=list(fields)))
        return packets


def finite_packet(message):
    """Validate a bridge message before it can influence robot commands."""
    if not isinstance(message, dict):
        return None
    try:
        pose = [float(value) for value in message['pose']]
        fields = [float(value) for value in message['magnetic_fields']]
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if len(pose) != 6 or len(fields) != 48:
        return None
    if not all(math.isfinite(value) for value in pose + fields):
        return None
    return dict(pose=pose, magnetic_fields=fields)


class SerialInput:
    """Nonblocking latest-packet adapter; local ports or Docker-mapped ports.

    A Docker timestamp is deliberately replaced by host receipt time, so the
    GUI's freshness check uses the same monotonic clock as its trial recorder.
    """

    def __init__(self, port, baudrate=921600, container='colmag_simon'):
        if not isinstance(port, str) or not port.strip():
            raise ValueError('A serial port is required.')
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError('baudrate must be a positive integer')
        self.port = port
        self.baudrate = baudrate
        self.container = container
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None
        self._error = ''
        self._serial_port = None
        self._process = None
        self._closed = False
        self._thread = threading.Thread(target=self._worker, name='magpilot-serial-input',
                                        daemon=True)
        self._thread.start()

    @property
    def error(self):
        with self._lock:
            return self._error

    def latest(self):
        with self._lock:
            return copy.deepcopy(self._latest)

    def _publish(self, packet, received_monotonic_s=None):
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        packet = dict(packet, received_monotonic_s=received)
        with self._lock:
            if not self._stop.is_set():
                self._latest = packet

    def _worker(self):
        try:
            if self.port.startswith('/host/dev/'):
                self._docker_worker()
            else:
                self._host_worker()
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = '{}'.format(exc) or type(exc).__name__

    def _host_worker(self):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError('Install pyserial in the teleoperation environment.') from exc
        connection = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.05)
        framer = PacketFramer()
        try:
            connection.reset_input_buffer()
            with self._lock:
                self._serial_port = connection
            while not self._stop.is_set():
                waiting = getattr(connection, 'in_waiting', 0)
                if waiting > MAX_SERIAL_BACKLOG_BYTES:
                    connection.reset_input_buffer()
                    framer.buffer.clear()
                    continue
                chunk = connection.read(waiting if waiting else PACKET_SIZE)
                received = time.monotonic()
                packets = framer.feed(chunk, received)
                if packets:
                    self._publish(packets[-1], received)
        finally:
            connection.close()

    def _docker_worker(self):
        command = ['docker', 'exec', '-i', self.container, 'python3', '-u',
                   '/colmag/tools/teleoperation_serial_stream.py',
                   '--port', self.port, '--baudrate', str(self.baudrate)]
        process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1)
        with self._lock:
            self._process = process
        sync_nonce = 0
        sync_sent = 0.0
        sync_warmed = False
        clock_offset = source_anchor = uncertainty = None

        def request_sync():
            nonlocal sync_nonce, sync_sent
            sync_nonce += 1
            sync_sent = time.monotonic()
            process.stdin.write(json.dumps(dict(type='clock_sync', nonce=sync_nonce)) + '\n')
            process.stdin.flush()

        try:
            request_sync()
            while not self._stop.is_set():
                line = process.stdout.readline()
                received = time.monotonic()
                if not line:
                    break
                try:
                    message = json.loads(line)
                    if not isinstance(message, dict):
                        continue
                    source_time = float(message['source_monotonic_s'])
                    if not math.isfinite(source_time):
                        continue
                    if message.get('type') == 'clock_sync':
                        if message.get('nonce') != sync_nonce:
                            continue
                        # Warm up Docker first, then bound the clock offset with
                        # a short round trip. Clock epochs need not match.
                        round_trip = received - sync_sent
                        if not sync_warmed or round_trip > MAX_PACKET_AGE_S / 2:
                            sync_warmed = True
                            request_sync()
                            continue
                        clock_offset = sync_sent - source_time
                        source_anchor = source_time
                        uncertainty = round_trip
                        continue
                    if clock_offset is None or source_time < source_anchor:
                        continue
                    age = received - (source_time + clock_offset)
                    # The conservative offset overestimates age by at most
                    # the bounded round-trip time. Old queued lines cannot be
                    # revived by assigning them a new host receipt timestamp.
                    if age < -1e-6 or age > MAX_PACKET_AGE_S:
                        continue
                    packet = finite_packet(message)
                    if packet is not None:
                        packet.update(source_monotonic_s=source_time, source_age_s=max(0, age),
                                      clock_sync_uncertainty_s=uncertainty)
                        self._publish(packet, received)
                except (KeyError, ValueError, TypeError, OverflowError):
                    continue
            if not self._stop.is_set():
                process.wait(timeout=2)
                detail = process.stderr.read().strip()
                raise RuntimeError(detail or 'Sensor-board stream disconnected (status {}).'.format(
                    process.returncode))
        finally:
            self._stop_process(process)

    @staticmethod
    def _stop_process(process):
        try:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()  # EOF asks the board bridge to release its port.
        except (OSError, ValueError):
            pass
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=0.5)
        finally:
            for stream in (process.stdout, process.stderr):
                if stream is not None:
                    stream.close()

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop.set()
            connection = self._serial_port
            process = self._process
        if connection is not None:
            try:
                connection.close()
            except (OSError, ValueError):
                pass
        if process is not None:
            try:
                if process.stdin is not None and not process.stdin.closed:
                    process.stdin.close()
            except (OSError, ValueError):
                pass
        self._thread.join(timeout=2)
        if self._thread.is_alive() and process is not None:
            # The stream worker owns pipe cleanup; terminate only to unblock it.
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            self._thread.join(timeout=1)
