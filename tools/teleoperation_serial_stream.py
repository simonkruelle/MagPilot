#!/usr/bin/env python3
"""Send finite board packets as newline JSON; stop on stdin EOF/q or SIGTERM."""

import argparse
from contextlib import nullcontext
import json
import os
import signal
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from colmag.teleoperation_input import MAX_SERIAL_BACKLOG_BYTES, PacketFramer  # noqa: E402
from tools.record_tracking_error import PACKET_SIZE  # noqa: E402


def watch_stdin(stream, stop, on_message=None):
    pending = ''
    try:
        while not stop.is_set():
            char = stream.read(1)
            if not char or (char.lower() == 'q' and not pending):
                stop.set()
                return
            if char == '\n':
                if on_message is not None:
                    on_message(pending)
                pending = ''
            elif len(pending) < 4096:
                pending += char
            else:
                pending = ''
    except (OSError, ValueError):
        stop.set()


def write_message(message, stop, output):
    try:
        output.write(json.dumps(message, allow_nan=False) + '\n')
        output.flush()
        return True
    except BrokenPipeError:
        stop.set()
        # Prevent Python's final flush from reporting another broken pipe.
        try:
            replacement = os.open(os.devnull, os.O_WRONLY)
            try:
                os.dup2(replacement, output.fileno())
            finally:
                os.close(replacement)
        except (AttributeError, OSError, ValueError):
            pass
        return False


def stream_serial(port, baudrate, stop, output, serial_factory=None, output_lock=None):
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial
    connection = serial_factory(port=port, baudrate=baudrate, timeout=0.05)
    framer = PacketFramer()
    try:
        connection.reset_input_buffer()
        while not stop.is_set():
            waiting = getattr(connection, 'in_waiting', 0)
            if waiting > MAX_SERIAL_BACKLOG_BYTES:
                connection.reset_input_buffer()
                framer.buffer.clear()
                continue
            chunk = connection.read(waiting if waiting else PACKET_SIZE)
            received = time.monotonic()
            packets = framer.feed(chunk, received)
            if packets and not stop.is_set():
                message = dict(packets[-1], source_monotonic_s=received)
                with output_lock if output_lock is not None else nullcontext():
                    if not write_message(message, stop, output):
                        return 0
    finally:
        connection.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baudrate', type=int, default=921600)
    args = parser.parse_args(argv)
    if args.baudrate <= 0:
        parser.error('--baudrate must be positive')
    stop = threading.Event()
    output_lock = threading.Lock()

    def control_message(text):
        try:
            message = json.loads(text)
            if not isinstance(message, dict) or message.get('type') != 'clock_sync':
                return
            with output_lock:
                write_message(dict(type='clock_sync', nonce=message.get('nonce'),
                                   source_monotonic_s=time.monotonic()), stop, sys.stdout)
        except (ValueError, TypeError):
            pass

    for name in ('SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, name), lambda signum, frame: stop.set())
    watcher = threading.Thread(target=watch_stdin, args=(sys.stdin, stop, control_message), daemon=True)
    watcher.start()
    try:
        return stream_serial(args.port, args.baudrate, stop, sys.stdout, output_lock=output_lock)
    except Exception as exc:
        if stop.is_set():
            return 0
        print('Sensor-board stream: {}'.format(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
