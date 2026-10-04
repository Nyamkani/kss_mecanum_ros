"""ROS-independent, single-worker TCP/NDJSON client (Python standard library)."""

import errno
import ipaddress
import json
import math
import queue
import select
import socket
import threading
import time
import zlib

from map_view import decode_map

DEFAULT_PORT = 8765
ROBOT_STATE_TIMEOUT = 1.5  # Monotonic seconds; socket open alone is not connected.
CONNECT_TIMEOUT = 3.0
IO_INTERVAL = 0.05
MAX_FRAME_BYTES = 2 * 1024 * 1024


def _reject_constant(value):
    raise ValueError('Non-finite JSON number: ' + value)


def valid_robot_state(packet):
    """Only complete, finite state packets refresh connection health."""
    if packet.get('mode') not in ('BASE', 'MAPPING', 'NAVIGATION'):
        return False
    if type(packet.get('base_ready')) is not bool:
        return False
    if packet.get('motion_state') not in ('UNKNOWN', 'STOPPED', 'MOVING'):
        return False
    if not isinstance(packet.get('error'), str):
        return False
    for section, keys in (('pose', ('x', 'y', 'yaw')), ('velocity', ('vx', 'vy', 'wz'))):
        values = packet.get(section)
        if not isinstance(values, dict):
            return False
        for key in keys:
            value = values.get(key)
            if type(value) not in (int, float):
                return False
            try:
                if not math.isfinite(value):
                    return False
            except OverflowError:
                return False
    return True


class RobotClient:
    def __init__(self):
        self.events = queue.Queue(maxsize=512)
        self._outbound = queue.Queue(maxsize=128)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._socket = None
        self._connecting = False
        self._last_robot_state_rx = None
        self._request_id = 0
        self._manual_outbound = None
        self._manual_sent = threading.Event()
        self._manual_sent.set()

    @property
    def last_robot_state_rx(self):
        with self._lock:
            return self._last_robot_state_rx

    @property
    def connecting(self):
        with self._lock:
            return self._connecting

    @property
    def active(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def is_connected(self):
        with self._lock:
            return (self._socket is not None and self._last_robot_state_rx is not None
                    and time.monotonic() - self._last_robot_state_rx < ROBOT_STATE_TIMEOUT)

    def connect(self, host, port=DEFAULT_PORT):
        host = host.strip()
        if host.lower() == 'localhost':
            host = '127.0.0.1'
        try:
            address = ipaddress.ip_address(host)
        except ValueError as exc:
            raise ValueError('Enter a numeric Robot IP address (or localhost).') from exc
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('Port must be an integer from 1 to 65535.')
        self.disconnect()
        for channel in (self.events, self._outbound):
            while True:
                try:
                    channel.get_nowait()
                except queue.Empty:
                    break
        with self._lock:
            self._last_robot_state_rx = None
            self._manual_outbound = None
            self._manual_sent.set()
            self._connecting = True
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._worker, args=(str(address), port, address.version),
            name='robot-client', daemon=False,
        )
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        with self._lock:
            sock = self._socket
            self._last_robot_state_rx = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join()  # All I/O is nonblocking; select is bounded to 50 ms.
            self._thread = None

    def send_command(self, command, **fields):
        if not self.is_connected:
            raise ConnectionError('No recent robot_state; reconnect or wait for telemetry.')
        if 'command' in fields or 'request_id' in fields:
            raise ValueError('command and request_id are managed by RobotClient.')
        self._request_id += 1
        packet = dict(fields, command=command, request_id=self._request_id)
        data = (json.dumps(packet, allow_nan=False, ensure_ascii=True) + '\n').encode('utf-8')
        if len(data) - 1 > 16384:
            raise ValueError('Command exceeds the Gateway frame limit.')
        if command == 'manual_velocity':
            # Coalesce unsent velocities so a slow link cannot replay old drive commands.
            with self._lock:
                self._manual_outbound = data
                self._manual_sent.clear()
            return self._request_id
        try:
            self._outbound.put_nowait(data)
        except queue.Full as exc:
            raise RuntimeError('Too many queued commands.') from exc
        return self._request_id

    def flush_manual(self, timeout=0.15):
        return self._manual_sent.wait(timeout)

    def _event(self, kind, value):
        self.events.put_nowait((kind, value))

    def _packet(self, line):
        try:
            packet = json.loads(line.decode('utf-8'), parse_constant=_reject_constant)
            if not isinstance(packet, dict):
                raise ValueError('packet must be a JSON object')
            if packet.get('type') == 'robot_state':
                if not valid_robot_state(packet):
                    raise ValueError('invalid robot_state fields')
                with self._lock:
                    self._last_robot_state_rx = time.monotonic()
            elif packet.get('type') == 'map':
                packet = decode_map(packet)
            elif packet.get('type') == 'command_result':
                if (type(packet.get('request_id')) not in (int, str, type(None))
                        or type(packet.get('success')) is not bool
                        or not isinstance(packet.get('message'), str)):
                    raise ValueError('invalid command_result fields')
            else:
                raise ValueError('unknown packet type')
        except (ValueError, UnicodeDecodeError, RecursionError, KeyError, TypeError, OverflowError, zlib.error) as exc:
            self._event('error', 'Ignored invalid packet: ' + str(exc))
            return
        self._event('packet', packet)

    def _worker(self, host, port, version):
        sock = None
        reason = 'Disconnected'
        try:
            sock = socket.socket(socket.AF_INET6 if version == 6 else socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            with self._lock:
                self._socket = sock
            status = sock.connect_ex((host, port))
            pending = {0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY,
                       getattr(errno, 'WSAEWOULDBLOCK', 10035), getattr(errno, 'WSAEINPROGRESS', 10036)}
            if status not in pending:
                raise OSError(status, 'Connection failed')
            deadline = time.monotonic() + CONNECT_TIMEOUT
            while status != 0 and not self._stop.is_set():
                if time.monotonic() >= deadline:
                    raise TimeoutError('Connection timed out')
                _, writable, exceptional = select.select([], [sock], [sock], IO_INTERVAL)
                if writable or exceptional:
                    status = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                    if status:
                        raise OSError(status, 'Connection failed')
            if self._stop.is_set():
                return
            with self._lock:
                self._connecting = False
            self._event('info', 'TCP connected; waiting for robot_state')
            incoming, outgoing = bytearray(), bytearray()
            outgoing_is_manual = False
            while not self._stop.is_set():
                if not outgoing:
                    with self._lock:
                        manual = self._manual_outbound
                        self._manual_outbound = None
                    outgoing_is_manual = manual is not None
                    if manual is not None:
                        outgoing.extend(manual)
                    else:
                        try:
                            outgoing.extend(self._outbound.get_nowait())
                        except queue.Empty:
                            pass
                readable, writable, _ = select.select([sock], [sock] if outgoing else [], [], IO_INTERVAL)
                if readable:
                    try:
                        chunk = sock.recv(65536)
                    except BlockingIOError:
                        continue
                    if not chunk:
                        reason = 'Gateway closed the connection'
                        break
                    incoming.extend(chunk)
                    while b'\n' in incoming:
                        line, _, remainder = incoming.partition(b'\n')
                        incoming = bytearray(remainder)
                        if len(line) > MAX_FRAME_BYTES:
                            raise ValueError('Incoming packet too large')
                        self._packet(line)
                    if len(incoming) > MAX_FRAME_BYTES:
                        raise ValueError('Incoming packet too large')
                if writable:
                    try:
                        count = sock.send(outgoing)
                    except BlockingIOError:
                        continue
                    if count == 0:
                        raise ConnectionError('Socket send failed')
                    del outgoing[:count]
                    if not outgoing and outgoing_is_manual:
                        with self._lock:
                            if self._manual_outbound is None:
                                self._manual_sent.set()
        except (OSError, ValueError, queue.Full) as exc:
            reason = 'Disconnected: ' + (str(exc) or 'receive queue full')
        finally:
            if sock is not None:
                sock.close()
            with self._lock:
                self._socket = None
                self._connecting = False
                self._last_robot_state_rx = None
            try:
                self._event('disconnected', reason)
            except queue.Full:
                pass
