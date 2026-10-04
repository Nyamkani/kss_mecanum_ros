"""Single-client NDJSON transport with ROS work confined to the executor."""

import json
import math
import queue
import select
import socket
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def encode_message(message):
    return (json.dumps(message, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')


def reject_constant(value):
    raise ValueError('Non-finite JSON number: ' + value)


class TcpServer:
    """Own sockets in one thread; exchange commands/results using bounded queues."""

    MAX_LINE_BYTES = 16384
    MAX_PENDING_BYTES = 262144

    def __init__(self, bind_address, port):
        self.commands = queue.Queue(maxsize=128)
        self.results = queue.Queue(maxsize=128)
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self._state = b''
        self._state_version = 0
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._listener.bind((bind_address, port))
            self._listener.listen(4)
            self._listener.setblocking(False)
        except Exception:
            self._listener.close()
            raise
        self._thread = threading.Thread(target=self._run, name='gateway-tcp', daemon=True)

    def start(self):
        self._thread.start()

    def publish_state(self, message):
        # Keep only the latest telemetry between network iterations or connections.
        data = encode_message(message)
        with self._state_lock:
            self._state = data
            self._state_version += 1

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join()  # select is bounded to 50 ms; all socket I/O is nonblocking.
        else:
            self._listener.close()

    def _run(self):
        client = None
        session = 0
        incoming = bytearray()
        outgoing = bytearray()
        sent_version = -1
        try:
            while not self._stop.is_set():
                readers = [self._listener] + ([client] if client else [])
                writers = [client] if client and outgoing else []
                readable, writable, _ = select.select(readers, writers, [], 0.05)
                if self._listener in readable:
                    try:
                        candidate, _ = self._listener.accept()
                    except (BlockingIOError, ConnectionAbortedError):
                        candidate = None
                    if candidate is not None:
                        if client is not None:
                            candidate.close()
                        else:
                            candidate.setblocking(False)
                            client = candidate
                            session += 1
                            incoming.clear()
                            outgoing.clear()
                            sent_version = -1

                try:
                    if client is not None and client in readable:
                        chunk = client.recv(4096)
                        if not chunk:
                            raise ConnectionError('client disconnected')
                        incoming.extend(chunk)
                        while b'\n' in incoming:
                            line, _, remainder = incoming.partition(b'\n')
                            incoming = bytearray(remainder)
                            if len(line) > self.MAX_LINE_BYTES:
                                raise ConnectionError('command too large')
                            try:
                                command = json.loads(line.decode('utf-8'), parse_constant=reject_constant)
                                error = None
                            except (UnicodeDecodeError, ValueError, RecursionError):
                                command, error = None, 'invalid JSON'
                            self.commands.put_nowait((session, command, error))
                        if len(incoming) > self.MAX_LINE_BYTES:
                            raise ConnectionError('command too large')

                    # Discard responses from previous connections, including when idle.
                    for _ in range(128):
                        try:
                            result_session, result = self.results.get_nowait()
                        except queue.Empty:
                            break
                        if client is not None and result_session == session:
                            outgoing.extend(result)
                    if client is not None:
                        with self._state_lock:
                            if sent_version != self._state_version:
                                outgoing.extend(self._state)
                                sent_version = self._state_version
                        if len(outgoing) > self.MAX_PENDING_BYTES:
                            raise ConnectionError('client is not consuming data')
                        if client in writable:
                            try:
                                count = client.send(outgoing)
                            except BlockingIOError:
                                count = 0
                            del outgoing[:count]
                except (OSError, queue.Full):
                    if client is not None:
                        client.close()
                        client = None
                    incoming.clear()
                    outgoing.clear()
        finally:
            if client is not None:
                client.close()
            self._listener.close()


class GatewayNode(Node):
    MODES = ('BASE', 'MAPPING', 'NAVIGATION')

    def __init__(self):
        super().__init__('gateway_node')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('linear_motion_threshold', 0.01)
        self.declare_parameter('angular_motion_threshold', 0.01)
        bind_address = self.get_parameter('bind_address').value
        port = self.get_parameter('port').value
        rate = self.get_parameter('telemetry_rate').value
        self._odom_timeout = self.get_parameter('odom_timeout').value
        self._linear_threshold = self.get_parameter('linear_motion_threshold').value
        self._angular_threshold = self.get_parameter('angular_motion_threshold').value
        if not 1 <= port <= 65535:
            raise ValueError('port must be between 1 and 65535')
        for name, value in [('telemetry_rate', rate), ('odom_timeout', self._odom_timeout)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        for value in (self._linear_threshold, self._angular_threshold):
            if not math.isfinite(value) or value < 0:
                raise ValueError('motion thresholds must be finite and nonnegative')

        self.mode = 'BASE'
        self._last_odom_time = None
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._velocity = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self._odom_subscription = self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odometry, 10
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._tick, clock=self._steady_clock)
        self._server = TcpServer(bind_address, port)
        self._tick()
        self._server.start()
        self.get_logger().info(f'TCP gateway listening on {bind_address}:{port}')

    def _on_odometry(self, message):
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        twist = message.twist.twist
        values = (position.x, position.y, q.x, q.y, q.z, q.w,
                  twist.linear.x, twist.linear.y, twist.angular.z)
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-12 or not math.isfinite(norm):
            return
        x, y, z, w = (q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self._pose = {'x': position.x, 'y': position.y, 'yaw': yaw}
        self._velocity = {'vx': twist.linear.x, 'vy': twist.linear.y, 'wz': twist.angular.z}
        self._last_odom_time = time.monotonic()

    def _tick(self):
        for _ in range(128):
            if self._server.results.full():
                break
            try:
                session, command, error = self._server.commands.get_nowait()
            except queue.Empty:
                break
            result = self._command_result(command, error)
            try:
                self._server.results.put_nowait((session, encode_message(result)))
            except queue.Full:
                break
        ready = (self._last_odom_time is not None
                 and time.monotonic() - self._last_odom_time <= self._odom_timeout)
        motion = 'UNKNOWN'
        if ready:
            moving = (math.hypot(self._velocity['vx'], self._velocity['vy']) > self._linear_threshold
                      or abs(self._velocity['wz']) > self._angular_threshold)
            motion = 'MOVING' if moving else 'STOPPED'
        error = '' if ready else ('odometry unavailable' if self._last_odom_time is None else 'odometry stale')
        self._server.publish_state({
            'type': 'robot_state',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'mode': self.mode,
            'base_ready': ready,
            'pose': self._pose,
            'velocity': self._velocity,
            'motion_state': motion,
            'error': error,
        })

    @staticmethod
    def _command_result(payload, error=None):
        command, request_id = None, None
        if error is None:
            if not isinstance(payload, dict):
                error = 'command must be a JSON object'
            else:
                command = payload.get('command')
                request_id = payload.get('request_id')
                if not (request_id is None or type(request_id) in (int, str)):
                    request_id = None
                    error = 'request_id must be an integer, string, or null'
                if not isinstance(command, str):
                    command = None
                    error = 'command must be a string'
        success = error is None and command == 'get_status'
        return {
            'type': 'command_result',
            'request_id': request_id,
            'command': command,
            'success': success,
            'message': error or ('ok' if success else 'unknown command'),
        }

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GatewayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
