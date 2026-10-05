"""Hardware-free integration test; run after sourcing gateway/bringup install.

Run: ROS_DOMAIN_ID=<isolated domain> python3 mecanum_gateway/test/test_map_pose_tf.py
Starts only the mapping launch; synthetic odometry and TF replace the base system.
"""

import json
import math
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster


class MapPoseTfTest(unittest.TestCase):
    def test_future_and_stale_tf(self):
        rclpy.init()
        node = rclpy.create_node('gateway_map_pose_test')
        publisher = node.create_publisher(Odometry, '/odometry/filtered', 10)
        broadcaster = TransformBroadcaster(node)
        # Reserve an available test port, then release it for Gateway.
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        client = None
        buffer = b''
        offset = None
        send_odom = True
        next_publish = 0.0
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'mecanum_gateway', 'gateway_node', '--ros-args',
                '-p', 'bind_address:=127.0.0.1', '-p', f'port:={port}',
                '-p', 'odom_timeout:=0.25',
            ], stdout=log, stderr=log, start_new_session=True)

            def receive(predicate, timeout=8.0):
                nonlocal buffer, next_publish
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.005)
                    if time.monotonic() >= next_publish:
                        next_publish = time.monotonic() + 0.04
                        now = node.get_clock().now()
                        if send_odom:
                            odom = Odometry()
                            odom.header.frame_id = 'odom'
                            odom.header.stamp = now.to_msg()
                            odom.pose.pose.position.x = 1.0
                            odom.pose.pose.position.y = 2.0
                            odom.pose.pose.orientation.w = 1.0
                            publisher.publish(odom)
                        if offset is not None:
                            transform = TransformStamped()
                            transform.header.frame_id = 'map'
                            transform.child_frame_id = 'odom'
                            transform.header.stamp = (now + Duration(seconds=offset)).to_msg()
                            transform.transform.translation.x = 2.0
                            transform.transform.translation.y = 3.0
                            transform.transform.rotation.z = math.sin(0.25)
                            transform.transform.rotation.w = math.cos(0.25)
                            broadcaster.sendTransform(transform)
                    if b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        packet = json.loads(line)
                        if predicate(packet):
                            return packet
                        continue
                    try:
                        chunk = client.recv(65536)
                    except socket.timeout:
                        continue
                    self.assertTrue(chunk, 'Gateway disconnected')
                    buffer += chunk
                self.fail('Timed out waiting for expected telemetry/result')

            def state(predicate):
                return receive(lambda p: p['type'] == 'robot_state' and predicate(p))

            try:
                for _ in range(150):
                    self.assertIsNone(process.poll(), 'Gateway exited during startup')
                    try:
                        client = socket.create_connection(('127.0.0.1', port), timeout=0.02)
                        break
                    except OSError:
                        time.sleep(0.04)
                self.assertIsNotNone(client)
                client.sendall(b'{"command":"start_mapping","request_id":1}\n')
                result = receive(lambda p: p['type'] == 'command_result')
                self.assertTrue(result['success'], result)
                state(lambda p: p['mode'] == 'MAPPING' and p['base_ready'] and p['map_pose'] is None)

                # Fresh odometry with an already stale dynamic transform stays hidden.
                offset = -3.0
                start = node.get_clock().now().nanoseconds / 1e9
                state(lambda p: p['timestamp'] > start + 0.5 and p['base_ready'] and p['map_pose'] is None)

                # AMCL-like +1 s stamp must work independently of odom_timeout=0.25.
                offset = 1.0
                packet = state(lambda p: p['map_pose'] is not None)
                self.assertEqual(packet['pose'], dict(x=1.0, y=2.0, yaw=0.0))
                self.assertAlmostEqual(packet['map_pose']['x'], 2 + math.cos(0.5) - 2 * math.sin(0.5))
                self.assertAlmostEqual(packet['map_pose']['y'], 3 + math.sin(0.5) + 2 * math.cos(0.5))
                self.assertAlmostEqual(packet['map_pose']['yaw'], 0.5)

                # Stop TF, keep odometry fresh: last future TF eventually becomes stale.
                offset = None
                state(lambda p: p['base_ready'] and p['map_pose'] is None)

                # A future stamp outside the configured bound must also be rejected.
                offset = 4.0
                start = node.get_clock().now().nanoseconds / 1e9
                state(lambda p: p['timestamp'] > start + 0.5 and p['base_ready'] and p['map_pose'] is None)
                send_odom = False
                state(lambda p: not p['base_ready'] and p['map_pose'] is None)
            finally:
                if client is not None:
                    client.close()
                if process.poll() is None:
                    import os
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()
                if process.returncode:
                    log.seek(0)
                    print(log.read())


if __name__ == '__main__':
    unittest.main()
