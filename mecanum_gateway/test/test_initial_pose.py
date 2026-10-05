"""Hardware-free Gateway TCP -> initial pose subscriber integration test.
Run after sourcing the workspace in an isolated ROS_DOMAIN_ID.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseTest(unittest.TestCase):
    def test_publish(self):
        for override in (False, True):
            with self.subTest(override=override):
                self.check_publish(override)

    def check_publish(self, override):
        rclpy.init()
        node = rclpy.create_node('initial_pose_test')
        messages = []
        subscription = node.create_subscription(PoseWithCovarianceStamped, '/initialpose', messages.append, 10)
        client = None
        buffer = b''
        request_id = 0
        with tempfile.TemporaryDirectory(prefix='initial-pose-test-') as directory:
            root = Path(directory)
            launcher = root / 'ros2'
            launcher.write_text('#!/bin/sh\nexec sleep 300\n')
            launcher.chmod(0o755)
            (root / 'room.yaml').write_text('image: room.pgm\n')
            with socket.socket() as reservation:
                reservation.bind(('127.0.0.1', 0))
                port = reservation.getsockname()[1]
            exe = Path(get_package_prefix('mecanum_gateway')) / 'lib/mecanum_gateway/gateway_node'
            args = [str(exe), '--ros-args', '-p', 'bind_address:=127.0.0.1', '-p', f'port:={port}', '-p', f'map_dir:={root}']
            xy, yaw_stddev = (0.4, 0.5) if override else (0.25, 0.261799)
            if override:
                args += ['-p', f'initial_pose_xy_stddev:={xy}', '-p', f'initial_pose_yaw_stddev:={yaw_stddev}']
            with (root / 'runtime.log').open('w+') as log:
                process = subprocess.Popen(args, env=dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH']), stdout=log, stderr=log)

                def receive(predicate):
                    nonlocal buffer
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.005)
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
                        self.assertTrue(chunk)
                        buffer += chunk
                    self.fail('response timeout')

                def command(name, **fields):
                    nonlocal request_id
                    request_id += 1
                    client.sendall((json.dumps(dict(command=name, request_id=request_id, **fields))+'\n').encode())
                    # Invalid JSON constants do not retain a request_id.
                    return receive(lambda p: p['type'] == 'command_result')

                def initial(**fields):
                    return command('set_initial_pose', **fields)

                try:
                    for _ in range(150):
                        self.assertIsNone(process.poll())
                        try:
                            client = socket.create_connection(('127.0.0.1', port), timeout=0.02)
                            break
                        except OSError:
                            time.sleep(0.04)
                    self.assertIsNotNone(client)
                    receive(lambda p: p['type'] == 'robot_state')
                    self.assertEqual(initial(x=1, y=2, yaw=0)['message'], 'initial pose is only allowed in NAVIGATION mode')
                    self.assertTrue(command('start_mapping')['success'])
                    self.assertFalse(initial(x=1, y=2, yaw=0)['success'])
                    self.assertTrue(command('stop_mode')['success'])
                    self.assertTrue(command('start_navigation', map_name='room')['success'])
                    deadline = time.monotonic() + 5
                    while subscription.get_publisher_count() == 0 and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertGreater(subscription.get_publisher_count(), 0)
                    for field in ('x', 'y', 'yaw'):
                        for bad in (None, True, '1.0', float('nan'), float('inf')):
                            fields = dict(x=1.2, y=-0.4, yaw=1.57)
                            fields[field] = bad
                            self.assertFalse(initial(**fields)['success'])
                    self.assertFalse(initial(x=1, y=2)['success'])
                    self.assertEqual(messages, [])
                    for yaw in (1.57, 7.0, -7.0):
                        before = len(messages)
                        start = node.get_clock().now().nanoseconds / 1e9
                        result = initial(x=1.2, y=-0.4, yaw=yaw)
                        self.assertTrue(result['success'])
                        self.assertEqual(result['message'], 'initial pose published')
                        deadline = time.monotonic() + 2
                        while len(messages) == before and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.05)
                        self.assertEqual(len(messages), before+1)
                        message = messages[-1]
                        self.assertEqual(message.header.frame_id, 'map')
                        self.assertGreaterEqual(message.header.stamp.sec + message.header.stamp.nanosec/1e9, start)
                        pose = message.pose.pose
                        self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (1.2, -0.4, 0.0))
                        angle = math.remainder(yaw, 2*math.pi)
                        self.assertAlmostEqual(pose.orientation.z, math.sin(angle/2))
                        self.assertAlmostEqual(pose.orientation.w, math.cos(angle/2))
                        self.assertEqual((pose.orientation.x, pose.orientation.y), (0.0, 0.0))
                        expected = [0.0]*36
                        expected[0] = expected[7] = xy*xy
                        expected[35] = yaw_stddev*yaw_stddev
                        self.assertEqual(list(message.pose.covariance), expected)
                finally:
                    if client:
                        client.close()
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        process.wait(timeout=10)
                    if process.returncode:
                        log.seek(0)
                        print(log.read())
                    node.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
