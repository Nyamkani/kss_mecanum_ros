"""No hardware: fake launch process + real ROS action server + Gateway TCP.
Run in an isolated ROS_DOMAIN_ID after sourcing the built workspace.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class NavigationTest(unittest.TestCase):
    def test_lifecycle(self):
        rclpy.init()
        node = rclpy.create_node('mock_navigation')
        outcome = threading.Event()
        finish = 'success'
        received = []
        cancels = []

        def accept(request):
            received.append(request)
            if request.pose.pose.position.x == 8.0:
                time.sleep(0.6)
            return GoalResponse.REJECT if request.pose.pose.position.x == -9.0 else GoalResponse.ACCEPT

        def execute(handle):
            while rclpy.ok() and not outcome.wait(0.02):
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
            if handle.is_cancel_requested:
                handle.canceled()
            elif finish == 'abort':
                handle.abort()
            else:
                handle.succeed()
            return NavigateToPose.Result()

        def cancel(handle):
            cancels.append(handle)
            return CancelResponse.ACCEPT

        server = ActionServer(node, NavigateToPose, '/navigate_to_pose', execute,
                              goal_callback=accept, cancel_callback=cancel,
                              callback_group=ReentrantCallbackGroup())
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin)
        worker.start()
        client = None
        process = None
        buf = b''
        rid = 0
        with tempfile.TemporaryDirectory(prefix='navigation-test-') as directory:
            root = Path(directory)
            # ModeManager still creates/terminates process groups; no actual Nav2 is launched.
            launcher = root / 'ros2'
            launcher.write_text('#!/bin/sh\nexec sleep 300\n')
            launcher.chmod(0o755)
            (root / 'room.yaml').write_text('image: room.pgm\n')
            with socket.socket() as reservation:
                reservation.bind(('127.0.0.1', 0))
                port = reservation.getsockname()[1]
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'])
            log = (root / 'gateway.log').open('w+')
            exe = Path(get_package_prefix('mecanum_gateway')) / 'lib/mecanum_gateway/gateway_node'
            process = subprocess.Popen([str(exe), '--ros-args', '-p', f'port:={port}',
                                        '-p', 'bind_address:=127.0.0.1', '-p', f'map_dir:={root}'],
                                       env=env, stdout=log, stderr=log)

            def receive(predicate, timeout=8):
                nonlocal buf
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        packet = json.loads(line)
                        if predicate(packet):
                            return packet
                        continue
                    try:
                        chunk = client.recv(65536)
                    except socket.timeout:
                        continue
                    self.assertTrue(chunk)
                    buf += chunk
                self.fail('response timeout')

            def send(command, **fields):
                nonlocal rid
                rid += 1
                client.sendall((json.dumps(dict(command=command, request_id=rid, **fields)) + '\n').encode())
                return rid

            def reply(request):
                return receive(lambda p: p['type'] == 'command_result' and p['request_id'] == request)

            def command(name, **fields):
                return reply(send(name, **fields))

            def state(value):
                return receive(lambda p: p['type'] == 'robot_state' and p['navigation_state'] == value)

            def goal(x=1.25, **fields):
                return command('navigation_goal', x=x, y=-0.8, yaw=7.0, **fields)

            try:
                for _ in range(150):
                    self.assertIsNone(process.poll())
                    try:
                        client = socket.create_connection(('127.0.0.1', port), timeout=0.1)
                        break
                    except OSError:
                        time.sleep(0.05)
                self.assertIsNotNone(client)
                state('IDLE')
                self.assertFalse(goal()['success'])
                self.assertTrue(command('start_mapping')['success'])
                self.assertFalse(goal()['success'])
                self.assertTrue(command('stop_mode')['success'])
                self.assertTrue(command('start_navigation', map_name='room')['success'])
                time.sleep(0.5)
                self.assertFalse(command('cancel_navigation_goal')['success'])
                for bad in (None, True, '1'):
                    self.assertFalse(command('navigation_goal', x=bad, y=0, yaw=0)['success'])
                for bad in (float('nan'), float('inf')):
                    send('navigation_goal', x=bad, y=0, yaw=0)
                    self.assertFalse(receive(lambda p: p['type'] == 'command_result')['success'])
                self.assertTrue(goal()['success'])
                state('EXECUTING')
                pose = received[-1].pose
                self.assertEqual(pose.header.frame_id, 'map')
                self.assertGreater(pose.header.stamp.sec, 0)
                self.assertEqual(pose.pose.position.y, -0.8)
                self.assertAlmostEqual(pose.pose.orientation.z, math.sin((7.0-2*math.pi)/2))
                self.assertEqual(goal()['message'], 'navigation goal already active')
                outcome.set()
                state('SUCCEEDED')
                outcome.clear()
                finish = 'abort'
                self.assertTrue(goal()['success'])
                state('EXECUTING')
                outcome.set()
                state('ABORTED')
                outcome.clear()
                self.assertFalse(goal(-9.0)['success'])
                state('REJECTED')
                self.assertTrue(goal()['success'])
                self.assertTrue(command('cancel_navigation_goal')['success'])
                state('CANCELED')
                self.assertTrue(goal()['success'])
                self.assertTrue(command('stop_mode')['success'])
                state('IDLE')
                self.assertTrue(cancels)
                self.assertTrue(command('start_navigation', map_name='room')['success'])
                pending = send('navigation_goal', x=8.0, y=0.0, yaw=0.0)
                state('PENDING')
                self.assertEqual(goal()['message'], 'navigation goal already active')
                # Stop before acceptance; late acceptance must be canceled and remain IDLE.
                cancel_count = len(cancels)
                stop_id = send('stop_mode')
                self.assertEqual(reply(pending)['message'], 'navigation mode ended')
                self.assertTrue(reply(stop_id)['success'])
                time.sleep(0.8)
                stamp = node.get_clock().now().nanoseconds / 1e9
                receive(lambda p: p['type'] == 'robot_state' and p['timestamp'] > stamp and p['navigation_state'] == 'IDLE')
                self.assertGreater(len(cancels), cancel_count)
                self.assertTrue(command('start_navigation', map_name='room')['success'])
                self.assertTrue(goal()['success'])
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
                self.assertEqual(process.returncode, 0)
            finally:
                outcome.set()
                if client:
                    client.close()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=10)
                if process.returncode:
                    log.seek(0)
                    print(log.read())
                log.close()
                executor.shutdown(timeout_sec=3)
                worker.join(timeout=3)
                server.destroy()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
