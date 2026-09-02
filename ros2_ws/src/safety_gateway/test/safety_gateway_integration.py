#!/usr/bin/env python3
import json
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class GatewayVerifier(Node):
    def __init__(self) -> None:
        super().__init__('safety_gateway_verifier')
        self.command_publisher = self.create_publisher(Twist, '/dashboard/cmd_vel', 10)
        self.heartbeat_publisher = self.create_publisher(Empty, '/dashboard/heartbeat', 10)
        self.estop_publisher = self.create_publisher(Bool, '/dashboard/emergency_stop', 10)
        self.output = None
        self.status = None
        self.create_subscription(Twist, '/cmd_vel', self._on_output, 10)
        self.create_subscription(String, '/safety_gateway/status', self._on_status, 10)

    def _on_output(self, message: Twist) -> None:
        self.output = message

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)

    def heartbeat(self) -> None:
        self.heartbeat_publisher.publish(Empty())

    def command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command_publisher.publish(message)

    def estop(self, latched: bool) -> None:
        message = Bool()
        message.data = latched
        self.estop_publisher.publish(message)


def wait_for(node, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('timed out waiting for expected gateway state')


def assert_zero(node, message):
    assert abs(message.linear.x) < 1e-6, 'gateway output was not stopped'
    assert abs(message.angular.z) < 1e-6, 'gateway output was not stopped'


def main() -> int:
    gateway = subprocess.Popen([
        'ros2', 'run', 'safety_gateway', 'safety_gateway',
        '--ros-args', '-p', 'command_timeout_sec:=0.25', '-p', 'heartbeat_timeout_sec:=0.5',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    rclpy.init()
    verifier = GatewayVerifier()
    try:
        wait_for(verifier, lambda: verifier.status is not None)
        assert verifier.status['state'] == 'ESTOP_LATCHED', 'gateway did not start latched'

        verifier.heartbeat()
        verifier.estop(False)
        wait_for(verifier, lambda: verifier.status.get('state') == 'READY')

        verifier.command(3.0, 4.0)
        verifier.heartbeat()
        wait_for(verifier, lambda: verifier.status.get('state') == 'ACTIVE')
        assert abs(verifier.output.linear.x - 1.0) < 1e-6, 'linear velocity was not clamped'
        assert abs(verifier.output.angular.z - 1.5) < 1e-6, 'angular velocity was not clamped'

        verifier.estop(True)
        wait_for(verifier, lambda: verifier.status.get('state') == 'ESTOP_LATCHED' and verifier.output is not None)
        wait_for(verifier, lambda: verifier.output is not None and abs(verifier.output.linear.x) < 1e-6 and abs(verifier.output.angular.z) < 1e-6)

        verifier.estop(False)
        verifier.heartbeat()
        wait_for(verifier, lambda: verifier.status.get('state') == 'READY')
        verifier.command(0.4, 0.2)
        verifier.heartbeat()
        wait_for(verifier, lambda: verifier.status.get('state') == 'ACTIVE')
        time.sleep(0.7)
        wait_for(verifier, lambda: verifier.status.get('state') == 'SAFE_STOP' and verifier.output is not None)
        wait_for(verifier, lambda: verifier.output is not None and abs(verifier.output.linear.x) < 1e-6 and abs(verifier.output.angular.z) < 1e-6)
        print('PASS: startup latch, reset, velocity clamping, E-stop, and heartbeat timeout')
        return 0
    finally:
        verifier.destroy_node()
        rclpy.shutdown()
        gateway.terminate()
        try:
            gateway.wait(timeout=2)
        except subprocess.TimeoutExpired:
            gateway.kill()
        if gateway.returncode not in (0, -15):
            print(gateway.stderr.read(), file=sys.stderr)


if __name__ == '__main__':
    raise SystemExit(main())