import json
import math
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from safety_gateway.node import SafetyGateway


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_initial_state(rclpy_init):
    node = SafetyGateway()
    try:
        assert node.estop_latched is True
        assert node.reason == 'startup'
        assert node.last_heartbeat_time is None
        assert node.last_command_time is None
        assert node.output.linear.x == 0.0
        assert node.output.angular.z == 0.0
    finally:
        node.destroy_node()


def test_velocity_clamping(rclpy_init):
    node = SafetyGateway()
    try:
        # Within limits
        req1 = Twist()
        req1.linear.x = 0.5
        req1.angular.z = -1.0
        out1 = node._limited_twist(req1)
        assert abs(out1.linear.x - 0.5) < 1e-6
        assert abs(out1.angular.z - (-1.0)) < 1e-6

        # Exceeding limits
        req2 = Twist()
        req2.linear.x = 5.0
        req2.angular.z = -3.5
        out2 = node._limited_twist(req2)
        assert abs(out2.linear.x - 1.0) < 1e-6
        assert abs(out2.angular.z - (-1.5)) < 1e-6

        # Non-finite values should be zeroed out
        req3 = Twist()
        req3.linear.x = float('nan')
        req3.angular.z = float('inf')
        out3 = node._limited_twist(req3)
        assert out3.linear.x == 0.0
        assert out3.angular.z == 0.0
    finally:
        node.destroy_node()


def test_reset_requires_heartbeat(rclpy_init):
    node = SafetyGateway()
    try:
        # Attempt reset without heartbeat -> rejected
        node._on_estop(Bool(data=False))
        assert node.estop_latched is True
        assert node.reason == 'reset_rejected_no_heartbeat'

        # Send heartbeat -> reset succeeds
        node._on_heartbeat(Empty())
        node._on_estop(Bool(data=False))
        assert node.estop_latched is False
        assert node.reason == 'operator_reset'
    finally:
        node.destroy_node()


def test_state_machine_transitions(rclpy_init):
    node = SafetyGateway()
    try:
        # Start latched
        node._tick()
        assert node.output.linear.x == 0.0

        # Heartbeat + Reset
        node._on_heartbeat(Empty())
        node._on_estop(Bool(data=False))

        # Command arriving
        cmd = Twist()
        cmd.linear.x = 0.8
        cmd.angular.z = 0.5
        node._on_command(cmd)
        node._tick()

        assert node.reason == 'active'
        assert abs(node.output.linear.x - 0.8) < 1e-6
        assert abs(node.output.angular.z - 0.5) < 1e-6

        # Command timeout (simulate stale command)
        node.last_command_time = node._now() - 0.30
        node._tick()
        assert node.reason == 'command_timeout'
        assert node.output.linear.x == 0.0

        # Heartbeat timeout (simulate stale heartbeat)
        node.last_heartbeat_time = node._now() - 0.60
        node._tick()
        assert node.reason == 'heartbeat_timeout'
        assert node.output.linear.x == 0.0

        # Emergency Stop Trigger
        node._on_estop(Bool(data=True))
        assert node.estop_latched is True
        assert node.reason == 'emergency_stop'
        node._tick()
        assert node.output.linear.x == 0.0
    finally:
        node.destroy_node()
