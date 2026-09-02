import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class SafetyGateway(Node):
    def __init__(self) -> None:
        super().__init__('safety_gateway')
        self.declare_parameter('input_cmd_topic', '/dashboard/cmd_vel')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('heartbeat_topic', '/dashboard/heartbeat')
        self.declare_parameter('estop_topic', '/dashboard/emergency_stop')
        self.declare_parameter('status_topic', '/safety_gateway/status')
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('heartbeat_timeout_sec', 0.5)
        self.declare_parameter('max_linear_mps', 1.0)
        self.declare_parameter('max_angular_rps', 1.5)

        input_topic = self.get_parameter('input_cmd_topic').value
        output_topic = self.get_parameter('output_cmd_topic').value
        heartbeat_topic = self.get_parameter('heartbeat_topic').value
        estop_topic = self.get_parameter('estop_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.max_linear = abs(float(self.get_parameter('max_linear_mps').value))
        self.max_angular = abs(float(self.get_parameter('max_angular_rps').value))

        self.output_publisher = self.create_publisher(Twist, output_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_command, 10)
        self.create_subscription(Empty, heartbeat_topic, self._on_heartbeat, 10)
        self.create_subscription(Bool, estop_topic, self._on_estop, 10)
        self.timer = self.create_timer(0.05, self._tick)

        self.estop_latched = True
        self.last_command_time: Optional[float] = None
        self.last_heartbeat_time: Optional[float] = None
        self.requested = Twist()
        self.output = Twist()
        self.reason = 'startup'
        self._publish_status()
        self.get_logger().info('Safety gateway started latched; waiting for dashboard reset and heartbeat')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, message: Twist) -> None:
        self.last_command_time = self._now()
        self.requested = message

    def _on_heartbeat(self, _message: Empty) -> None:
        self.last_heartbeat_time = self._now()

    def _on_estop(self, message: Bool) -> None:
        if message.data:
            self.estop_latched = True
            self.reason = 'emergency_stop'
        elif self.last_heartbeat_time is not None and self._now() - self.last_heartbeat_time <= self.heartbeat_timeout:
            self.estop_latched = False
            self.reason = 'operator_reset'
        else:
            self.reason = 'reset_rejected_no_heartbeat'
        self._publish_status()

    def _tick(self) -> None:
        now = self._now()
        heartbeat_age = math.inf if self.last_heartbeat_time is None else now - self.last_heartbeat_time
        command_age = math.inf if self.last_command_time is None else now - self.last_command_time
        heartbeat_valid = heartbeat_age <= self.heartbeat_timeout
        command_valid = command_age <= self.command_timeout

        if self.estop_latched:
            self.reason = self.reason or 'emergency_stop'
        elif not heartbeat_valid:
            self.reason = 'heartbeat_timeout'
        elif not command_valid:
            self.reason = 'command_timeout'
        else:
            self.reason = 'active'

        if not self.estop_latched and heartbeat_valid and command_valid:
            self.output = self._limited_twist(self.requested)
        else:
            self.output = Twist()
        self.output_publisher.publish(self.output)
        self._publish_status(heartbeat_age, command_age)

    def _limited_twist(self, requested: Twist) -> Twist:
        limited = Twist()
        limited.linear.x = max(-self.max_linear, min(self.max_linear, self._finite(requested.linear.x)))
        limited.angular.z = max(-self.max_angular, min(self.max_angular, self._finite(requested.angular.z)))
        return limited

    @staticmethod
    def _finite(value: float) -> float:
        return value if math.isfinite(value) else 0.0

    def _publish_status(self, heartbeat_age: float = math.inf, command_age: float = math.inf) -> None:
        status = String()
        state = 'ESTOP_LATCHED' if self.estop_latched else ('SAFE_STOP' if heartbeat_age > self.heartbeat_timeout else ('ACTIVE' if command_age <= self.command_timeout else 'READY'))
        status.data = json.dumps({
            'state': state,
            'reason': self.reason,
            'heartbeat_age_sec': None if math.isinf(heartbeat_age) else round(heartbeat_age, 3),
            'command_age_sec': None if math.isinf(command_age) else round(command_age, 3),
            'output_linear_mps': round(self.output.linear.x, 3),
            'output_angular_rps': round(self.output.angular.z, 3),
        })
        self.status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()