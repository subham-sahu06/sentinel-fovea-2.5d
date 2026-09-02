# ROS 2 Safety Gateway

Build and source the workspace:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run safety_gateway safety_gateway
```

The gateway starts latched. The dashboard must publish a heartbeat and an explicit
`false` value on `/dashboard/emergency_stop` before motion is enabled.

## Topics

- `/dashboard/cmd_vel` (`geometry_msgs/Twist`): untrusted operator command input
- `/dashboard/heartbeat` (`std_msgs/Empty`): dashboard liveness signal
- `/dashboard/emergency_stop` (`std_msgs/Bool`): `true` latches stop, `false` requests reset
- `/cmd_vel` (`geometry_msgs/Twist`): limited robot command output
- `/safety_gateway/status` (`std_msgs/String`): JSON safety state and timing telemetry

The gateway clamps velocity, rejects non-finite values, and publishes zero velocity
when the command or heartbeat becomes stale.