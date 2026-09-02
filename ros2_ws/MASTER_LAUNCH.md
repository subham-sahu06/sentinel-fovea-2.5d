# Master ROS 2 launch

Build and start the complete local demo pipeline:

```bash
cd /home/subham/robot-dashboard/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup master.launch.py
```

This starts synthetic LiDAR, ground filtering, the safety gateway, adaptive
elevation grid, and `rosbridge_websocket` on port `9090`.