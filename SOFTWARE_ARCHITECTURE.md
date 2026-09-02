# Robot Dashboard: Software Architecture

**Document Version:** 1.0  
**Last Updated:** 2026-08-29  
**Audience:** Developers, architects, system designers

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [System Components](#system-components)
3. [Data Flow Architecture](#data-flow-architecture)
4. [Module Dependencies](#module-dependencies)
5. [Topic & Message Architecture](#topic--message-architecture)
6. [Safety Architecture](#safety-architecture)
7. [Scalability & Performance](#scalability--performance)
8. [Deployment Architecture](#deployment-architecture)

---

## Architecture Overview

### Architectural Style: Microservice-Based Pub-Sub Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│                        ROS 2 DDS Middleware                          │
├──────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐            │
│  │ Perception  │    │    Safety    │    │   Odometry   │            │
│  │  Pipeline   │───→│   Gateway    │───→│  Feedback    │            │
│  │   (10 Hz)   │    │   (20 Hz)    │    │   (20 Hz)    │            │
│  └─────────────┘    └──────────────┘    └──────────────┘            │
│                                                │                     │
│                                                ↓                     │
│                                    ┌──────────────────┐              │
│                                    │   rosbridge_ws   │              │
│                                    │   (port 9090)    │              │
│                                    └──────────────────┘              │
│                                           │                         │
└───────────────────────────────────────────┼─────────────────────────┘
                                            │
                                    [WebSocket Bridge]
                                            │
┌───────────────────────────────────────────┼─────────────────────────┐
│                    Browser (JavaScript)   │                         │
│                                           ↓                         │
│                                 ┌──────────────────┐                │
│                                 │  React Dashboard │                │
│                                 │  (60 FPS render) │                │
│                                 └──────────────────┘                │
│                                           │                         │
│                                           ↓                         │
│                                 ┌──────────────────┐                │
│                                 │  Gamepad Input   │                │
│                                 │  Keyboard Input  │                │
│                                 └──────────────────┘                │
│                                           │                         │
└───────────────────────────────────────────┼─────────────────────────┘
                                            │
                                    [WebSocket Bridge]
                                            │
                                            ↓
                        ┌──────────────────────────────┐
                        │   /dashboard/cmd_vel         │
                        │   /dashboard/heartbeat       │
                        │   /dashboard/emergency_stop  │
                        └──────────────────────────────┘
```

### Architectural Layers

| Layer | Purpose | Technology | Rate |
|-------|---------|-----------|------|
| **Perception** | Sensor fusion → obstacle detection | Python + ROS 2 | 10 Hz |
| **Safety** | Command validation + gating | Python + ROS 2 | 20 Hz |
| **Motion** | Pose estimation → telemetry | Python + ROS 2 | 20 Hz |
| **Bridge** | ROS ↔ Browser serialization | rosbridge_websocket | Event-driven |
| **Interface** | Operator control + visualization | React 18 + Vite | 60 FPS |

---

## System Components

### Component 1: Perception Pipeline

```
Inputs:  /dashboard/cmd_vel (for odometry only)
Outputs: /lidar/points, /filtered_points, /adaptive_grid/occupancy
Rate:    10 Hz
```

#### Subcomponent 1.1: Synthetic LiDAR (`synthetic_lidar.py`)

**Responsibility**: Generate point cloud mimicking real LiDAR scanner

**Architecture**:

```python
class SyntheticLidar(Node):
    # Publishers
    publisher: Publisher[PointCloud2]          # /lidar/points
    odometry_publisher: Publisher[Odometry]    # /odom
    
    # Subscribers
    cmd_vel_sub: Subscription[Twist]           # /cmd_vel
    
    # Timers
    publish_timer: Timer(0.1s)                 # 10 Hz point generation
    odometry_timer: Timer(0.05s)               # 20 Hz odometry update
    
    # State
    phase: float                               # Animation parameter
    pose_x, pose_y, yaw: float                 # Integrated odometry
    cmd_vel: Twist                             # Latest command
```

**Implementation Details**:

| Method | Purpose | Complexity |
|--------|---------|-----------|
| `_publish()` | Generate 720-point cloud + obstacles | O(n) = O(1) for fixed n |
| `_update_odometry()` | Integrate /cmd_vel into pose | O(1) |
| `_on_cmd_vel()` | Store latest motion command | O(1) |

**Message Format** (PointCloud2):
```python
PointCloud2:
  header:
    stamp: <current_time>
    frame_id: "base_link"
  width: 784           # Total points
  height: 1            # Unorganized cloud
  fields: [x, y, z]    # FLOAT32 each
  data: <packed bytes> # 784 points × 12 bytes/point
```

---

#### Subcomponent 1.2: Ground Filter (`ground_filter.py`)

**Responsibility**: Remove ground plane from point cloud

**Architecture**:

```python
class GroundFilter(Node):
    # Subscribers
    lidar_sub: Subscription[PointCloud2]       # /lidar/points
    
    # Publishers
    filtered_pub: Publisher[PointCloud2]       # /filtered_points
    
    # Parameters
    ground_threshold: float = 0.1              # Z-height threshold (meters)
    
    # State
    last_points: PointCloud2
```

**Algorithm** (pseudo-code):
```python
def filter_points(cloud):
    filtered = []
    for point in unpack_cloud(cloud):
        if point.z >= ground_threshold:  # Above ground
            filtered.append(point)
    return pack_cloud(filtered)
```

**Complexity**: O(n) where n = number of points (720)

---

#### Subcomponent 1.3: Adaptive Grid (`adaptive_grid.py`)

**Responsibility**: Accumulate points into occupancy cells

**Architecture**:

```python
class AdaptiveGrid(Node):
    # Subscribers
    filtered_sub: Subscription[PointCloud2]    # /filtered_points
    
    # Publishers
    occupancy_pub: Publisher[OccupancyGrid]    # /adaptive_grid/occupancy
    markers_pub: Publisher[MarkerArray]        # /adaptive_grid/elevation_markers
    
    # Grid state
    grid_size_m: float = 20.0                  # 20m × 20m coverage
    cell_resolution: float = 0.5               # 50cm cells
    cells: numpy.ndarray[40, 40]               # Grid array
    cell_hits: numpy.ndarray[40, 40]           # Hit count per cell
    
    # Parameters
    occupancy_threshold: int = 50              # Confidence threshold
```

**Algorithm** (Bayesian occupancy):
```
For each point (x, y) in filtered_points:
    cell_index = world_to_grid(x, y)
    cell_hits[cell_index] += 1
    
    if cell_hits[cell_index] >= threshold:
        cells[cell_index].occupancy = 100
    else:
        cells[cell_index].occupancy = cell_hits[cell_index] * 100 / threshold
```

**Complexity**: O(n) per cloud; O(m) to publish grid where m = 1600 cells

---

### Component 2: Safety Gateway

```
Inputs:  /dashboard/cmd_vel, /dashboard/heartbeat, /dashboard/emergency_stop
Outputs: /cmd_vel, /safety_gateway/status
Rate:    20 Hz
```

#### Architecture

```python
class SafetyGateway(Node):
    # Subscribers
    cmd_sub: Subscription[Twist]               # /dashboard/cmd_vel
    hb_sub: Subscription[Empty]                # /dashboard/heartbeat
    estop_sub: Subscription[Bool]              # /dashboard/emergency_stop
    
    # Publishers
    cmd_out_pub: Publisher[Twist]              # /cmd_vel
    status_pub: Publisher[String]              # /safety_gateway/status
    
    # State machine
    estop_latched: bool = True
    reason: str = "ESTOP_LATCHED"
    
    # Timeouts
    heartbeat_timeout_sec: float = 0.5
    command_timeout_sec: float = 0.25
    
    # Velocity limits
    max_linear_mps: float = 1.0
    max_angular_rps: float = 1.5
    
    # Time tracking
    last_heartbeat_time: Optional[float] = None
    last_command_time: Optional[float] = None
    
    # Timer
    tick_timer: Timer(0.05s)                   # 20 Hz decision rate
```

#### State Machine (Formal Specification)

```
States: {ESTOP_LATCHED, RESET_REJECTED_NO_HEARTBEAT, OPERATOR_RESET, 
         ACTIVE, HEARTBEAT_TIMEOUT, COMMAND_TIMEOUT, EMERGENCY_STOP}

Transitions:
  ESTOP_LATCHED → (heartbeat arrives) → RESET_REJECTED_NO_HEARTBEAT
  RESET_REJECTED_NO_HEARTBEAT → (estop=false AND heartbeat_valid) → OPERATOR_RESET
  OPERATOR_RESET → (command arrives) → ACTIVE
  ACTIVE → (heartbeat_age > 500ms) → HEARTBEAT_TIMEOUT
  ACTIVE → (command_age > 250ms) → COMMAND_TIMEOUT
  ACTIVE → (estop=true) → EMERGENCY_STOP
  [TIMEOUT states] → (heartbeat_valid AND estop_valid) → ACTIVE
  [EMERGENCY_STOP] → (reset flow) → OPERATOR_RESET

Guards:
  heartbeat_valid = (now - last_heartbeat_time <= 500ms)
  command_valid = (now - last_command_time <= 250ms)
```

#### Velocity Clamping

```python
def clamp_velocity(requested: Twist) -> Twist:
    limited = Twist()
    limited.linear.x = clamp(requested.linear.x, -1.0, 1.0)
    limited.angular.z = clamp(requested.angular.z, -1.5, 1.5)
    return limited
```

#### Decision Logic (20 Hz)

```python
def _tick(self):
    now = current_time()
    
    # Age calculations
    heartbeat_age = now - self.last_heartbeat_time if self.last_heartbeat_time else ∞
    command_age = now - self.last_command_time if self.last_command_time else ∞
    
    # Validation checks
    heartbeat_valid = heartbeat_age <= 0.5
    command_valid = command_age <= 0.25
    
    # State machine
    if estop_latched:
        output = Twist()  # No motion
        reason = "ESTOP_LATCHED"
    elif not heartbeat_valid:
        output = Twist()
        reason = "HEARTBEAT_TIMEOUT"
    elif not command_valid:
        output = Twist()
        reason = "COMMAND_TIMEOUT"
    else:
        output = clamp_velocity(requested)
        reason = "ACTIVE"
    
    # Publish outputs
    cmd_out_pub.publish(output)
    status_pub.publish(json.dumps({
        "state": reason,
        "timestamp_ms": now * 1000,
        "heartbeat_age_ms": heartbeat_age * 1000,
        "command_age_ms": command_age * 1000
    }))
```

---

### Component 3: Odometry Feedback

**Integrated into synthetic_lidar.py** (see Component 1.1)

**Key Equations**:

Kinematic integration over 50ms timestep:
```
x_{k+1} = x_k + v·cos(yaw_k)·Δt
y_{k+1} = y_k + v·sin(yaw_k)·Δt
yaw_{k+1} = yaw_k + ω·Δt
```

Where:
- v = /cmd_vel.linear.x (m/s)
- ω = /cmd_vel.angular.z (rad/s)
- Δt = 0.05s (50ms)

**Odometry Message** (Odometry):
```
header:
  stamp: <current_time>
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position: {x, y, z=0}
    orientation: {x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)}
  covariance: [0, 0, ...] (not set; optional)
twist:
  twist:
    linear: {x=v, y=0, z=0}
    angular: {x=0, y=0, z=ω}
  covariance: [0, 0, ...] (not set; optional)
```

---

### Component 4: WebSocket Bridge (rosbridge_websocket)

**Responsibility**: Serialize ROS 2 topics to/from JSON for browser

**Configuration**:
```python
# In master.launch.py
Node(
    package='rosbridge_server',
    executable='rosbridge_websocket',
    name='rosbridge_websocket',
    output='screen',
    parameters=[{'port': 9090}]
)
```

**Protocol**:
```json
// Client subscribes to topic
{"op": "subscribe", "topic": "/odom", "type": "nav_msgs/Odometry"}

// Server sends message
{"op": "publish", "topic": "/odom", "msg": {"header": {...}, "pose": {...}}}

// Client publishes topic
{"op": "publish", "topic": "/dashboard/cmd_vel", 
 "msg": {"linear": {"x": 0.5}, "angular": {"z": 0.3}}}
```

**Message Serialization** (PointCloud2 example):
```json
{
  "header": {"stamp": {"secs": 1693401652, "nsecs": 123456789}, "frame_id": "base_link"},
  "height": 1,
  "width": 784,
  "fields": [
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1}
  ],
  "is_bigendian": false,
  "point_step": 12,
  "row_step": 9408,
  "data": [...]  // Base64-encoded binary point data
}
```

---

### Component 5: React Dashboard

**Responsibility**: Operator interface + visualization

#### Architecture

```javascript
// App.jsx main component

function App() {
  const [connected, setConnected] = useState(false)
  const [telemetry, setTelemetry] = useState({...})
  const [points, setPoints] = useState([])
  const [occupancy, setOccupancy] = useState(null)
  
  useEffect(() => {
    // Initialize ROSLIB connection
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' })
    
    // Subscribe to multiple topics
    setupSubscription(ros, '/odom', 'nav_msgs/Odometry', setTelemetry)
    setupSubscription(ros, '/filtered_points', 'sensor_msgs/PointCloud2', setPoints)
    setupSubscription(ros, '/adaptive_grid/occupancy', 'nav_msgs/OccupancyGrid', setOccupancy)
    
    // Publish commands on gamepad input
    gamepadListener = setInterval(() => {
      if (gamepadPressed()) {
        publishCommand(ros, '/dashboard/cmd_vel', getGamepadTwist())
        publishHeartbeat(ros, '/dashboard/heartbeat')
      }
    }, 100)  // 10 Hz command rate
  }, [])
  
  return (
    <>
      <PointCloudView points={points} occupancy={occupancy} />
      <TelemetryPanel telemetry={telemetry} />
      <ControlPanel onEmergencyStop={handleEmergencyStop} />
    </>
  )
}
```

#### Data Flow (Subscriptions)

```
Topic: /odom
  ↓ (via ROSLIB)
JavaScript: handleOdometry(message)
  ↓
React State: setTelemetry({actualLinear, actualAngular})
  ↓
Component Re-render
  ↓
HTML Display: "ACTUAL LINEAR: 0.5 m/s"
```

#### Canvas Rendering Pipeline

```
1. requestAnimationFrame(draw)
   ↓
2. Get current velocity from telemetry state
   ↓
3. Update ego pose: x += v·cos(yaw)·dt
   ↓
4. Transform world points to camera frame
   ↓
5. Project 3D points to 2D canvas coordinates
   ↓
6. Render occupancy grid cells
   ↓
7. Render obstacle points + elevation markers
   ↓
8. Next frame request
```

**Computational Complexity**:
- Grid rendering: O(cells) = O(1600)
- Point rendering: O(points) = O(784)
- Total: O(2400) per frame = ~5 ms at 60 FPS

---

## Data Flow Architecture

### Perception → Safety → Feedback Loop

```
Time: T=0, T=50ms, T=100ms, ...
  
T=0ms:
  synthetic_lidar.publish(/lidar/points)  [720 points]
  
T=50ms:
  ground_filter.receive(/lidar/points)
  ground_filter.publish(/filtered_points)  [~135 points]
  synthetic_lidar.publish(/odom)           [pose + twist]
  
T=100ms:
  adaptive_grid.receive(/filtered_points)
  adaptive_grid.publish(/occupancy)        [40×40 cells]
  safety_gateway.tick()
  safety_gateway.publish(/cmd_vel)         [clamped velocity]
  dashboard receives all updates via rosbridge
  
T=150ms:
  synthetic_lidar.receive(/cmd_vel)
  Update odometry internal state
  
T=200ms: [cycle repeats]
```

### End-to-End Latency Breakdown

| Component | Latency | Details |
|-----------|---------|---------|
| Dashboard → ROS topic | 5-10 ms | Gamepad read + publish |
| ROS topic → Safety Gateway | <5 ms | DDS middleware latency |
| Safety decision + clamp | <1 ms | Pure computation |
| Output → synthetic_lidar | <5 ms | Subscription callback |
| Odometry update + publish | <1 ms | Kinematic math |
| Network serialization | 20-30 ms | rosbridge + WebSocket |
| Browser reception + render | 10-50 ms | React + canvas |
| **Total** | **~50-150 ms** | Acceptable for teleop |

---

## Module Dependencies

### Dependency Graph

```
master.launch.py
  │
  ├─→ synthetic_lidar
  │     │
  │     └─→ [ROS 2 core, geometry_msgs, nav_msgs, sensor_msgs]
  │
  ├─→ ground_filter
  │     │
  │     ├─→ synthetic_lidar (/lidar/points)
  │     │
  │     └─→ [ROS 2 core, sensor_msgs]
  │
  ├─→ adaptive_grid
  │     │
  │     ├─→ ground_filter (/filtered_points)
  │     │
  │     └─→ [ROS 2 core, nav_msgs, visualization_msgs]
  │
  ├─→ safety_gateway
  │     │
  │     ├─→ [Dashboard inputs via /dashboard/cmd_vel, /dashboard/heartbeat, /dashboard/emergency_stop]
  │     │
  │     └─→ [ROS 2 core, geometry_msgs, std_msgs]
  │
  └─→ rosbridge_websocket
        │
        ├─→ All ROS 2 topics (pub/sub bridge)
        │
        └─→ [ROS 2 core, rosbridge_server]

Dashboard (Browser)
  │
  ├─→ rosbridge WebSocket (localhost:9090)
  │
  └─→ [React 18, ROSLIB.js, lucide-react, Vite]
```

### Build Dependency Order

```
1. ament_cmake (build system)
2. rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs (ROS core)
3. demo_pipeline (uses core msgs)
4. safety_gateway (uses geometry_msgs)
5. adaptive_grid (uses nav_msgs)
6. robot_bringup (references all 3 above)
7. rosbridge_server (bridges ROS → WebSocket)
```

---

## Topic & Message Architecture

### Topic Topology

```
Published by ROS:
  /lidar/points                     ← synthetic_lidar (PointCloud2)
  /filtered_points                  ← ground_filter (PointCloud2)
  /adaptive_grid/occupancy          ← adaptive_grid (OccupancyGrid)
  /adaptive_grid/elevation_markers  ← adaptive_grid (MarkerArray)
  /odom                             ← synthetic_lidar (Odometry)
  /cmd_vel                          ← safety_gateway (Twist)
  /safety_gateway/status            ← safety_gateway (String/JSON)

Subscribed by Dashboard:
  /dashboard/cmd_vel                → safety_gateway (Twist)
  /dashboard/heartbeat              → safety_gateway (Empty)
  /dashboard/emergency_stop         → safety_gateway (Bool)
```

### Message Schemas

#### PointCloud2

```python
PointCloud2(
  header: Header,
  height: 1,
  width: int,
  fields: List[PointField],
  is_bigendian: bool,
  point_step: int,
  row_step: int,
  data: bytes
)
```

#### OccupancyGrid

```python
OccupancyGrid(
  header: Header,
  info: MapMetaData(
    map_load_time: Time,
    resolution: float,
    width: int,
    height: int,
    origin: Pose
  ),
  data: List[int]  # 0-100 occupancy per cell
)
```

#### Odometry

```python
Odometry(
  header: Header,
  child_frame_id: str,
  pose: PoseWithCovariance(
    pose: Pose(position: Point[x,y,z], orientation: Quaternion),
    covariance: List[36 floats]
  ),
  twist: TwistWithCovariance(
    twist: Twist(linear: Vector3, angular: Vector3),
    covariance: List[36 floats]
  )
)
```

#### Twist

```python
Twist(
  linear: Vector3(x, y, z),
  angular: Vector3(x, y, z)
)
```

---

## Safety Architecture

### Redundant Safety Checks

```
Layer 1: Heartbeat Monitoring (500 ms timeout)
  │
  └─→ If timeout: stop motion + publish HEARTBEAT_TIMEOUT
      ↑
      └─ Automatically recovers when heartbeat resumes
      
Layer 2: E-Stop Latch (requires manual reset)
  │
  └─→ If estop pressed: stop motion + publish EMERGENCY_STOP
      ↑
      └─ Requires explicit reset command (no auto-recovery)
      
Layer 3: Command Timeout (250 ms timeout)
  │
  └─→ If command stale: stop motion + publish COMMAND_TIMEOUT
      ↑
      └─ Automatically recovers with next valid command
      
Layer 4: Velocity Clamping (secondary defense)
  │
  └─→ All velocities bounded: ±1.0 m/s linear, ±1.5 rad/s angular
      ↑
      └─ Active regardless of state machine mode
```

### Safety Verification Method

```python
# Test: Verify heartbeat timeout stops motion
1. Send heartbeat + reset + command → motion starts
2. Wait 600ms without new heartbeat
3. Verify /cmd_vel = Twist() (zeros)
4. Verify /safety_gateway/status = "HEARTBEAT_TIMEOUT"

# Test: Verify E-stop cannot be auto-recovered
1. Send heartbeat + reset + command → motion starts
2. Send estop=true
3. Verify motion stops
4. Try sending new command (should be rejected)
5. Send reset flow (heartbeat + estop=false)
6. Verify motion allowed again
```

---

## Scalability & Performance

### Computational Complexity

| Component | Complexity | Actual Time |
|-----------|-----------|---|
| Point cloud generation | O(1) constant | ~2 ms |
| Ground filtering | O(n) linear | ~1 ms (n=720) |
| Grid accumulation | O(n+m) | ~3 ms (m=1600 cells) |
| Safety gateway logic | O(1) constant | <0.5 ms |
| Odometry integration | O(1) constant | <0.1 ms |
| WebSocket serialization | O(n) linear | ~10 ms (n=720 points) |
| React rendering | O(m) linear | ~5 ms (m=1600 cells + points) |

**Total per cycle**: ~22 ms (well under 100 ms budget)

### Memory Usage

| Component | Memory | Notes |
|-----------|--------|-------|
| Point cloud (720 points) | 12 KB | 720 × 12 bytes/point |
| Ground filtered points | 2 KB | ~135 × 12 bytes |
| Occupancy grid (40×40) | 2 KB | 1600 cells × 1 byte/cell |
| ROS message buffers | ~100 KB | Multiple in-flight messages |
| Dashboard state | ~50 KB | React component state |
| **Total typical** | **~200 KB** | Per ROS cycle |
| **Peak (buffered)** | **~500 KB** | With message queues |

### Bandwidth Analysis

| Data Type | Rate | Bandwidth | Notes |
|-----------|------|-----------|-------|
| /lidar/points | 10 Hz | ~90 KB/s | 720 points × 12 bytes |
| /filtered_points | 10 Hz | ~16 KB/s | ~135 points |
| /occupancy | 10 Hz | ~2 KB/s | 1600 cells |
| /odom | 20 Hz | ~10 KB/s | Odometry message |
| /cmd_vel | 10 Hz | ~1 KB/s | Twist message |
| **Total ROS** | — | **~119 KB/s** | Before serialization |
| **WebSocket overhead** | — | **~50%** | Base64 encoding + JSON |
| **Total bandwidth** | — | **~180 KB/s** | Over network |

**Conclusion**: Suitable for 1 Mbps+ connection (WiFi/Ethernet)

### Scalability Considerations

#### Scaling to Multiple Robots

**Current architecture (single robot)**:
```
/lidar/points → /filtered_points → /adaptive_grid/occupancy → /cmd_vel
```

**Multi-robot (proposed)**:
```
/robot1/lidar/points → /robot1/filtered_points → /robot1/adaptive_grid/occupancy → /robot1/cmd_vel
/robot2/lidar/points → /robot2/filtered_points → /robot2/adaptive_grid/occupancy → /robot2/cmd_vel
...
```

**Changes needed**:
- Namespace each robot's topics (ROS 2 native support)
- Dashboard multiplexes multiple robots (UI change)
- Safety gateway × N (one per robot)
- No architectural changes required (pub-sub naturally supports namespacing)

#### Scaling to Higher Frequency

**Current**: 10 Hz perception, 20 Hz safety/odometry

**If 30 Hz needed** (drones, high-speed robots):
- Increase timer frequencies (0.1s → 0.033s)
- Same algorithm complexity (still O(1) or O(n))
- Computational load increases 3x (still <100ms)
- No algorithm changes required

#### Scaling to More Sensors

**Current**: Single synthetic LiDAR

**If adding RGB camera**:
```
/camera/image → image_processor → /camera/processed → display
```

**No changes to existing pipeline** (pub-sub decoupled)

---

## Deployment Architecture

### Development Deployment

```
Developer Laptop:
  ├─ ROS 2 Jazzy (Ubuntu 22.04)
  ├─ Python 3.10+
  ├─ Node.js 18+ (frontend dev)
  │
  └─ Runs all components locally:
     ├─ ros2 launch robot_bringup master.launch.py
     ├─ npm run dev (dashboard dev server)
     └─ Browser: localhost:5173
```

### Production Deployment (Proposed)

```
Robot Host (embedded PC):
  ├─ ROS 2 Jazzy container (Docker)
  │  ├─ synthetic_lidar node
  │  ├─ ground_filter node
  │  ├─ adaptive_grid node
  │  ├─ safety_gateway node
  │  └─ rosbridge_websocket (port 9090)
  │
  └─ Systemd service (auto-restart)

Operator Workstation (any device):
  ├─ Web browser (Chrome, Firefox, Safari)
  └─ WebSocket connection to robot:9090
```

### Deployment Sequence

```
1. Boot robot PC
2. systemd starts ROS container
3. All 5 nodes launch automatically
4. rosbridge listens on port 9090
5. Operator opens browser → localhost:5173 or robot-ip:9090
6. ROSLIB connects via WebSocket
7. Topics begin flowing
8. Ready for teleop
```

---

## Summary: Architecture Strengths

| Strength | Mechanism |
|----------|-----------|
| **Modularity** | Pub-sub decoupling (nodes independent) |
| **Observability** | JSON status topics + rosbag logging |
| **Safety** | Redundant validation gates (defense-in-depth) |
| **Extensibility** | Namespace isolation (multi-robot ready) |
| **Real-time** | Deterministic rates (10 Hz perception, 20 Hz safety) |
| **Web-first** | Browser-based UI (no desktop ROS client needed) |
| **Debuggability** | Standard ROS tools (rosbag, rqt, rviz compatible) |

---

**End of Software Architecture Document**
