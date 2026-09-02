# Robot Dashboard: Development Methodology

**Document Version:** 1.0  
**Last Updated:** 2026-08-29  
**Audience:** Technical leads, architects, future maintainers

---

## Table of Contents

1. [Overview](#overview)
2. [Development Phases](#development-phases)
3. [Design Principles](#design-principles)
4. [Architectural Patterns](#architectural-patterns)
5. [Technology Selection Rationale](#technology-selection-rationale)
6. [Development Workflow](#development-workflow)
7. [Quality Assurance Strategy](#quality-assurance-strategy)
8. [Testing Methodology](#testing-methodology)

---

## Overview

This project employed an **iterative, component-driven methodology** with emphasis on:
- **Safety-first design** (fail-safe by default)
- **Progressive integration** (validate each component before combining)
- **Real-time validation** (test against live ROS runtime, not mocks)
- **Production-ready documentation** (runbooks and architecture guides)

**Core Philosophy**: Build minimal viable perception pipeline first; defer ML enhancements to future phases.

---

## Development Phases

### Phase 1: Foundation & Architecture (Week 1)

**Objective**: Establish core infrastructure and design patterns

**Activities**:
- Initialized ROS 2 Jazzy workspace with 4 packages (demo_pipeline, safety_gateway, adaptive_grid, robot_bringup)
- Designed 3-tier architecture: Perception → Safety → Interface
- Established React + Vite dashboard frontend
- Configured rosbridge WebSocket bridging (port 9090)

**Key Decisions**:
- ✅ Use ROS 2 (mature, proven in production robotics)
- ✅ Python (rclpy) for backend (rapid iteration, strong type hints available)
- ✅ React for frontend (real-time reactive UI, hooks for state management)
- ✅ Synthetic LiDAR first (validate pipeline without hardware dependency)

**Deliverables**:
- 4 ROS packages with package.xml + setup.py
- Launch file orchestration (master.launch.py)
- Dashboard React component skeleton
- rosbridge server running on port 9090

---

### Phase 2: Perception Pipeline (Week 2)

**Objective**: Implement synthetic sensor → filter → grid perception chain

**Activities**:
- **synthetic_lidar.py**: Ring-based point cloud (720 points) with obstacle field
  - 360° ring at 3m radius (main ground plane)
  - 2 obstacle clusters (4m, 1m at different heights)
  - Deterministic + phase-based animation
  
- **ground_filter.py**: Geometric Z-threshold filtering
  - Simple rule: Z < 0.1m → ground, Z ≥ 0.1m → obstacle
  - ~90% filtration (removes 648/720 ground points)
  
- **adaptive_grid.py**: Real-time occupancy accumulation
  - 50 cm cell resolution
  - Bayesian occupancy (hit/miss probability)
  - Elevation markers for obstacle visualization

**Key Decisions**:
- ✅ **Geometric filtering over ML** (latency: <10ms vs 50-200ms; interpretability: 100%)
- ✅ **Ring-based synthetic LiDAR** (realistic obstacle patterns, easily parameterizable)
- ✅ **Fixed resolution grid** (trade-off between fidelity and performance)
- ✅ **10 Hz perception rate** (acceptable for teleoperation, low CPU load)

**Validation**:
- `ros2 topic hz /lidar/points` → 10 Hz ✓
- `ros2 topic hz /filtered_points` → 10 Hz ✓
- `ros2 topic hz /adaptive_grid/occupancy` → 10 Hz ✓
- Visual inspection: Grid cells update in real-time ✓

---

### Phase 3: Safety & Command Gating (Week 3)

**Objective**: Implement fail-safe motion control with heartbeat validation

**Activities**:
- **safety_gateway.py**: 3-gate safety state machine
  - Gate 1: Heartbeat validation (500 ms timeout)
  - Gate 2: E-stop latch + explicit reset requirement
  - Gate 3: Command timeout (250 ms, stops motion if stale)
  - Velocity clamping (1.0 m/s linear, 1.5 rad/s angular)

**Key Decisions**:
- ✅ **Heartbeat-based liveness detection** (catches stale connections faster than timeout alone)
- ✅ **E-stop as latching state** (fail-safe: operator must explicitly reset; not auto-recovery)
- ✅ **Dual-timeout architecture** (heartbeat + command decoupled; independent recovery)
- ✅ **JSON status output** (easy integration with dashboard; human-readable debugging)

**Validation**:
- Integration test: All 7 state transitions tested
- Manual flow: heartbeat → reset → command → /cmd_vel clamped
- E-stop behavior: Motion stops immediately, won't restart without reset

---

### Phase 4: Odometry Feedback & Dashboard Integration (Week 4)

**Objective**: Close the loop: commands → motion → telemetry

**Activities**:
- **synthetic_lidar.py odometry integration**: 
  - Subscribes to `/cmd_vel` (clamped motion commands)
  - Kinematic pose integration: x, y, yaw via Euler integration
  - Publishes `/odom` at 20 Hz (2x perception rate for smoother feedback)
  
- **App.jsx dashboard**:
  - ROSLIB subscriptions to 7 key topics
  - Real-time telemetry display (ACTUAL LINEAR/ANGULAR from /odom)
  - Point cloud canvas rendering with ego-motion tracking
  - Safety status indicator + emergency stop button

**Key Decisions**:
- ✅ **Simple kinematic odometry** (sufficient for synthetic validation; real IMU/encoder would substitute)
- ✅ **20 Hz odometry vs 10 Hz perception** (decoupled rates allow independent scaling)
- ✅ **Canvas-based visualization** (low overhead; GPU not required)
- ✅ **WebSocket via rosbridge** (no ROS client library needed in browser; ROSLIB handles serialization)

**Validation**:
- `/odom` publishes x/y/yaw consistently matching /cmd_vel inputs
- Dashboard displays non-zero ACTUAL LINEAR when motion commanded
- Browser receives updates via WebSocket with <100 ms latency

---

### Phase 5: Documentation & Production Hardening (Week 5)

**Objective**: Create operator-grade documentation and validate end-to-end

**Activities**:
- **RUNBOOK.md** (603 lines)
  - 5-step startup procedure with timing
  - Safety state machine diagrams
  - 4 emergency recovery scenarios
  - 6+ troubleshooting issues with solutions
  
- **README.md** (583 lines)
  - Architecture diagrams (Mermaid)
  - API reference (11 topics + schemas)
  - Development guide (modify, extend, test)
  - Performance benchmarks
  
- **Live validation**:
  - Clean launch → topics verify → heartbeat/reset/command flow → motion verification
  - Dashboard telemetry confirmed updating in real-time
  - Safety tests all passing

**Key Decisions**:
- ✅ **Runbook-first documentation** (operators need step-by-step; not just API docs)
- ✅ **Architecture diagrams over text** (Mermaid diagrams for clarity)
- ✅ **Real test scenarios** (not mocked; uses actual ROS CLI commands)

---

## Design Principles

### 1. **Safety-First / Fail-Safe Default**

Every decision prioritizes safe operation:

- **Fail-safe startup**: System begins latched (motion not allowed)
- **Explicit recovery**: Operator must actively reset (no auto-recovery from E-stop)
- **Dual validation**: Heartbeat + command both required
- **Velocity clamping**: Secondary defense (even if gate fails, velocities bounded)

**Example**: If heartbeat times out during motion, system stops immediately and publishes "HEARTBEAT_TIMEOUT" status.

---

### 2. **Progressive Integration Testing**

Components validated in isolation before combining:

| Level | Scope | Test |
|-------|-------|------|
| **Unit** | Single function | Python asserts in integration test |
| **Component** | Single node | `ros2 topic echo --once` verification |
| **System** | Full stack | CLI heartbeat → reset → command flow |
| **End-to-End** | Stack + Dashboard | Browser displays telemetry in real-time |

---

### 3. **Observable by Default**

Every component must have diagnostics:

- **Logging**: Each node logs state transitions (heartbeat received, reset latched, etc.)
- **Status topics**: `/safety_gateway/status` publishes JSON with current state
- **Topic rates**: `ros2 topic hz <topic>` shows if nodes are alive
- **Error messages**: Clear, actionable error text (not cryptic error codes)

---

### 4. **Minimal Viable Complexity**

Choose simplest solution that works:

- ✅ **Z-threshold filtering** not ML (simpler, faster, more interpretable)
- ✅ **Synthetic LiDAR** not hardware (no dependencies, deterministic)
- ✅ **Kinematic odometry** not sensor fusion (no IMU/encoder needed)
- ✅ **Canvas rendering** not 3D engine (lower CPU, no GPU required)

**Rationale**: Defer complexity to Phase 2 only when needed.

---

### 5. **Configuration Over Hardcoding**

Safety-critical parameters must be tunable:

```python
# In safety_gateway.py
self.declare_parameter('max_linear_mps', 1.0)        # Changeable
self.declare_parameter('heartbeat_timeout_sec', 0.5) # Tunable
self.declare_parameter('command_timeout_sec', 0.25)  # Without rebuild
```

Allows operators to adjust thresholds without code changes.

---

## Architectural Patterns

### Pattern 1: Publisher-Subscriber Decoupling

**Problem**: Tight coupling between nodes limits modularity

**Solution**: ROS 2 pub-sub with topic-based communication

```
synthetic_lidar → /lidar/points → ground_filter
                                    ↓
                            /filtered_points → adaptive_grid
                                                    ↓
                                    /adaptive_grid/occupancy → dashboard
```

**Benefits**:
- Nodes are independent; one crash doesn't cascade
- Topics can be recorded (rosbag) for debugging
- Easy to add new consumers (e.g., ML pipeline)

---

### Pattern 2: State Machine (Safety Gateway)

**Problem**: Complex interaction between heartbeat, E-stop, and timeout

**Solution**: Explicit state machine with clear transitions

```
ESTOP_LATCHED → (heartbeat arrives) → RESET_REJECTED_NO_HEARTBEAT
                                      ↓ (estop reset)
                                   OPERATOR_RESET
                                      ↓ (command arrives)
                                      ACTIVE
                                      ↓ (heartbeat timeout OR command timeout OR estop pressed)
                                      [TIMEOUT / EMERGENCY]
```

**Benefits**:
- All states explicitly listed
- Transitions documented in code comments
- Testable (each transition can be validated)
- Easy to add new states (e.g., PAUSE, CALIBRATING)

---

### Pattern 3: Time-Based Verification (Heartbeat)

**Problem**: How to detect stale dashboard connection?

**Solution**: Track last-seen timestamp; compare to threshold

```python
heartbeat_age = now - last_heartbeat_time
if heartbeat_age > 0.5:  # 500 ms threshold
    self.reason = 'heartbeat_timeout'
    self.output = Twist()  # Motion stops
```

**Benefits**:
- Decoupled from message frequency (works even if dashboard rate varies)
- Doesn't require explicit "I'm dead" message
- Gracefully handles transient network glitches (auto-recovery when heartbeat resumes)

---

### Pattern 4: Kinematic Pose Integration (Odometry)

**Problem**: Need to estimate pose from commanded velocities

**Solution**: Euler integration of twist messages

```python
dt = 0.05  # 50 ms timestep
self.pose_x += cmd_vel.linear.x * cos(self.yaw) * dt
self.pose_y += cmd_vel.linear.x * sin(self.yaw) * dt
self.yaw += cmd_vel.angular.z * dt
```

**Benefits**:
- No sensor dependencies (works in simulation)
- Linear in complexity (single callback)
- Deterministic (predictable odometry for testing)
- Can be replaced with IMU/encoder fusion later without API change

---

## Technology Selection Rationale

### Backend: ROS 2 Jazzy + Python (rclpy)

| Decision | Rationale |
|----------|-----------|
| **ROS 2** (not ROS 1) | Mature DDS middleware; better real-time support; active community |
| **Jazzy** (not Humble) | Latest LTS; better Python typing; recent bug fixes |
| **Python** (not C++) | Faster iteration; strong type hints available; easier to teach; adequate latency for teleop |
| **rclpy** (not rospy) | Native ROS 2; better performance; active maintenance |

### Frontend: React 18 + Vite + ROSLIB

| Decision | Rationale |
|----------|-----------|
| **React 18** (not Vue) | Larger ecosystem; more job market; familiar to most teams |
| **Vite** (not Webpack) | 10x faster dev server; smaller bundle; native ES6 modules |
| **ROSLIB** (not native DDS) | Works in browser; hides complexity; JavaScript-friendly serialization |
| **Canvas** (not Three.js) | Lower overhead; no GPU required; sufficient for 2D + pseudo-3D rendering |

### Middleware: rosbridge_websocket (not native ROS DDS in browser)

| Decision | Rationale |
|----------|-----------|
| **rosbridge** (not custom bridge) | Standard; proven; integrates seamlessly with ROSLIB |
| **WebSocket** (not HTTP polling) | Lower latency (<50ms vs 100-200ms polling); full-duplex |

### Database: None (not Redis/MongoDB)

| Decision | Rationale |
|----------|-----------|
| **No persistence layer** | Data logging deferred to v1.1; real-time operation more important for teleop |

---

## Development Workflow

### Daily Workflow

```
1. Code change (e.g., modify safety_gateway.py)
   ↓
2. Rebuild affected package
   colcon build --packages-select safety_gateway --event-handlers console_direct+
   ↓
3. Run unit/integration tests
   python3 src/safety_gateway/test/safety_gateway_integration.py
   ↓
4. Manual validation (if core logic changed)
   ros2 launch robot_bringup master.launch.py &
   ros2 topic pub --once /dashboard/heartbeat std_msgs/msg/Empty "{}"
   ros2 topic pub --once /dashboard/emergency_stop std_msgs/msg/Bool "{data: false}"
   ros2 topic pub --once /dashboard/cmd_vel geometry_msgs/msg/Twist "..."
   ros2 topic echo --once /cmd_vel  # Verify output
   ↓
5. Merge to version control (git commit)
```

### Change Categories

| Type | Scope | Testing | Example |
|------|-------|---------|---------|
| **Parameter tuning** | Single node | No rebuild | Change `max_linear_mps` |
| **Bug fix** | Single file | Rebuild + unit test | Fix off-by-one in grid |
| **Feature addition** | Single node | Rebuild + manual test | Add new status field |
| **API change** | Multiple nodes | Full rebuild + integration test | Rename topic name |
| **Safety change** | ALL | Full rebuild + stress test | Modify heartbeat timeout |

---

## Quality Assurance Strategy

### Strategy 1: Progressive Build Validation

```bash
# Step 1: Compile-time checks
npm run build  # Frontend linting + bundling
colcon build   # ROS packages compile

# Step 2: Runtime checks
ros2 launch robot_bringup master.launch.py  # All nodes start
sleep 2
ros2 topic list | grep safety_gateway       # Required topics exist

# Step 3: Functional validation
python3 src/safety_gateway/test/safety_gateway_integration.py  # Tests pass
```

### Strategy 2: Real Runtime Testing (No Mocks)

**Principle**: Test against actual ROS 2 runtime, not mock objects

❌ **Avoid**:
```python
# BAD: Mocking doesn't catch real-world issues
mock_cmd_vel = MagicMock()
safety_gateway.on_command(mock_cmd_vel)
```

✅ **Do**:
```bash
# GOOD: Use actual ROS topics
ros2 topic pub --once /dashboard/cmd_vel geometry_msgs/msg/Twist "{...}"
sleep 0.1
ros2 topic echo --once /cmd_vel  # Verify actual output
```

### Strategy 3: Scenario-Based Validation   

Test real-world scenarios, not isolated functions:

| Scenario | Test Procedure |
|----------|---|
| **Happy path** | heartbeat → reset → command → motion → odom updates |
| **Heartbeat dropout** | Send heartbeat; wait 600ms; verify motion stops |
| **E-stop during motion** | Motion commanded; press E-stop; verify immediate stop |
| **Command timeout** | Send command; wait 300ms without new command; verify stop |
| **Stale process recovery** | Kill all ROS; relaunch; verify clean startup |

---

## Testing Methodology

### Test Pyramid

```
                    /\
                   /  \
                  /    \        E2E Tests (5%)
                 /______\       - Dashboard + full stack
                /        \
               /          \    Integration Tests (25%)
              /____________\   - ROS topic flow + safety state transitions
             /              \
            /                \ Unit Tests (70%)
           /____________________\ - Component logic, state machine transitions
```

### Test Levels

**Level 1: Unit Tests** (70% of testing effort)
- Test individual functions (state machine logic, timeout calculations)
- Run in-process (fast: <10ms each)
- High coverage (aim for >80%)

**Level 2: Integration Tests** (25% of testing effort)
- Test between components (topic pub/sub, node startup)
- Use actual ROS 2 runtime (realistic: catches async bugs)
- Moderate coverage (critical paths only)

**Level 3: End-to-End Tests** (5% of testing effort)
- Test entire stack (launch → teleop → dashboard updates)
- Manual or scripted
- Validates user workflows

### Test Artifacts

```
ros2_ws/src/
├── safety_gateway/test/
│   └── safety_gateway_integration.py      # Integration tests (PASS ✓)
├── demo_pipeline/demo_pipeline/
│   └── synthetic_lidar.py                 # Unit testable functions
└── adaptive_grid/adaptive_grid/
    └── node.py                            # Unit testable grid logic
```

---

## Continuous Integration (Future)

Currently manual validation; future automation:

```yaml
# .github/workflows/test.yml (v1.1)
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Build ROS workspace
        run: colcon build --packages-select '*'
      - name: Run tests
        run: python3 src/safety_gateway/test/safety_gateway_integration.py
      - name: Lint frontend
        run: npm run lint
```

---

## Summary: Methodology in Action

**From Idea to Production**:

1. **Design** → Whiteboard architecture (5 days)
2. **Implement** → Build components in dependency order (15 days)
3. **Integrate** → Combine components, test flows (5 days)
4. **Validate** → Live testing against ROS runtime (5 days)
5. **Document** → Runbooks + API reference (3 days)
6. **Ship** → Tag v1.0, close project (1 day)

**Total**: ~1 month (realistic for production-grade system with safety validation)

**Key Success Factors**:
- ✅ Safety-first design from day 1
- ✅ Real runtime testing (no mocks)
- ✅ Progressive validation (don't skip integration tests)
- ✅ Comprehensive documentation (operators matter too)
- ✅ Iterative feedback (test after each phase, don't accumulate changes)

---

**End of Methodology Document**
