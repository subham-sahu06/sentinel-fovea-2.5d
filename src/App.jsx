import React, { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import {
  Activity,
  Crosshair,
  Layers,
  LocateFixed,
  Radio,
  ShieldAlert,
  ShieldCheck,
  Square,
  Zap,
  AlertTriangle,
} from 'lucide-react';
import { parsePointCloud, SEMANTIC_CLASSES } from './utils/pointcloud.js';
import { projectWorldPoint } from './utils/projection.js';
import './App.css';

const ROS = ROSLIB;
const COMMAND_TIMEOUT_MS = 250;

// Synthetic fallback points for offline mode
const fallbackPoints = Array.from({ length: 600 }, (_, index) => {
  const angle = (index / 600) * Math.PI * 2;
  const radius = 2.5 + (index % 15) * 0.4;
  const isTrench = index > 120 && index < 150;
  const isDynamic = index > 280 && index < 310;
  const isStatic = index % 11 === 0;
  const classId = isTrench ? 1 : isDynamic ? 3 : isStatic ? 2 : 0;
  const z = isTrench ? -0.35 : isDynamic ? 0.9 : isStatic ? 0.8 : 0.02;
  return {
    x: Math.cos(angle) * radius,
    y: Math.sin(angle) * radius,
    z,
    class_id: classId,
    confidence: 0.92,
    intensity: 60,
    obstacle: classId !== 0,
  };
});

function PointCloudView({ points, occupancy, elevationMarkers, connected, velocityRef, viewMode }) {
  const canvasRef = useRef(null);
  const pointsRef = useRef(points);
  const occupancyRef = useRef(occupancy);
  const elevationMarkersRef = useRef(elevationMarkers);
  const viewModeRef = useRef(viewMode);

  useEffect(() => { pointsRef.current = points; }, [points]);
  useEffect(() => { occupancyRef.current = occupancy; }, [occupancy]);
  useEffect(() => { elevationMarkersRef.current = elevationMarkers; }, [elevationMarkers]);
  useEffect(() => { viewModeRef.current = viewMode; }, [viewMode]);

  useEffect(() => {
    const canvas = canvasRef.current;
    const context = canvas.getContext('2d');
    let frame;
    let lastTime = performance.now();
    const pose = { x: 0, y: 0, yaw: 0 };

    const draw = (time) => {
      const delta = Math.min((time - lastTime) / 1000, 0.1);
      lastTime = time;
      const velocity = velocityRef.current;
      pose.x += velocity.linear * Math.cos(pose.yaw) * delta;
      pose.y += velocity.linear * Math.sin(pose.yaw) * delta;
      pose.yaw += velocity.angular * delta;

      const { width, height } = canvas.getBoundingClientRect();
      const ratio = window.devicePixelRatio || 1;
      if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
        canvas.width = width * ratio;
        canvas.height = height * ratio;
      }
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.fillStyle = '#0c1214';
      context.fillRect(0, 0, width, height);

      const mode = viewModeRef.current;

      // 1. Draw Perspective Grid & Concentric Fovea Ground Rings
      context.strokeStyle = 'rgba(173, 194, 185, 0.10)';
      context.lineWidth = 1;
      for (let line = -6; line <= 6; line += 1) {
        const offset = line * 48;
        context.beginPath();
        context.moveTo(width / 2 + offset, height * 0.58);
        context.lineTo(width / 2 + offset * 0.52, height);
        context.stroke();
      }

      // Fovea Range Rings (10m Micro, 30m Meso, 60m Macro)
      const foveaRings = [
        { radius: 10, label: 'MICRO FOVEA (5cm)', color: 'rgba(52, 211, 153, 0.45)' },
        { radius: 25, label: 'MESO TRACK (15cm)', color: 'rgba(233, 185, 73, 0.35)' },
        { radius: 50, label: 'MACRO HORIZON (50cm)', color: 'rgba(148, 163, 184, 0.25)' },
      ];

      foveaRings.forEach((ring) => {
        context.strokeStyle = ring.color;
        context.lineWidth = mode === 'fovea' ? 2 : 1;
        context.setLineDash(mode === 'fovea' ? [4, 4] : [2, 6]);
        context.beginPath();
        for (let a = 0; a <= Math.PI * 2; a += 0.08) {
          const rx = Math.cos(a) * ring.radius;
          const ry = Math.sin(a) * ring.radius;
          const p = projectWorldPoint(rx, ry, 0, pose, width, height);
          if (a === 0) context.moveTo(p.x, p.y);
          else context.lineTo(p.x, p.y);
        }
        context.closePath();
        context.stroke();
        context.setLineDash([]);
      });

      // 2. Draw 2.5D Elevation & Semantic Markers
      elevationMarkersRef.current.forEach((marker) => {
        const mx = marker.pose?.position?.x || 0;
        const my = marker.pose?.position?.y || 0;
        const mz = marker.pose?.position?.z || 0;
        const projected = projectWorldPoint(mx, my, mz, pose, width, height);
        const markerWidth = Math.max(3, (marker.scale?.x || 0.15) * projected.depth * 32);
        const markerHeight = Math.max(4, (marker.scale?.z || 0.15) * projected.depth * 45);

        const r = Math.round((marker.color?.r || 0.2) * 255);
        const g = Math.round((marker.color?.g || 0.8) * 255);
        const b = Math.round((marker.color?.b || 0.4) * 255);
        const a = marker.color?.a || 0.6;

        context.fillStyle = `rgba(${r}, ${g}, ${b}, ${a})`;
        context.fillRect(projected.x - markerWidth / 2, projected.y - markerHeight, markerWidth, markerHeight);
      });

      // 3. Draw Point Cloud with Mode-Specific Shading
      pointsRef.current.forEach((point) => {
        const projected = projectWorldPoint(point.x, point.y, point.z, pose, width, height);
        let color = 'rgba(52, 211, 153, 0.75)'; // default drivable
        let size = 1.8;

        if (mode === 'semantic') {
          const sem = SEMANTIC_CLASSES[point.class_id] || SEMANTIC_CLASSES[0];
          color = sem.color;
          size = point.class_id === 3 ? 3.4 : point.class_id === 1 ? 3.0 : point.class_id === 2 ? 2.6 : 1.8;
        } else if (mode === 'elevation') {
          // Heatmap from ground (cool teal) to high (warm orange/red)
          const normZ = Math.max(-0.5, Math.min(2.0, point.z));
          if (normZ < 0.0) {
            color = 'rgba(251, 146, 60, 0.9)'; // Negative / Trench
          } else if (normZ < 0.2) {
            color = 'rgba(52, 211, 153, 0.8)';
          } else if (normZ < 0.8) {
            color = 'rgba(233, 185, 73, 0.9)';
          } else {
            color = 'rgba(248, 113, 113, 0.95)';
          }
          size = 2.2;
        } else if (mode === 'traversability') {
          // Traversability tau: Green (1.0) -> Red (0.0)
          if (point.class_id === 0) {
            color = 'rgba(52, 211, 153, 0.85)'; // Traversable
          } else if (point.class_id === 3) {
            color = 'rgba(233, 185, 73, 0.9)';  // Dynamic Warning
          } else {
            color = 'rgba(239, 68, 68, 0.95)';   // Blocked
          }
          size = 2.4;
        } else if (mode === 'fovea') {
          const dist = Math.hypot(point.x - pose.x, point.y - pose.y);
          if (dist <= 10.0) {
            color = 'rgba(52, 211, 153, 0.95)'; // 5cm Zone
            size = 2.6;
          } else if (dist <= 30.0) {
            color = 'rgba(233, 185, 73, 0.85)'; // 15cm Zone
            size = 2.0;
          } else {
            color = 'rgba(148, 163, 184, 0.70)'; // 50cm Zone
            size = 1.6;
          }
        }

        context.fillStyle = color;
        context.fillRect(projected.x - size / 2, projected.y - size / 2, size, size);

        // Draw bounding brackets around Dynamic Targets
        if (point.class_id === 3 && mode === 'semantic') {
          context.strokeStyle = '#38bdf8';
          context.lineWidth = 1;
          context.strokeRect(projected.x - 5, projected.y - 8, 10, 14);
        }
      });

      // 4. Ego Robot Icon at Origin
      context.strokeStyle = '#f5ba49';
      context.fillStyle = 'rgba(245, 186, 73, 0.25)';
      context.lineWidth = 2;
      context.beginPath();
      context.moveTo(width / 2, height * 0.58 - 11);
      context.lineTo(width / 2 - 9, height * 0.58 + 9);
      context.lineTo(width / 2 + 9, height * 0.58 + 9);
      context.closePath();
      context.fill();
      context.stroke();

      frame = requestAnimationFrame(draw);
    };

    frame = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(frame);
  }, [velocityRef]);

  return (
    <div className="viewport">
      <canvas ref={canvasRef} />
      <div className="viewport-label">
        <Crosshair size={14} /> {connected ? 'LIVE /semantic_points (FOVEATED 2.5D)' : 'SIMULATION MODE'}
      </div>

      <div className="semantic-legend">
        <div className="legend-item">
          <span className="legend-dot" style={{ background: '#34d399' }} /> DRIVABLE
        </div>
        <div className="legend-item">
          <span className="legend-dot" style={{ background: '#fb923c' }} /> TRENCH / POTHOLE
        </div>
        <div className="legend-item">
          <span className="legend-dot" style={{ background: '#f87171' }} /> STATIC OBSTACLE
        </div>
        <div className="legend-item">
          <span className="legend-dot" style={{ background: '#38bdf8' }} /> DYNAMIC TARGET
        </div>
      </div>

      <div className="axis">
        <span>Y</span>
        <span>X</span>
        <span>Z</span>
      </div>
    </div>
  );
}

export default function App() {
  const [connected, setConnected] = useState(false);
  const [emergencyStopped, setEmergencyStopped] = useState(true);
  const [gatewayStatus, setGatewayStatus] = useState('OFFLINE');
  const [points, setPoints] = useState(fallbackPoints);
  const [occupancy, setOccupancy] = useState(null);
  const [elevationMarkers, setElevationMarkers] = useState([]);
  const [viewMode, setViewMode] = useState('semantic'); // 'semantic' | 'elevation' | 'traversability' | 'fovea'
  const [showBaseline, setShowBaseline] = useState(false); // Toggle between baseline and foveated metrics
  const [metrics, setMetrics] = useState({
    uniform_5cm_memory_mb: 64.0,
    foveated_2_5d_memory_mb: 5.24,
    memory_savings_percent: 91.8,
    compression_ratio: '12.3x',
    active_cells_count: 3280,
    grid_fps: 32.5,
    grid_latency_ms: 8.4,
  });
  const [semanticStats, setSemanticStats] = useState({
    fps: 34.0,
    inference_latency_ms: 7.8,
    class_distribution: { drivable: 11450, negative_trench: 140, static_obstacle: 850, dynamic_target: 1550 },
  });
  const [telemetry, setTelemetry] = useState({ commandLinear: 0, commandAngular: 0, actualLinear: 0, actualAngular: 0 });

  const rosRef = useRef(null);
  const cmdVelRef = useRef(null);
  const heartbeatRef = useRef(null);
  const estopRef = useRef(null);
  const velocityRef = useRef({ linear: 0, angular: 0 });
  const connectedRef = useRef(false);
  const emergencyRef = useRef(true);
  const deadmanRef = useRef(false);
  const lastCommandAtRef = useRef(0);

  const publishVelocity = (linear, angular) => {
    if (!cmdVelRef.current || !connectedRef.current) return;
    cmdVelRef.current.publish({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });
  };

  const stopMotion = () => {
    deadmanRef.current = false;
    velocityRef.current = { linear: 0, angular: 0 };
    lastCommandAtRef.current = 0;
    publishVelocity(0, 0);
    setTelemetry((c) => ({ ...c, commandLinear: 0, commandAngular: 0 }));
  };

  const setMovement = (linear, angular) => {
    if ((linear || angular) && (emergencyRef.current || !deadmanRef.current)) return;
    velocityRef.current = { linear, angular };
    lastCommandAtRef.current = linear || angular ? performance.now() : 0;
    setTelemetry((c) => ({ ...c, commandLinear: linear, commandAngular: angular }));
    publishVelocity(linear, angular);
  };

  const publishEmergency = (latched) => {
    if (estopRef.current && connectedRef.current) {
      estopRef.current.publish({ data: latched });
    }
  };

  const triggerEmergencyStop = () => {
    emergencyRef.current = true;
    setEmergencyStopped(true);
    stopMotion();
    publishEmergency(true);
  };

  const resetEmergencyStop = () => {
    emergencyRef.current = false;
    setEmergencyStopped(false);
    stopMotion();
    publishEmergency(false);
  };

  useEffect(() => {
    const ros = new ROS.Ros({ url: 'ws://localhost:9090' });
    const cmdVel = new ROS.Topic({ ros, name: '/dashboard/cmd_vel', messageType: 'geometry_msgs/Twist' });
    const heartbeat = new ROS.Topic({ ros, name: '/dashboard/heartbeat', messageType: 'std_msgs/Empty' });
    const estop = new ROS.Topic({ ros, name: '/dashboard/emergency_stop', messageType: 'std_msgs/Bool' });
    const gateway = new ROS.Topic({ ros, name: '/safety_gateway/status', messageType: 'std_msgs/String' });
    const occupancyTopic = new ROS.Topic({ ros, name: '/adaptive_grid/occupancy', messageType: 'nav_msgs/OccupancyGrid' });
    const elevationTopic = new ROS.Topic({ ros, name: '/adaptive_grid/elevation_markers', messageType: 'visualization_msgs/MarkerArray' });
    const metricsTopic = new ROS.Topic({ ros, name: '/adaptive_grid/metrics', messageType: 'std_msgs/String' });
    const statsTopic = new ROS.Topic({ ros, name: '/semantic/stats', messageType: 'std_msgs/String' });
    const odometry = new ROS.Topic({ ros, name: '/odom', messageType: 'nav_msgs/Odometry' });
    const semanticCloud = new ROS.Topic({ ros, name: '/semantic_points', messageType: 'sensor_msgs/PointCloud2' });

    ros.on('connection', () => {
      connectedRef.current = true;
      setConnected(true);
      setGatewayStatus('OFFLINE');
      publishEmergency(emergencyRef.current);
    });
    ros.on('error', () => {
      connectedRef.current = false;
      setConnected(false);
      setGatewayStatus('OFFLINE');
      stopMotion();
    });
    ros.on('close', () => {
      connectedRef.current = false;
      setConnected(false);
      setGatewayStatus('OFFLINE');
      stopMotion();
    });

    rosRef.current = ros;
    cmdVelRef.current = cmdVel;
    heartbeatRef.current = heartbeat;
    estopRef.current = estop;

    semanticCloud.subscribe((message) => {
      const parsed = parsePointCloud(message);
      if (parsed.length) setPoints(parsed);
    });

    occupancyTopic.subscribe((message) => setOccupancy(message));
    elevationTopic.subscribe((message) => setElevationMarkers(message.markers || []));

    metricsTopic.subscribe((message) => {
      try {
        setMetrics(JSON.parse(message.data));
      } catch {}
    });

    statsTopic.subscribe((message) => {
      try {
        setSemanticStats(JSON.parse(message.data));
      } catch {}
    });

    gateway.subscribe((message) => {
      try {
        const status = JSON.parse(message.data);
        setGatewayStatus(status.state || 'OFFLINE');
        if (status.state === 'ESTOP_LATCHED') {
          emergencyRef.current = true;
          setEmergencyStopped(true);
        }
      } catch {
        setGatewayStatus('INVALID STATUS');
      }
    });

    odometry.subscribe((message) => {
      setTelemetry((c) => ({
        ...c,
        actualLinear: message.twist?.twist?.linear?.x || 0,
        actualAngular: message.twist?.twist?.angular?.z || 0,
      }));
    });

    const interval = setInterval(() => {
      if (!connectedRef.current) return;
      heartbeatRef.current?.publish({});
      if (!deadmanRef.current) return;
      const now = performance.now();
      if (now - lastCommandAtRef.current > COMMAND_TIMEOUT_MS) {
        stopMotion();
        return;
      }
      publishVelocity(velocityRef.current.linear, velocityRef.current.angular);
      lastCommandAtRef.current = now;
    }, 50);

    const controlKeys = ['arrowup', 'arrowdown', 'arrowleft', 'arrowright', 'w', 'a', 's', 'd'];
    const handleKeyDown = (event) => {
      if (event.repeat) return;
      const key = event.key.toLowerCase();
      if (controlKeys.includes(key)) {
        event.preventDefault();
        deadmanRef.current = true;
        if (key === 'arrowup' || key === 'w') setMovement(1, 0);
        else if (key === 'arrowdown' || key === 's') setMovement(-1, 0);
        else if (key === 'arrowleft' || key === 'a') setMovement(0, 1.5);
        else if (key === 'arrowright' || key === 'd') setMovement(0, -1.5);
      } else if (key === ' ') {
        event.preventDefault();
        triggerEmergencyStop();
      }
    };

    const handleKeyUp = (event) => {
      if (controlKeys.includes(event.key.toLowerCase())) stopMotion();
    };

    const handleBlur = () => stopMotion();

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', handleBlur);

    return () => {
      clearInterval(interval);
      stopMotion();
      semanticCloud.unsubscribe();
      occupancyTopic.unsubscribe();
      elevationTopic.unsubscribe();
      metricsTopic.unsubscribe();
      statsTopic.unsubscribe();
      gateway.unsubscribe();
      odometry.unsubscribe();
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', handleBlur);
      ros.close();
    };
  }, []);

  const drive = (linear, angular) => ({
    onPointerDown: () => {
      deadmanRef.current = true;
      setMovement(linear, angular);
    },
    onPointerUp: stopMotion,
    onPointerCancel: stopMotion,
    onPointerLeave: stopMotion,
  });

  const moving = Math.abs(telemetry.actualLinear) > 0.01 || Math.abs(telemetry.actualAngular) > 0.01;
  const gatewayReady = gatewayStatus === 'ACTIVE' || gatewayStatus === 'READY';
  const controlsDisabled = emergencyStopped || (connected && !gatewayReady);

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark">
            <Activity size={18} />
          </div>
          <div>
            <h1>SENTINEL-FOVEA 2.5D</h1>
            <p>DRDO Autonomous Tactical Perception & Control Console</p>
          </div>
        </div>
        <div className={`connection ${connected ? 'online' : ''}`}>
          <Radio size={15} /> {connected ? 'ROS 2 ONLINE (30+ FPS)' : 'ROS 2 OFFLINE (SIM)'}
          <span>ws://localhost:9090</span>
        </div>
      </header>

      <section className="workspace">
        {/* Live Efficiency & Memory Savings Bar */}
        <div className="efficiency-bar">
          <div className="scenario-toggle">
            <label className="toggle-label">
              <input
                type="checkbox"
                checked={showBaseline}
                onChange={(e) => setShowBaseline(e.target.checked)}
              />
              <span className="toggle-slider"></span>
              <span className="toggle-text">
                {showBaseline ? 'BASELINE (Uniform Grid)' : 'FOVEATED 2.5D'}
              </span>
            </label>
          </div>
          <div className="metric-card">
            <span className="metric-label">Uniform 5cm Grid</span>
            <div className="metric-val">
              {metrics.uniform_5cm_memory_mb.toFixed(1)} <small>MB (4.0M cells)</small>
            </div>
          </div>
          <div className="metric-card">
            <span className="metric-label">
              {showBaseline ? 'Uniform 5cm Grid (Baseline)' : 'Foveated 2.5D Grid'}
            </span>
            <div className="metric-val" style={{ color: showBaseline ? '#6b7280' : '#34d399' }}>
              {showBaseline
                ? metrics.uniform_5cm_memory_mb.toFixed(1)
                : metrics.foveated_2_5d_memory_mb.toFixed(2)
              } <small>MB
                {showBaseline
                  ? '(4.0M cells)'
                  : `(${metrics.active_cells_count} cells)`
                }
              </small>
            </div>
          </div>
          <div className="metric-card">
            <span className="metric-label">Memory Bandwidth Reduction</span>
            <div className="metric-val">
              <span className="savings-badge">
                <Zap size={13} /> {showBaseline ? '0%' : metrics.memory_savings_percent}% SAVED
              </span>
              <span className="compression-tag">
                {showBaseline ? '1x' : metrics.compression_ratio}
              </span>
            </div>
          </div>
          <div className="metric-card">
            <span className="metric-label">Neural Pipeline Throughput</span>
            <div className="metric-val" style={{ color: '#e9b949' }}>
              {semanticStats.fps.toFixed(1)} <small>FPS ({semanticStats.inference_latency_ms.toFixed(1)} ms latency)</small>
            </div>
          </div>
        </div>

        {/* Tactical Threat Alert Banner */}
        <div className="threat-banner">
          <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
            <AlertTriangle size={15} color="#fb923c" />
            <span>
              <strong>TACTICAL PERCEPTION ACTIVE:</strong> Negative Trench detected at <strong>6.0m</strong> (τ = 0.0) | Dynamic Target Patrol Vehicle at <strong>18.5m</strong> (2.5 m/s)
            </span>
          </div>
          <span className="threat-badge">4 CLASSES SEGMENTED</span>
        </div>

        {/* Section Heading & View Layer Switcher */}
        <div className="section-heading">
          <div>
            <p className="eyebrow">01 / ADAPTIVE SCENE PERCEPTION</p>
            <h2>Hierarchical Foveated 2.5D Spatial Mesh</h2>
          </div>
          <div className="layer-switcher">
            <button
              className={`layer-btn ${viewMode === 'semantic' ? 'active' : ''}`}
              onClick={() => setViewMode('semantic')}
            >
              <Layers size={13} style={{ verticalAlign: 'middle', marginRight: 4 }} /> SEMANTIC (4-CLASS)
            </button>
            <button
              className={`layer-btn ${viewMode === 'elevation' ? 'active' : ''}`}
              onClick={() => setViewMode('elevation')}
            >
              ELEVATION MESH
            </button>
            <button
              className={`layer-btn ${viewMode === 'traversability' ? 'active' : ''}`}
              onClick={() => setViewMode('traversability')}
            >
              TRAVERSABILITY (τ)
            </button>
            <button
              className={`layer-btn ${viewMode === 'fovea' ? 'active' : ''}`}
              onClick={() => setViewMode('fovea')}
            >
              FOVEA RINGS (5/15/50cm)
            </button>
          </div>
          <div className="rate">
            <span className="pulse" />
            <strong>{points.length.toLocaleString()}</strong> <span className="muted">points rendered</span>
          </div>
        </div>

        <div className="scene-layout">
          <PointCloudView
            points={points}
            occupancy={occupancy}
            elevationMarkers={elevationMarkers}
            connected={connected}
            velocityRef={velocityRef}
            viewMode={viewMode}
          />

          <aside className="control-panel">
            <div>
              <div className="panel-title">
                <LocateFixed size={17} />
                <span>Tactical Teleoperation</span>
                <small>{connected ? `GATEWAY ${gatewayStatus}` : 'HOLD TO DRIVE'}</small>
              </div>

              <div className="readouts">
                <div>
                  <span>ACTUAL LINEAR</span>
                  <strong>
                    {telemetry.actualLinear.toFixed(2)}
                    <em>m/s</em>
                  </strong>
                </div>
                <div>
                  <span>ACTUAL ANGULAR</span>
                  <strong>
                    {telemetry.actualAngular.toFixed(2)}
                    <em>rad/s</em>
                  </strong>
                </div>
              </div>

              <div className="command-readout">
                COMMAND {telemetry.commandLinear.toFixed(1)} m/s / {telemetry.commandAngular.toFixed(1)} rad/s
              </div>

              <div className="dpad">
                <button aria-label="Forward" disabled={controlsDisabled} {...drive(1, 0)}>
                  ↑
                </button>
                <button aria-label="Turn left" disabled={controlsDisabled} {...drive(0, 1.5)}>
                  ←
                </button>
                <button className="stop" aria-label="Stop" onClick={stopMotion}>
                  <Square size={16} fill="currentColor" />
                </button>
                <button aria-label="Turn right" disabled={controlsDisabled} {...drive(0, -1.5)}>
                  →
                </button>
                <button aria-label="Reverse" disabled={controlsDisabled} {...drive(-1, 0)}>
                  ↓
                </button>
              </div>
            </div>

            <div>
              <button
                className={`emergency-button ${emergencyStopped ? 'latched' : ''}`}
                onClick={emergencyStopped ? resetEmergencyStop : triggerEmergencyStop}
              >
                {emergencyStopped ? <ShieldCheck size={17} /> : <ShieldAlert size={17} />}
                {emergencyStopped ? 'RESET EMERGENCY STOP' : 'EMERGENCY STOP'}
              </button>

              <div className="state">
                <span className={moving ? 'active-dot' : ''} /> {emergencyStopped ? 'ESTOP LATCHED' : moving ? 'ACTIVE DRIVING' : 'STANDBY IDLE'}
                <small>{connected ? `GATEWAY ${gatewayStatus}` : 'SIMULATION ONLY'}</small>
              </div>
            </div>
          </aside>
        </div>
      </section>

      <footer>
        <span>FRAME <b>base_link</b></span>
        <span>TOPIC <b>/semantic_points</b></span>
        <span>FOVEA ZONES <b className="teal">5cm (0-10m) | 15cm (10-30m) | 50cm (30-100m)</b></span>
        <span>MEMORY REDUCTION <b className="teal">{metrics.memory_savings_percent}% ({metrics.compression_ratio})</b></span>
        <span>TRAVERSABILITY <b className="orange">NEGATIVE TRENCH DETECTED</b></span>
      </footer>
    </main>
  );
}
