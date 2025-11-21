# Formation Control Monitoring Dashboard

Real-time web dashboard for multi-agent drone formation control simulations. Built with React + FastAPI + WebSocket for ROS2 integration.

**Thesis Project**: Formation Control of Agents Under Dynamic Meteorological Conditions

---

## IMPORTANT: Do Not Delete

When working with LLMs/AI assistants, **preserve these critical files**:

### Backend (Required)
- `backend/app.py` - FastAPI server, WebSocket handler, simulation control API
- `backend/ros_bridge/` - ROS2 subscriptions, schemas, metrics processing

### Frontend (Required)
- `frontend/src/App.tsx` - Main state management, WebSocket connection, history tracking
- `frontend/src/api/ws.ts` - WebSocket client, data types (MetricsData, OdometryData, etc.)
- `frontend/src/components/` - UI components (9 files):
  - `DashboardLayout.tsx` - Resizable panels, simulation start/stop controls
  - `SimulationMetricsPanel.tsx` - Main left panel with tabs (Overview, Map, Charts, etc.)
  - `FormationMap.tsx` - 2D drone position visualization with targets
  - `RosGraphPanel.tsx` - rqt_graph-style ROS2 topology visualization
  - `TimeSeries.tsx` - Real-time charts (error, RMSE, wind, etc.)
  - `ControllerParamsPanel.tsx` - PID/Fuzzy/Hybrid controller parameters display
  - `SystemStatePanel.tsx` - System state overview
  - `DiagnosticsPanel.tsx` - Bottom diagnostics wrapper
  - `EventLog.tsx` - Log display component

### Configuration (Required)
- `frontend/vite.config.ts` - Build config, dev proxy settings
- `frontend/package.json` - Dependencies
- `frontend/tailwind.config.js` - Styling

### Scripts (Useful)
- `scripts/run_backend.sh` - Start backend with ROS2 sourcing
- `scripts/check_health.sh` - Health diagnostics
- `scripts/test_publisher.py` - Test data publisher

---

## Architecture

```
monitoring_dashboard/
├── backend/
│   ├── app.py                 # FastAPI + WebSocket + Simulation Control
│   └── ros_bridge/
│       ├── subscriptions.py   # ROS2 topic auto-discovery
│       └── schemas.py         # Data models
├── frontend/
│   ├── src/
│   │   ├── App.tsx           # Main app, state, WebSocket
│   │   ├── api/ws.ts         # WebSocket client
│   │   └── components/       # UI components
│   ├── dist/                 # Production build (generated)
│   └── vite.config.ts
└── scripts/
```

## Quick Start

### 1. Build Frontend (once)
```bash
cd monitoring_dashboard/frontend
npm install
npm run build
```

### 2. Start Backend (serves UI at /ui)
```bash
cd monitoring_dashboard
source /opt/ros/humble/setup.bash
source ../../install/setup.bash  # or your workspace
./scripts/run_backend.sh
```

### 3. Open Dashboard
**http://localhost:8000/ui**

## Features

### Dashboard Layout
- **Resizable panels** - Drag borders to resize left/right columns
- **Simulation control** - Start/stop scripts from header dropdown
- **Connection status** - Live WebSocket indicator

### Left Panel (Tabs)
- **Overview** - System summary, controller comparison, error stats
- **Map** - 2D formation visualization (current + target positions)
- **Charts** - Time series (error, RMSE, wind velocity)
- **System** - Detailed system state
- **Params** - Controller parameters (PID gains, Fuzzy config)

### Right Panel
- **ROS Graph** - rqt_graph-style node/topic visualization
- **Zoom/Pan** - Interactive navigation
- **Filter** - Hide system topics

### Bottom Panel
- **Diagnostics** - Collapsible log panel

## Key Data Types

```typescript
// From frontend/src/api/ws.ts
MetricsData: {
  timestamp, error_x, error_y, error_z, error_magnitude,
  rmse_x, rmse_y, rmse_z, rmse_total,
  iae_x, iae_y, iae_z, itae_x, itae_y, itae_z,
  settling_time, overshoot, is_settled,
  target_x, target_y, target_z  // Actual targets
}

ControllerParams: {
  controller_type, // 'pid' | 'hybrid'
  kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_z, ki_z, kd_z,
  fuzzy_enabled, fuzzy_weight, pid_weight
}

OdometryData: { position: {x,y,z}, velocity: {x,y,z}, orientation: {x,y,z,w} }
WindData: { velocity: {x,y,z}, force: {x,y,z} }
FormationState: { shape, spacing, center_x, center_y, center_z, agent_count }
```

## API Endpoints

### REST
- `GET /api/status` - System status
- `GET /api/topics` - Available ROS2 topics
- `GET /api/agents` - Active agents
- `GET /api/snapshot` - Complete data snapshot
- `GET /api/simulation/status` - Simulation running state
- `GET /api/simulation/scripts` - Available launch scripts
- `POST /api/simulation/start?script=X` - Start simulation
- `POST /api/simulation/stop` - Stop simulation

### WebSocket (`/ws`)
Receives real-time updates: metrics, odom, wind, formation, ros_graph, controller_params

## Monitored ROS2 Topics

Per-agent:
- `/agent_X/odom` - Position/velocity
- `/agent_X/metrics` - Performance metrics
- `/agent_X/controller_params` - Controller config
- `/agent_X/target_pose` - Target position

Global:
- `/wind/velocity` - Wind velocity
- `/formation_coordinator_node/state` - Formation state

## Development

### Frontend Dev (with hot reload)
```bash
cd frontend
npm run dev  # Runs on :3000, proxies API to :8000
```

### Backend Dev
```bash
cd backend
source /opt/ros/humble/setup.bash
source install/setup.bash
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

## Troubleshooting

### No Data
1. Verify simulation running: `ros2 topic list`
2. Check backend logs for subscriptions
3. Verify WebSocket connected (header indicator)

### Charts Empty
- Charts require history data - wait a few seconds after connection
- Verify `metricsHistory` is populated in App.tsx

### Map Shows Wrong Targets
- Map uses `metricsData.target_x/y` (controller targets)
- Falls back to `targetData` if metrics unavailable

---

## Thesis Results & Figures Guide

### Recommended Dashboard Screenshots for Thesis

1. **Formation Map (Map Tab)**
   - Show agents converging to formation (current positions → targets)
   - Capture with wind arrow visible showing disturbance direction
   - Compare: initial scattered state vs. final formation achieved

2. **Controller Comparison (Overview Tab)**
   - PID vs Hybrid (PID+Fuzzy) error comparison table
   - Shows which controller type each agent uses
   - Average RMSE, settling time differences

3. **Time Series Charts (Charts Tab)**
   - **Error Plot**: Position error convergence over time (X, Y, Z axes)
   - **RMSE Plot**: Cumulative accuracy comparison between agents
   - **Wind Plot**: Wind disturbance profile during experiment
   - **Overshoot Plot**: Maximum overshoot comparison

4. **ROS2 Graph (Right Panel)**
   - System architecture visualization
   - Shows node-topic-node connections
   - Demonstrates multi-agent communication topology

5. **Controller Parameters (Params Tab)**
   - PID gains (Kp, Ki, Kd) for each axis
   - Fuzzy weight vs PID weight ratio
   - Feed-forward compensation settings

### Key Metrics to Report

| Metric | Description | Compare |
|--------|-------------|---------|
| RMSE | Root Mean Square Error | PID vs Hybrid |
| IAE | Integral Absolute Error | Lower = better tracking |
| ITAE | Integral Time-weighted AE | Penalizes slow convergence |
| Settling Time | Time to reach target | Faster = better response |
| Overshoot | Max error after approach | Lower = smoother control |

### Suggested Experiments

1. **No Wind**: Baseline formation convergence
2. **Constant Wind**: Steady disturbance rejection
3. **Variable Wind**: Sinusoidal/gust response
4. **Formation Change**: Triangle → Line → Square transitions

---

**Built for thesis: Formation Control of Agents Under Dynamic Meteorological Conditions**
