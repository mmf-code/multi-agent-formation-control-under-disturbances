# 🚁 Multi-Agent Formation Control - Real-time Monitoring Dashboard

Modern, real-time web-based monitoring dashboard for multi-agent drone formation control simulations. Built with React, FastAPI, and WebSocket for seamless ROS2 integration.

![Dashboard Preview](https://via.placeholder.com/800x400?text=Formation+Control+Monitor)

## ✨ Features

### 📊 Real-time Monitoring
- **Live Drone Tracking**: 2D formation map with current & target positions
- **Performance Metrics**: Error, RMSE, IAE, ITAE, settling time, overshoot
- **Wind Monitoring**: Real-time wind velocity and force tracking
- **Formation State**: Dynamic formation shape, spacing, and center tracking

### 🎯 Interactive Interface
- **Topic Selection**: Choose which ROS2 topics to monitor
- **Agent Filtering**: Focus on specific agents or view all
- **Dynamic Charts**: Real-time Plotly.js charts with multiple views
- **Event Log**: System events and status updates

### 📈 Visualizations
1. **KPI Cards**: Key performance indicators for each agent
2. **Formation Map**: 2D visualization of drone positions
3. **Time Series Charts**:
   - Position error (X, Y, Z)
   - IAE/ITAE trends
   - Wind velocity
   - Overshoot & RMSE
4. **Event Log**: Real-time system messages

## 🏗️ Architecture

```
monitoring_dashboard/
├── backend/                    # FastAPI + ROS2 Bridge
│   ├── app.py                  # Main FastAPI application
│   └── ros_bridge/             # ROS2 integration
│       ├── __init__.py
│       ├── subscriptions.py    # Auto-discovery & topic management
│       ├── schemas.py          # Pydantic data models
│       └── metrics.py
├── frontend/                   # React + TypeScript
│   ├── src/
│   │   ├── App.tsx             # Main application
│   │   ├── api/
│   │   │   └── ws.ts           # WebSocket client
│   │   └── components/
│   │       ├── KpiCards.tsx    # Performance metrics cards
│   │       ├── TopicStatus.tsx # Topic/agent selection
│   │       ├── TimeSeries.tsx  # Real-time charts
│   │       ├── FormationMap.tsx# 2D drone visualization
│   │       └── EventLog.tsx    # Event logging
│   ├── package.json
│   └── index.html
├── scripts/                    # Utility scripts
│   ├── setup.sh                # One-time setup
│   ├── run_backend.sh          # Start backend server
│   └── run_frontend.sh         # Start frontend dev server
└── README.md
```

## 🚀 Quick Start

### Prerequisites

- **ROS2** (Humble/Iron/Jazzy)
- **Python 3.8+**
- **Node.js 18+** and npm
- Active ROS2 workspace with simulation running

### 1. Setup (First Time Only)

```bash
cd monitoring_dashboard
./scripts/setup.sh
```

This will:
- Install Python dependencies (FastAPI, uvicorn, websockets, pydantic)
- Install Node.js dependencies (React, Plotly.js, etc.)
- Make scripts executable

### 2. Start the Simulation

In a new terminal, start your formation control simulation:

```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch simulation (example)
ros2 launch agent_control_pkg multi_agent_formation.launch.py
```

### 3. Start the Backend

In a new terminal:

```bash
cd monitoring_dashboard
./scripts/run_backend.sh
```

The backend will:
- Auto-discover active ROS2 topics
- Subscribe to metrics, odometry, wind, formation topics
- Serve WebSocket API on `ws://localhost:8000/ws`
- Serve REST API on `http://localhost:8000`

### 4. Start the Frontend

In another terminal:

```bash
cd monitoring_dashboard
./scripts/run_frontend.sh
```

### 5. Open Dashboard

Open your browser to **http://localhost:3000**

You should see the dashboard with live data streaming!

## 📡 Monitored ROS2 Topics

### Per-Agent Topics
The dashboard automatically discovers and subscribes to:

- `/agent_X/odom` (nav_msgs/Odometry) - Position, velocity, orientation
- `/agent_X/target_pose` (geometry_msgs/PoseStamped) - Target position
- `/agent_X/metrics` (my_custom_interfaces_pkg/MetricsData) - Performance metrics
- `/agent_X/diagnostics` (std_msgs/Float64MultiArray) - Controller contributions

### Global Topics
- `/wind/velocity` (geometry_msgs/Vector3) - Wind velocity
- `/wind/force` (geometry_msgs/Vector3) - Wind force/bias
- `/formation_coordinator_node/state` (my_custom_interfaces_pkg/FormationState) - Formation state

## 🎮 Usage Guide

### Selecting Topics

Use the **Topic Selection** panel to enable/disable data streams:
- Click topic buttons to toggle subscription
- Active topics show a green pulse indicator
- Disabled topics appear grayed out

### Filtering Agents

Click on agent buttons to focus on specific drones:
- Selected agents are highlighted in green
- Charts and maps update to show only selected agents
- Clear all selections to view all agents

### Chart Views

Switch between different chart types:
- **Position Error**: X, Y, Z tracking errors
- **IAE/ITAE**: Integral performance metrics
- **Wind Velocity**: Wind environment conditions
- **Overshoot & RMSE**: Response characteristics

### Formation Map

The 2D map shows:
- **Blue circles**: Current drone positions
- **Green X**: Target positions
- **Dotted lines**: Error vectors (current → target)
- **Orange diamond**: Formation center

## 🔧 Configuration

### Backend Configuration

Edit `backend/app.py` to configure:
- WebSocket port (default: 8000)
- CORS settings
- Data buffer sizes
- Update rates

### Frontend Configuration

Edit `frontend/src/api/ws.ts` to change:
- WebSocket URL (default: ws://localhost:8000/ws)
- Reconnection interval
- Data history limits

## 📊 Performance Metrics Explained

| Metric | Description | Unit |
|--------|-------------|------|
| **Error** | Instantaneous position error magnitude | meters |
| **RMSE** | Root Mean Square Error (cumulative accuracy) | meters |
| **IAE** | Integral Absolute Error (total error accumulation) | m·s |
| **ITAE** | Integral Time-weighted Absolute Error (penalizes late errors) | m·s² |
| **Settling Time** | Time to reach and stay within threshold | seconds |
| **Overshoot** | Maximum error after initial approach | meters |
| **Settled** | Whether error is within threshold for duration | boolean |

## 🐛 Troubleshooting

### Backend Not Starting

**Problem**: ROS2 topics not discovered

**Solution**:
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify topics are publishing
ros2 topic list
ros2 topic echo /agent_0/metrics
```

### Frontend Not Connecting

**Problem**: WebSocket connection failed

**Solution**:
1. Check backend is running: `curl http://localhost:8000`
2. Check browser console for errors
3. Verify WebSocket URL in `frontend/src/api/ws.ts`

### No Data Appearing

**Problem**: Dashboard shows "No data available"

**Solution**:
1. Ensure simulation is running and publishing data
2. Check topic selection (topics must be enabled)
3. Verify ROS2 topics: `ros2 topic hz /agent_0/metrics`

### Charts Not Updating

**Problem**: Charts frozen or not updating

**Solution**:
1. Check WebSocket connection status (top-right indicator)
2. Refresh browser (Ctrl+R)
3. Check browser console for errors

## 🔌 API Reference

### REST Endpoints

```http
GET /                           # API info
GET /api/status                 # System status
GET /api/topics                 # Available ROS2 topics
GET /api/agents                 # Active agents
GET /api/snapshot               # Complete data snapshot
GET /api/metrics/{agent_id}     # Metrics history
GET /api/odom/{agent_id}        # Odometry history
GET /api/wind                   # Wind history
GET /api/formation              # Formation state
```

### WebSocket Messages

**Subscribe to topics:**
```json
{
  "action": "subscribe",
  "topics": ["metrics", "odom", "wind"],
  "agents": ["agent_0", "agent_1"]
}
```

**Receive updates:**
```json
{
  "type": "update",
  "data_type": "metrics",
  "agent_id": "agent_0",
  "data": { ... }
}
```

## 🛠️ Development

### Backend Development

```bash
# Install dev dependencies
pip3 install black flake8 mypy

# Run with auto-reload
cd backend
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

### Frontend Development

```bash
cd frontend

# Install dependencies
npm install

# Start dev server with hot reload
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

## 📝 Adding Custom Metrics

### 1. Backend (ROS2 → WebSocket)

Add new subscription in `backend/ros_bridge/subscriptions.py`:

```python
def subscribe_custom_topic(self, agent_id: str):
    topic = f'/{agent_id}/custom_topic'
    sub = self.create_subscription(
        CustomMsg,
        topic,
        lambda msg: self._on_custom(agent_id, msg),
        10
    )
    self.subscriptions[topic] = sub
```

### 2. Frontend (Display)

Create component in `frontend/src/components/CustomView.tsx`:

```tsx
export const CustomView: React.FC<{data: CustomData}> = ({data}) => {
  return <div>{/* Render custom data */}</div>;
};
```

Add to `App.tsx`:

```tsx
import { CustomView } from './components/CustomView';
// ... in render:
<CustomView data={customData} />
```

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This monitoring dashboard is part of the Multi-Agent Formation Control project.

## 🙏 Acknowledgments

- **ROS2** for robotics framework
- **FastAPI** for high-performance backend
- **React** + **Plotly.js** for interactive UI
- **Tailwind CSS** for modern styling

## 📧 Support

For issues or questions:
1. Check troubleshooting section above
2. Review ROS2 topic output: `ros2 topic echo /agent_0/metrics`
3. Check browser console for errors
4. Verify backend logs

---

**Built for real-time multi-agent formation control simulation monitoring** 🚁✨
