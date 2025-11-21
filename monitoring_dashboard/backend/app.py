"""
Monitoring Dashboard Backend
FastAPI + WebSocket server for real-time ROS2 data streaming
"""
import asyncio
import time
import logging
import threading
from contextlib import asynccontextmanager
from typing import Set, Optional

import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from pathlib import Path

from ros_bridge import ROSBridge

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global ROS bridge instance
ros_bridge: Optional[ROSBridge] = None
ros_thread: Optional[threading.Thread] = None


def ros_spin_thread():
    """ROS2 spin in separate thread"""
    global ros_bridge
    logger.info("ROS2 spin thread started")
    try:
        rclpy.spin(ros_bridge)
    except Exception:
        # Log full traceback to identify exact failure line
        logger.exception("ROS2 spin error")
    finally:
        logger.info("ROS2 spin thread stopped")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    global ros_bridge, ros_thread

    # Startup
    logger.info("Starting monitoring dashboard backend...")

    # Initialize ROS2
    try:
        rclpy.init()
        ros_bridge = ROSBridge()

        # Start ROS2 spin in separate thread
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()

        logger.info("ROS2 bridge initialized and running")
    except Exception as e:
        logger.error(f"Failed to initialize ROS2: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down monitoring dashboard backend...")
    if ros_bridge:
        try:
            ros_bridge.destroy_node()
        except Exception:
            logger.debug("ROS bridge destroy_node raised; continuing shutdown", exc_info=True)
    # Avoid double-shutdown errors if context is already down
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        logger.debug("rclpy.shutdown raised (possibly already shut down)", exc_info=True)


# Create FastAPI app
app = FastAPI(
    title="Multi-Agent Formation Control - Monitoring Dashboard",
    description="Real-time monitoring for ROS2 drone simulation",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware for frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Active WebSocket connections
active_connections: Set[WebSocket] = set()


# ============================================================================
# REST API Endpoints
# ============================================================================

@app.get("/")
async def root():
    """API root endpoint"""
    return {
        "name": "Monitoring Dashboard API",
        "version": "1.0.0",
        "status": "running",
        "ros_ok": rclpy.ok() if ros_bridge else False
    }


@app.get("/api/health")
async def health_check():
    """Health check endpoint with detailed ROS status"""
    if not ros_bridge:
        return JSONResponse(
            status_code=503,
            content={
                "status": "unhealthy",
                "ros_initialized": False,
                "message": "ROS bridge not initialized"
            }
        )

    return {
        "status": "healthy",
        "ros_initialized": True,
        "ros_ok": rclpy.ok(),
        "active_agents": len(ros_bridge.active_agents),
        "discovered_topics": len(ros_bridge.discovered_topics),
        "active_connections": len(active_connections)
    }


@app.get("/api/status")
async def get_status():
    """Get system status"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    status = ros_bridge.get_system_status()
    return status.dict()


@app.get("/api/topics")
async def get_topics():
    """Get available ROS2 topics"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    return {
        "topics": ros_bridge.discovered_topics,
        "active_agents": sorted(list(ros_bridge.active_agents))
    }


@app.get("/api/agents")
async def get_agents():
    """Get list of active agents"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    return {
        "agents": sorted(list(ros_bridge.active_agents))
    }


@app.get("/api/snapshot")
async def get_snapshot():
    """Get complete data snapshot (all latest values)"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    return ros_bridge.get_all_latest_data()


@app.get("/api/metrics/{agent_id}")
async def get_metrics(agent_id: str, limit: int = 100):
    """Get metrics history for specific agent"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    history = ros_bridge.get_metrics_history(agent_id, limit)
    return {
        "agent_id": agent_id,
        "count": len(history),
        "data": [m.dict() for m in history]
    }


@app.get("/api/odom/{agent_id}")
async def get_odom(agent_id: str, limit: int = 100):
    """Get odometry history for specific agent"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    history = ros_bridge.get_odom_history(agent_id, limit)
    return {
        "agent_id": agent_id,
        "count": len(history),
        "data": [o.dict() for o in history]
    }


@app.get("/api/wind")
async def get_wind(limit: int = 100):
    """Get wind data history"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    history = ros_bridge.get_wind_history(limit)
    return {
        "count": len(history),
        "data": [w.dict() for w in history]
    }


@app.get("/api/formation")
async def get_formation():
    """Get latest formation state"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    formation = ros_bridge.get_latest_formation()
    if not formation:
        return {"status": "no_data"}

    return formation.dict()


    return formation.dict()


@app.get("/api/graph")
async def get_graph():
    """Get ROS2 graph topology"""
    if not ros_bridge:
        raise HTTPException(status_code=503, detail="ROS bridge not initialized")

    graph = ros_bridge.get_ros_graph()
    return graph.dict()


# ============================================================================
# WebSocket Endpoint for Real-time Updates
# ============================================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time data streaming

    Client can send subscription messages:
    {
        "action": "subscribe",
        "topics": ["metrics", "odom", "wind", "formation"],
        "agents": ["agent_0", "agent_1"]  // optional, all if not specified
    }
    """
    await websocket.accept()
    active_connections.add(websocket)
    logger.info(f"WebSocket client connected. Total connections: {len(active_connections)}")

    # Client subscription preferences
    subscribed_topics = set()
    subscribed_agents = set()
    subscribe_all_agents = True

    try:
        # Send initial snapshot to frontend
        if ros_bridge:
            try:
                initial_data = ros_bridge.get_all_latest_data()
                if initial_data:
                    # Add graph data to snapshot
                    graph_data = ros_bridge.get_ros_graph()
                    
                    await websocket.send_json({
                        "type": "snapshot",
                        "data": {
                            **initial_data,
                            "ros_graph": graph_data.dict()
                        }
                    })
                    logger.info(f"Initial snapshot sent (metrics: {len(initial_data.get('metrics', {}))}, odom: {len(initial_data.get('odom', {}))})")
                else:
                    logger.info("No initial data available yet")
            except Exception as e:
                logger.error(f"Failed to send initial snapshot: {e}", exc_info=True)

        # Register callback for ROS data updates
        async def on_ros_update(data_type: str, agent_id: Optional[str] = None):
            """Callback when ROS data updates"""
            # Check subscription filters
            if subscribed_topics and data_type not in subscribed_topics:
                return
            if agent_id and not subscribe_all_agents:
                if agent_id not in subscribed_agents:
                    return

            # Get updated data
            data_payload = None
            if data_type == 'metrics' and agent_id:
                data_payload = ros_bridge.get_latest_metrics(agent_id)
            elif data_type == 'odom' and agent_id:
                data_payload = ros_bridge.get_latest_odom(agent_id)
            elif data_type == 'wind':
                data_payload = ros_bridge.get_latest_wind()
            elif data_type == 'formation':
                data_payload = ros_bridge.get_latest_formation()
            elif data_type == 'diagnostics' and agent_id:
                latest = ros_bridge.latest_diagnostics.get(agent_id)
                if latest:
                    data_payload = latest

            if data_payload:
                try:
                    if websocket.client_state.name == "CONNECTED":
                        await websocket.send_json({
                            "type": "update",
                            "data_type": data_type,
                            "agent_id": agent_id,
                            "data": data_payload.dict() if hasattr(data_payload, 'dict') else data_payload
                        })
                except RuntimeError as e:
                    # WebSocket might be closed during send
                    logger.debug(f"WebSocket runtime error during update (connection likely closed): {e}")
                    return
                except Exception as e:
                    logger.error(f"Error sending WebSocket update: {e}")

        ros_callback = None
        if ros_bridge:
            # Store the event loop for callbacks from ROS thread
            loop = asyncio.get_event_loop()

            def ros_callback(dt, aid):
                """Callback from ROS thread - schedule coroutine in event loop"""
                try:
                    asyncio.run_coroutine_threadsafe(on_ros_update(dt, aid), loop)
                except Exception as e:
                    logger.error(f"Error scheduling ROS update: {e}")

            ros_bridge.add_update_callback(ros_callback)

        async def heartbeat():
            """Periodic heartbeat to keep connection alive and aid debugging"""
            try:
                while True:
                    await asyncio.sleep(20)
                    await websocket.send_json({"type": "server_heartbeat", "ts": time.time()})
            except Exception:
                # Any error here will be handled by the main loop/close
                pass

        hb_task = asyncio.create_task(heartbeat())

        # Listen for client messages (subscription management)
        while True:
            try:
                message = await asyncio.wait_for(
                    websocket.receive_json(),
                    timeout=1.0
                )

                action = message.get("action")

                if action == "subscribe":
                    # Update subscription preferences
                    topics = message.get("topics", [])
                    agents = message.get("agents", [])

                    if topics:
                        subscribed_topics = set(topics)
                    else:
                        subscribed_topics.clear()  # Subscribe to all

                    if agents:
                        subscribed_agents = set(agents)
                        subscribe_all_agents = False
                    else:
                        subscribe_all_agents = True

                    await websocket.send_json({
                        "type": "subscription_updated",
                        "topics": list(subscribed_topics),
                        "agents": list(subscribed_agents) if not subscribe_all_agents else "all"
                    })
                    logger.info(f"Client subscription updated: topics={subscribed_topics}, agents={subscribed_agents}")

                elif action == "ping":
                    await websocket.send_json({"type": "pong"})

            except asyncio.TimeoutError:
                # No message received, continue (allows background updates)
                continue
            except WebSocketDisconnect:
                break

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        try:
            hb_task.cancel()
        except Exception:
            pass
        if ros_bridge and ros_callback:
            ros_bridge.remove_update_callback(ros_callback)
        active_connections.discard(websocket)
        logger.info(f"WebSocket client removed. Total connections: {len(active_connections)}")


# Provide an alias under /ui/ws to support deployments serving UI at /ui
@app.websocket("/ui/ws")
async def websocket_endpoint_ui(websocket: WebSocket):
    await websocket_endpoint(websocket)


# ============================================================================
# Broadcast helper (for future use)
# ============================================================================

async def broadcast_message(message: dict):
    """Broadcast message to all connected WebSocket clients"""
    disconnected = set()

    for connection in active_connections:
        try:
            await connection.send_json(message)
        except Exception as e:
            logger.error(f"Error broadcasting to client: {e}")
            disconnected.add(connection)

    # Clean up disconnected clients
    active_connections.difference_update(disconnected)


# ============================================================================
# Static Files (must be last to avoid blocking routes)
# ============================================================================

# Optionally serve built frontend (if available) at /ui
# IMPORTANT: This MUST come after all route definitions to avoid blocking them
try:
    frontend_dist = (Path(__file__).resolve().parent.parent / 'frontend' / 'dist')
    if frontend_dist.is_dir():
        app.mount('/ui', StaticFiles(directory=str(frontend_dist), html=True), name='ui')
        logger.info(f"Serving frontend UI from {frontend_dist} at /ui")
    else:
        logger.info("Frontend UI not built; skipping static mount (/ui)")
except Exception:
    logger.exception("Failed to mount frontend UI static files")


# ============================================================================
# Simulation Control API
# ============================================================================

import subprocess
import signal
import os

# Global process tracking
simulation_process: Optional[subprocess.Popen] = None
simulation_status = {"running": False, "pid": None, "script": None}

@app.post("/api/simulation/start")
async def start_simulation(script: str = "run_formation_demo.sh"):
    """Start a simulation script"""
    global simulation_process, simulation_status

    logger.info(f"Start simulation request received: script='{script}'")

    if simulation_process and simulation_process.poll() is None:
        return JSONResponse(
            status_code=400,
            content={"error": "Simulation already running", "pid": simulation_process.pid}
        )

    # Get project root directory
    project_root = Path(__file__).resolve().parent.parent.parent
    scripts_dir = project_root / "scripts"
    script_path = scripts_dir / script

    if not script_path.exists():
        return JSONResponse(
            status_code=404,
            content={"error": f"Script not found: {script}"}
        )

    # Build the command to run in a new shell with ROS2 sourced
    ros_setup = "/opt/ros/humble/setup.bash"
    workspace_setup = project_root / "install" / "setup.bash"

    cmd = f"""
    cd {project_root} && \
    source {ros_setup} && \
    source {workspace_setup} && \
    {script_path}
    """

    try:
        # Start the process in background
        simulation_process = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # Create new process group for clean termination
            cwd=str(project_root)
        )

        simulation_status = {
            "running": True,
            "pid": simulation_process.pid,
            "script": script
        }

        logger.info(f"Started simulation: {script} (PID: {simulation_process.pid})")

        return {
            "status": "started",
            "pid": simulation_process.pid,
            "script": script
        }
    except Exception as e:
        logger.error(f"Failed to start simulation: {e}")
        return JSONResponse(
            status_code=500,
            content={"error": str(e)}
        )


@app.post("/api/simulation/stop")
async def stop_simulation():
    """Stop the running simulation"""
    global simulation_process, simulation_status

    if not simulation_process or simulation_process.poll() is not None:
        simulation_status = {"running": False, "pid": None, "script": None}
        return {"status": "not_running"}

    try:
        # Kill the entire process group
        os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
        simulation_process.wait(timeout=5)

        logger.info(f"Stopped simulation (PID: {simulation_status['pid']})")

        simulation_status = {"running": False, "pid": None, "script": None}
        return {"status": "stopped"}
    except subprocess.TimeoutExpired:
        # Force kill if it doesn't stop gracefully
        os.killpg(os.getpgid(simulation_process.pid), signal.SIGKILL)
        simulation_status = {"running": False, "pid": None, "script": None}
        return {"status": "force_killed"}
    except Exception as e:
        logger.error(f"Error stopping simulation: {e}")
        return JSONResponse(
            status_code=500,
            content={"error": str(e)}
        )


@app.get("/api/simulation/status")
async def get_simulation_status():
    """Get current simulation status"""
    global simulation_process, simulation_status

    # Check if process is still running
    if simulation_process:
        if simulation_process.poll() is None:
            simulation_status["running"] = True
        else:
            simulation_status = {"running": False, "pid": None, "script": None}

    return simulation_status


@app.get("/api/simulation/scripts")
async def list_simulation_scripts():
    """List available simulation scripts"""
    project_root = Path(__file__).resolve().parent.parent.parent
    scripts_dir = project_root / "scripts"

    if not scripts_dir.exists():
        return {"scripts": []}

    scripts = []
    for f in scripts_dir.glob("*.sh"):
        scripts.append({
            "name": f.name,
            "path": str(f.relative_to(project_root))
        })

    return {"scripts": sorted(scripts, key=lambda x: x["name"])}


# ============================================================================
# Controller Parameters API
# ============================================================================

@app.get("/api/params")
async def get_params():
    """Get controller parameters (Mock for now)"""
    return {
        "groups": [
            {
                "id": 0,
                "type": "PID+Fuzzy",
                "params": {
                    "kp": 1.5,
                    "ki": 0.01,
                    "kd": 0.5,
                    "k_fuzzy": 0.8,
                    "fuzzy_rules": "standard"
                },
                "thresholds": {
                    "settled_pos": 0.1,
                    "settled_time": 2.0
                }
            },
            {
                "id": 1,
                "type": "PID",
                "params": {
                    "kp": 1.2,
                    "ki": 0.0,
                    "kd": 0.3
                },
                "thresholds": {
                    "settled_pos": 0.15,
                    "settled_time": 1.5
                }
            }
        ],
        "global": {
            "metrics_window_sec": 60,
            "safety_enabled": True
        }
    }


# ============================================================================
# Main entry point
# ============================================================================

if __name__ == "__main__":
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=8000,
        log_level="info",
        reload=False  # Set to True for development
    )
