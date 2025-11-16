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

# Optionally serve built frontend (if available) at /ui
try:
    frontend_dist = (Path(__file__).resolve().parent.parent / 'frontend' / 'dist')
    if frontend_dist.is_dir():
        app.mount('/ui', StaticFiles(directory=str(frontend_dist), html=True), name='ui')
        logger.info(f"Serving frontend UI from {frontend_dist} at /ui")
    else:
        logger.info("Frontend UI not built; skipping static mount (/ui)")
except Exception:
    logger.exception("Failed to mount frontend UI static files")

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
        # Send initial snapshot
        if ros_bridge:
            try:
                initial_data = ros_bridge.get_all_latest_data()
                await websocket.send_json({
                    "type": "snapshot",
                    "data": initial_data
                })
                logger.info("Initial snapshot sent to WebSocket client")
            except Exception:
                logger.exception("Failed to send initial snapshot to WebSocket client")

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
                    await websocket.send_json({
                        "type": "update",
                        "data_type": data_type,
                        "agent_id": agent_id,
                        "data": data_payload.dict() if hasattr(data_payload, 'dict') else data_payload
                    })
                except Exception as e:
                    logger.error(f"Error sending WebSocket update: {e}")

        if ros_bridge:
            ros_bridge.add_update_callback(
                lambda dt, aid: asyncio.create_task(on_ros_update(dt, aid))
            )

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
