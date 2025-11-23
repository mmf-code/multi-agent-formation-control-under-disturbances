"""
Data schemas for ROS2 messages
Pydantic models for type safety and JSON serialization
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class Vector3(BaseModel):
    """3D vector"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class MetricsData(BaseModel):
    """Performance metrics for controller evaluation"""
    agent_id: str
    timestamp: float

    # Current and target positions
    current_x: float
    current_y: float
    current_z: float
    target_x: float
    target_y: float
    target_z: float

    # Position errors
    error_x: float
    error_y: float
    error_z: float
    error_magnitude: float

    # RMSE
    rmse_x: float
    rmse_y: float
    rmse_z: float
    rmse_total: float

    # Integral metrics
    iae_x: float
    iae_y: float
    itae_x: float
    itae_y: float

    # Response characteristics
    settling_time: float
    is_settled: bool
    max_overshoot_x: float
    max_overshoot_y: float


class OdometryData(BaseModel):
    """Drone odometry data"""
    agent_id: str
    timestamp: float

    # Position
    position: Vector3

    # Velocity
    velocity: Vector3

    # Orientation (quaternion)
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0


class TargetPoseData(BaseModel):
    """Target pose for drone"""
    agent_id: str
    timestamp: float
    position: Vector3


class FormationState(BaseModel):
    """Formation coordinator state"""
    timestamp: float
    shape: str
    spacing: float
    center_x: float
    center_y: float
    center_z: float
    yaw_deg: float
    agent_ids: list[str]


class WindData(BaseModel):
    """Wind environment data"""
    timestamp: float
    velocity: Vector3
    force: Optional[Vector3] = None


class DiagnosticsData(BaseModel):
    """Controller diagnostics (PID/Fuzzy contributions)"""
    agent_id: str
    timestamp: float

    # X-axis controller outputs
    x_total: float
    x_pid: float
    x_fuzzy: float

    # Y-axis controller outputs
    y_total: float
    y_pid: float
    y_fuzzy: float


class TopicInfo(BaseModel):
    """Information about available ROS2 topics"""
    name: str
    msg_type: str
    publisher_count: int
    subscription_count: int


class ControllerParams(BaseModel):
    """Controller parameters for agent"""
    agent_id: str
    timestamp: float

    # Controller type
    controller_type: str

    # PID parameters
    pid_kp: float
    pid_ki: float
    pid_kd: float

    # Fuzzy parameters
    fuzzy_enable: bool
    fuzzy_wind_scalar: float

    # Hybrid mixing
    mix_k_pid: float
    mix_k_fuzzy: float

    # Feed-forward
    feedforward_enable_drag: bool
    feedforward_enable_wind: bool
    feedforward_k_drag: float
    feedforward_k_wind: float

    # Output limits
    output_limit_x_min: float
    output_limit_x_max: float
    output_limit_y_min: float
    output_limit_y_max: float

    # Timing
    control_frequency_hz: float
    dt: float


class AggregatedMetrics(BaseModel):
    """Aggregated metrics for a group of agents (e.g., by controller type)"""
    controller_type: str
    timestamp: float
    agent_count: int

    # Average position errors
    avg_error_x: float
    avg_error_y: float
    avg_error_z: float
    avg_error_magnitude: float

    # Average RMSE
    avg_rmse_x: float
    avg_rmse_y: float
    avg_rmse_z: float
    avg_rmse_total: float

    # Average integral metrics
    avg_iae_x: float
    avg_iae_y: float
    avg_itae_x: float
    avg_itae_y: float

    # Response characteristics (max overshoot is max, not avg)
    avg_settling_time: float
    max_overshoot_x: float
    max_overshoot_y: float


class SystemStatus(BaseModel):
    """Overall system status"""
    timestamp: float
    ros_ok: bool
    active_agents: list[str]
    available_topics: list[str]
    formation_active: bool


class RosNode(BaseModel):
    """ROS2 Node info for graph"""
    id: str
    name: str
    namespace: str
    type: str = "node"
    role: str = "unknown"
    group: Optional[str] = None


class RosEdge(BaseModel):
    """ROS2 connection edge"""
    id: str
    source: str
    target: str
    topicName: str
    msgType: str


class RosGraphData(BaseModel):
    """Complete ROS2 graph topology"""
    nodes: list[RosNode]
    edges: list[RosEdge]
    timestamp: float
