"""
ROS2 Bridge - Dynamic topic subscription and data management
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading
import time
from typing import Dict, List, Optional, Callable, Any
from collections import deque, defaultdict
import logging

# ROS2 message types
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float64MultiArray
from my_custom_interfaces_pkg.msg import MetricsData as ROSMetricsData
from my_custom_interfaces_pkg.msg import FormationState as ROSFormationState

from .schemas import (
    MetricsData,
    OdometryData,
    TargetPoseData,
    FormationState,
    WindData,
    DiagnosticsData,
    SystemStatus,
    Vector3 as Vector3Schema
)

logger = logging.getLogger(__name__)


class ROSBridge(Node):
    """
    ROS2 Bridge for monitoring dashboard
    Dynamically subscribes to topics and provides real-time data access
    """

    def __init__(self):
        super().__init__('monitoring_dashboard_bridge')

        # Data storage with history (last 1000 samples per topic)
        self.max_history = 1000
        self.metrics_data: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.max_history))
        self.odom_data: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.max_history))
        self.target_data: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.max_history))
        self.diagnostics_data: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.max_history))
        self.formation_data: deque = deque(maxlen=self.max_history)
        self.wind_velocity_data: deque = deque(maxlen=self.max_history)
        self.wind_force_data: deque = deque(maxlen=self.max_history)

        # Latest data cache (for quick access)
        self.latest_metrics: Dict[str, MetricsData] = {}
        self.latest_odom: Dict[str, OdometryData] = {}
        self.latest_target: Dict[str, TargetPoseData] = {}
        self.latest_diagnostics: Dict[str, DiagnosticsData] = {}
        self.latest_formation: Optional[FormationState] = None
        self.latest_wind: Optional[WindData] = None

        # Active agents tracking
        self.active_agents: set = set()
        self.discovered_topics: Dict[str, str] = {}  # topic_name -> msg_type

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions registry (avoid clashing with rclpy Node internals: `subscriptions` and `_subscriptions`)
        self._subs_registry: Dict[str, Any] = {}

        # Data update callbacks (for WebSocket notifications)
        self.update_callbacks: List[Callable] = []

        # Lock for thread-safe data access
        self.data_lock = threading.Lock()

        # Auto-discovery of agents and topics
        self.discover_timer = self.create_timer(1.0, self.discover_topics)

        logger.info("ROSBridge initialized - monitoring dashboard ready")

    def add_update_callback(self, callback: Callable):
        """Register callback for data updates (WebSocket push)"""
        self.update_callbacks.append(callback)

    def notify_update(self, data_type: str, agent_id: Optional[str] = None):
        """Notify all registered callbacks about data update"""
        for callback in self.update_callbacks:
            try:
                callback(data_type, agent_id)
            except Exception as e:
                logger.error(f"Error in update callback: {e}")

    def discover_topics(self):
        """Auto-discover available topics and create subscriptions"""
        topic_list = self.get_topic_names_and_types()

        for topic_name, msg_types in topic_list:
            if topic_name in self._subs_registry:
                continue  # Already subscribed

            msg_type = msg_types[0] if msg_types else ""

            # Discover agent topics
            if '/agent_' in topic_name:
                parts = topic_name.split('/')
                if len(parts) >= 3:
                    agent_id = parts[1]  # e.g., "agent_0"
                    self.active_agents.add(agent_id)

                    if topic_name.endswith('/metrics'):
                        self.subscribe_metrics(agent_id)
                    elif topic_name.endswith('/odom'):
                        self.subscribe_odom(agent_id)
                    elif topic_name.endswith('/target_pose'):
                        self.subscribe_target_pose(agent_id)
                    elif topic_name.endswith('/diagnostics'):
                        self.subscribe_diagnostics(agent_id)

            # Global topics
            elif topic_name == '/wind/velocity':
                self.subscribe_wind_velocity()
            elif topic_name == '/wind/force':
                self.subscribe_wind_force()
            else:
                # Formation state can be under different namespaces, match by msg type or suffix
                if msg_type.endswith('my_custom_interfaces_pkg/msg/FormationState') or \
                   (topic_name.endswith('/state') and 'formation_' in topic_name):
                    self.subscribe_formation_state(topic_name)

            self.discovered_topics[topic_name] = msg_type

        logger.debug(f"Active agents: {self.active_agents}")

    # =========================================================================
    # Subscription Methods
    # =========================================================================

    def subscribe_metrics(self, agent_id: str):
        """Subscribe to agent metrics topic"""
        topic = f'/{agent_id}/metrics'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            ROSMetricsData,
            topic,
            lambda msg: self._on_metrics(agent_id, msg),
            10
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_odom(self, agent_id: str):
        """Subscribe to agent odometry topic"""
        topic = f'/{agent_id}/odom'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            Odometry,
            topic,
            lambda msg: self._on_odom(agent_id, msg),
            self.sensor_qos
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_target_pose(self, agent_id: str):
        """Subscribe to agent target pose topic"""
        topic = f'/{agent_id}/target_pose'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            PoseStamped,
            topic,
            lambda msg: self._on_target_pose(agent_id, msg),
            self.sensor_qos
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_diagnostics(self, agent_id: str):
        """Subscribe to agent diagnostics topic"""
        topic = f'/{agent_id}/diagnostics'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            Float64MultiArray,
            topic,
            lambda msg: self._on_diagnostics(agent_id, msg),
            10
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_wind_velocity(self):
        """Subscribe to wind velocity topic"""
        topic = '/wind/velocity'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            Vector3,
            topic,
            self._on_wind_velocity,
            10
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_wind_force(self):
        """Subscribe to wind force topic"""
        topic = '/wind/force'
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            Vector3,
            topic,
            self._on_wind_force,
            10
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    def subscribe_formation_state(self, topic: str):
        """Subscribe to formation state topic (namespaced)"""
        if topic in self._subs_registry:
            return

        sub = self.create_subscription(
            ROSFormationState,
            topic,
            self._on_formation_state,
            10
        )
        self._subs_registry[topic] = sub
        logger.info(f"Subscribed to {topic}")

    # =========================================================================
    # Callback Handlers
    # =========================================================================

    def _on_metrics(self, agent_id: str, msg: ROSMetricsData):
        """Handle metrics message"""
        timestamp = time.time()

        data = MetricsData(
            agent_id=agent_id,
            timestamp=timestamp,
            current_x=msg.current_x,
            current_y=msg.current_y,
            current_z=msg.current_z,
            target_x=msg.target_x,
            target_y=msg.target_y,
            target_z=msg.target_z,
            error_x=msg.error_x,
            error_y=msg.error_y,
            error_z=msg.error_z,
            error_magnitude=msg.error_magnitude,
            rmse_x=msg.rmse_x,
            rmse_y=msg.rmse_y,
            rmse_z=msg.rmse_z,
            rmse_total=msg.rmse_total,
            iae_x=msg.iae_x,
            iae_y=msg.iae_y,
            itae_x=msg.itae_x,
            itae_y=msg.itae_y,
            settling_time=msg.settling_time,
            is_settled=msg.is_settled,
            max_overshoot_x=msg.max_overshoot_x,
            max_overshoot_y=msg.max_overshoot_y
        )

        with self.data_lock:
            if not isinstance(self.metrics_data.get(agent_id), deque):
                self.metrics_data[agent_id] = deque(maxlen=self.max_history)
            self.metrics_data[agent_id].append(data)
            self.latest_metrics[agent_id] = data

        self.notify_update('metrics', agent_id)

    def _on_odom(self, agent_id: str, msg: Odometry):
        """Handle odometry message"""
        timestamp = time.time()

        data = OdometryData(
            agent_id=agent_id,
            timestamp=timestamp,
            position=Vector3Schema(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                z=msg.pose.pose.position.z
            ),
            velocity=Vector3Schema(
                x=msg.twist.twist.linear.x,
                y=msg.twist.twist.linear.y,
                z=msg.twist.twist.linear.z
            ),
            orientation_x=msg.pose.pose.orientation.x,
            orientation_y=msg.pose.pose.orientation.y,
            orientation_z=msg.pose.pose.orientation.z,
            orientation_w=msg.pose.pose.orientation.w
        )

        with self.data_lock:
            if not isinstance(self.odom_data.get(agent_id), deque):
                self.odom_data[agent_id] = deque(maxlen=self.max_history)
            self.odom_data[agent_id].append(data)
            self.latest_odom[agent_id] = data

        self.notify_update('odom', agent_id)

    def _on_target_pose(self, agent_id: str, msg: PoseStamped):
        """Handle target pose message"""
        timestamp = time.time()

        data = TargetPoseData(
            agent_id=agent_id,
            timestamp=timestamp,
            position=Vector3Schema(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z
            )
        )

        with self.data_lock:
            if not isinstance(self.target_data.get(agent_id), deque):
                self.target_data[agent_id] = deque(maxlen=self.max_history)
            self.target_data[agent_id].append(data)
            self.latest_target[agent_id] = data

        self.notify_update('target', agent_id)

    def _on_diagnostics(self, agent_id: str, msg: Float64MultiArray):
        """Handle diagnostics message"""
        if len(msg.data) < 6:
            return

        timestamp = time.time()

        data = DiagnosticsData(
            agent_id=agent_id,
            timestamp=timestamp,
            x_total=msg.data[0],
            x_pid=msg.data[1],
            x_fuzzy=msg.data[2],
            y_total=msg.data[3],
            y_pid=msg.data[4],
            y_fuzzy=msg.data[5]
        )

        with self.data_lock:
            if not isinstance(self.diagnostics_data.get(agent_id), deque):
                self.diagnostics_data[agent_id] = deque(maxlen=self.max_history)
            self.diagnostics_data[agent_id].append(data)
            self.latest_diagnostics[agent_id] = data

        self.notify_update('diagnostics', agent_id)

    def _on_wind_velocity(self, msg: Vector3):
        """Handle wind velocity message"""
        timestamp = time.time()

        with self.data_lock:
            if not isinstance(self.wind_velocity_data, deque):
                self.wind_velocity_data = deque(maxlen=self.max_history)
            wind_data = WindData(
                timestamp=timestamp,
                velocity=Vector3Schema(x=msg.x, y=msg.y, z=msg.z)
            )
            self.wind_velocity_data.append(wind_data)

            # Merge with force data if available
            if self.latest_wind:
                self.latest_wind.velocity = wind_data.velocity
            else:
                self.latest_wind = wind_data

        self.notify_update('wind')

    def _on_wind_force(self, msg: Vector3):
        """Handle wind force message"""
        timestamp = time.time()

        with self.data_lock:
            if not isinstance(self.wind_force_data, deque):
                self.wind_force_data = deque(maxlen=self.max_history)
            force_vec = Vector3Schema(x=msg.x, y=msg.y, z=msg.z)
            self.wind_force_data.append((timestamp, force_vec))

            # Merge with velocity data
            if self.latest_wind:
                self.latest_wind.force = force_vec
            else:
                self.latest_wind = WindData(
                    timestamp=timestamp,
                    velocity=Vector3Schema(x=0, y=0, z=0),
                    force=force_vec
                )

        self.notify_update('wind')

    def _on_formation_state(self, msg: ROSFormationState):
        """Handle formation state message"""
        timestamp = time.time()

        data = FormationState(
            timestamp=timestamp,
            shape=msg.shape,
            spacing=msg.spacing,
            center_x=msg.center_x,
            center_y=msg.center_y,
            center_z=msg.center_z,
            yaw_deg=msg.yaw_deg,
            agent_ids=list(msg.agent_ids)
        )

        with self.data_lock:
            if not isinstance(self.formation_data, deque):
                self.formation_data = deque(maxlen=self.max_history)
            self.formation_data.append(data)
            self.latest_formation = data

        self.notify_update('formation')

    # =========================================================================
    # Data Access Methods
    # =========================================================================

    def get_latest_metrics(self, agent_id: str) -> Optional[MetricsData]:
        """Get latest metrics for an agent"""
        with self.data_lock:
            return self.latest_metrics.get(agent_id)

    def get_metrics_history(self, agent_id: str, limit: int = 100) -> List[MetricsData]:
        """Get metrics history for an agent"""
        with self.data_lock:
            data = list(self.metrics_data.get(agent_id, []))
            return data[-limit:]

    def get_latest_odom(self, agent_id: str) -> Optional[OdometryData]:
        """Get latest odometry for an agent"""
        with self.data_lock:
            return self.latest_odom.get(agent_id)

    def get_odom_history(self, agent_id: str, limit: int = 100) -> List[OdometryData]:
        """Get odometry history for an agent"""
        with self.data_lock:
            data = list(self.odom_data.get(agent_id, []))
            return data[-limit:]

    def get_latest_wind(self) -> Optional[WindData]:
        """Get latest wind data"""
        with self.data_lock:
            return self.latest_wind

    def get_wind_history(self, limit: int = 100) -> List[WindData]:
        """Get wind history"""
        with self.data_lock:
            return list(self.wind_velocity_data)[-limit:]

    def get_latest_formation(self) -> Optional[FormationState]:
        """Get latest formation state"""
        with self.data_lock:
            return self.latest_formation

    def get_system_status(self) -> SystemStatus:
        """Get overall system status"""
        with self.data_lock:
            return SystemStatus(
                timestamp=time.time(),
                ros_ok=rclpy.ok(),
                active_agents=sorted(list(self.active_agents)),
                available_topics=sorted(list(self.discovered_topics.keys())),
                wind_active=self.latest_wind is not None,
                formation_active=self.latest_formation is not None
            )

    def get_all_latest_data(self) -> dict:
        """Get all latest data in one shot (for dashboard snapshot)"""
        with self.data_lock:
            return {
                'metrics': {k: v.dict() for k, v in self.latest_metrics.items()},
                'odom': {k: v.dict() for k, v in self.latest_odom.items()},
                'target': {k: v.dict() for k, v in self.latest_target.items()},
                'diagnostics': {k: v.dict() for k, v in self.latest_diagnostics.items()},
                'formation': self.latest_formation.dict() if self.latest_formation else None,
                'wind': self.latest_wind.dict() if self.latest_wind else None,
                'status': self.get_system_status().dict()
            }
