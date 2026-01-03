/**
 * Consolidated TypeScript types for the monitoring dashboard
 */

// ============================================================================
// Core Data Types
// ============================================================================

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface MetricsData {
  agent_id: string;
  timestamp: number;
  current_x: number;
  current_y: number;
  current_z: number;
  target_x: number;
  target_y: number;
  target_z: number;
  error_x: number;
  error_y: number;
  error_z: number;
  error_magnitude: number;
  rmse_x: number;
  rmse_y: number;
  rmse_z: number;
  rmse_total: number;
  iae_x: number;
  iae_y: number;
  itae_x: number;
  itae_y: number;
  settling_time: number;
  is_settled: boolean;
  max_overshoot_x: number;
  max_overshoot_y: number;
}

export interface OdometryData {
  agent_id: string;
  timestamp: number;
  position: Vector3;
  velocity: Vector3;
  orientation_x: number;
  orientation_y: number;
  orientation_z: number;
  orientation_w: number;
}

export interface WindData {
  timestamp: number;
  velocity: Vector3;
  force?: Vector3;
}

export interface FormationState {
  timestamp: number;
  shape: string;
  spacing: number;
  center_x: number;
  center_y: number;
  center_z: number;
  yaw_deg: number;
  agent_ids: string[];
}

export interface DiagnosticsData {
  agent_id: string;
  timestamp: number;
  x_total: number;
  x_pid: number;
  x_fuzzy: number;
  y_total: number;
  y_pid: number;
  y_fuzzy: number;
}

export interface ControllerParams {
  agent_id: string;
  timestamp: number;
  controller_type: ControllerType;
  pid_kp: number;
  pid_ki: number;
  pid_kd: number;
  fuzzy_enable: boolean;
  fuzzy_wind_scalar: number;
  mix_k_pid: number;
  mix_k_fuzzy: number;
  feedforward_enable_drag: boolean;
  feedforward_enable_wind: boolean;
  feedforward_k_drag: number;
  feedforward_k_wind: number;
  output_limit_x_min: number;
  output_limit_x_max: number;
  output_limit_y_min: number;
  output_limit_y_max: number;
  control_frequency_hz: number;
  dt: number;
}

export interface SystemStatus {
  timestamp: number;
  ros_ok: boolean;
  active_agents: string[];
  available_topics: string[];
  wind_active: boolean;
  formation_active: boolean;
}

// ============================================================================
// ROS Graph Types
// ============================================================================

export type NodeRole = 'controller' | 'sensor' | 'metrics' | 'infra' | 'viz' | 'planner' | 'agent_node';
export type NodeType = 'node' | 'topic' | 'service' | 'action';

export interface RosNode {
  id: string;
  name: string;
  namespace: string;
  type: NodeType;
  role?: NodeRole;
  group?: string;
  rates?: {
    publish: number;
    subscribe: number;
  };
}

export interface RosEdge {
  id: string;
  source: string;
  target: string;
  topicName?: string;
  msgType?: string;
  rate?: number;
}

export interface RosGraphData {
  nodes: RosNode[];
  edges: RosEdge[];
  timestamp: number;
}

// ============================================================================
// Controller Types
// ============================================================================

export type ControllerType = 'pid' | 'pd' | 'pid_fuzzy' | 'hybrid' | 'unknown';

export const CONTROLLER_LABELS: Record<ControllerType, string> = {
  pid: 'PID',
  pd: 'PD',
  pid_fuzzy: 'PID + Fuzzy',
  hybrid: 'PID + Fuzzy',
  unknown: 'Unknown',
};

export const CONTROLLER_COLORS: Record<ControllerType, { bg: string; text: string; border: string }> = {
  pid: { bg: 'bg-blue-900/50', text: 'text-blue-400', border: 'border-blue-700' },
  pd: { bg: 'bg-amber-900/50', text: 'text-amber-400', border: 'border-amber-700' },
  pid_fuzzy: { bg: 'bg-emerald-900/50', text: 'text-emerald-400', border: 'border-emerald-700' },
  hybrid: { bg: 'bg-emerald-900/50', text: 'text-emerald-400', border: 'border-emerald-700' },
  unknown: { bg: 'bg-gray-800/50', text: 'text-gray-400', border: 'border-gray-700' },
};

// ============================================================================
// Snapshot & WebSocket Types
// ============================================================================

export interface SnapshotData {
  metrics: Record<string, MetricsData>;
  odom: Record<string, OdometryData>;
  target: Record<string, { position: Vector3 }>;
  diagnostics: Record<string, DiagnosticsData>;
  controller_params: Record<string, ControllerParams>;
  formation: FormationState | null;
  wind: WindData | null;
  ros_graph?: RosGraphData;
  status: SystemStatus;
}

export interface SimulationEvent {
  event: 'started' | 'stopped';
  timestamp: number;
  reason?: string;
  script?: string;
}

export interface SimulationStatus {
  running: boolean;
  pid: number | null;
  script: string | null;
}

export interface ScriptInfo {
  name: string;
  path: string;
}

// ============================================================================
// Log Types
// ============================================================================

export type LogLevel = 'info' | 'warning' | 'error' | 'success' | 'debug';

export interface LogEntry {
  timestamp: number;
  level: LogLevel;
  message: string;
  agent?: string;
}

// ============================================================================
// UI State Types
// ============================================================================

export type TabType = 'dashboard' | 'overview' | 'charts' | 'agents' | 'map' | 'system' | 'params';
export type ChartType = 'error' | 'metrics' | 'wind' | 'overshoot';
export type ViewMode = 'individual' | 'aggregated';

// ============================================================================
// Crazyflie Mapping
// ============================================================================

export const CRAZYFLIE_MAPPING: Record<number, number> = {
  231: 0, 232: 1, 233: 2,  // Group 0: PID + Fuzzy
  234: 3, 235: 4, 236: 5,  // Group 1: PD
  237: 6, 238: 7, 239: 8,  // Group 2: PID
};

export const AGENT_TO_CRAZYFLIE: Record<number, number> = {
  0: 231, 1: 232, 2: 233,
  3: 234, 4: 235, 5: 236,
  6: 237, 7: 238, 8: 239,
};

/**
 * Get the agent ID from a Crazyflie ID
 */
export function cfIdToAgentId(cfId: number): string {
  const agentNum = CRAZYFLIE_MAPPING[cfId];
  return agentNum !== undefined ? `agent_${agentNum}` : `cf${cfId}`;
}

/**
 * Get display name for an agent (handles both sim and Crazyflie)
 */
export function getAgentDisplayName(agentId: string): string {
  const match = agentId.match(/agent_(\d+)/);
  if (match) {
    const num = parseInt(match[1], 10);
    const cfId = AGENT_TO_CRAZYFLIE[num];
    return cfId ? `Drone ${num + 1} (cf${cfId})` : `Agent ${num}`;
  }
  return agentId;
}

/**
 * Get controller group for an agent
 */
export function getAgentGroup(agentNum: number): { name: string; type: ControllerType } {
  if (agentNum >= 0 && agentNum <= 2) return { name: 'Group A', type: 'pid_fuzzy' };
  if (agentNum >= 3 && agentNum <= 5) return { name: 'Group B', type: 'pd' };
  if (agentNum >= 6 && agentNum <= 8) return { name: 'Group C', type: 'pid' };
  return { name: 'Unknown', type: 'unknown' };
}
