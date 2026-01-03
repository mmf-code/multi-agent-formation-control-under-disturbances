/**
 * Zustand store for dashboard state management
 */
import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type {
  MetricsData,
  OdometryData,
  WindData,
  FormationState,
  SystemStatus,
  ControllerParams,
  RosGraphData,
  LogEntry,
  SimulationStatus,
  ScriptInfo,
  SnapshotData,
  Vector3,
} from '../types';

// ============================================================================
// Store Types
// ============================================================================

interface DashboardState {
  // Connection
  connected: boolean;

  // Core data
  metrics: Record<string, MetricsData>;
  odom: Record<string, OdometryData>;
  targets: Record<string, { position: Vector3 }>;
  controllerParams: Record<string, ControllerParams>;

  // History for charts
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];

  // Global state
  wind: WindData | null;
  formation: FormationState | null;
  systemStatus: SystemStatus | null;
  rosGraph: RosGraphData | null;

  // Logs
  logs: LogEntry[];

  // Simulation control
  simStatus: SimulationStatus;
  availableScripts: ScriptInfo[];
  selectedScript: string;
  simLoading: boolean;
  simulationEnded: boolean;
  simulationEndReason: string;

  // UI state
  activeTab: string;

  // Actions
  setConnected: (connected: boolean) => void;
  updateMetrics: (agentId: string, data: MetricsData) => void;
  updateOdom: (agentId: string, data: OdometryData) => void;
  updateTarget: (agentId: string, data: { position: Vector3 }) => void;
  updateControllerParams: (agentId: string, data: ControllerParams) => void;
  updateWind: (data: WindData) => void;
  updateFormation: (data: FormationState) => void;
  updateSystemStatus: (data: SystemStatus) => void;
  updateRosGraph: (data: RosGraphData) => void;
  addLog: (entry: LogEntry) => void;
  handleSnapshot: (snapshot: SnapshotData) => void;
  handleSimulationEvent: (event: { event: string; timestamp?: number; reason?: string; script?: string }) => void;
  setSimStatus: (status: SimulationStatus) => void;
  setAvailableScripts: (scripts: ScriptInfo[]) => void;
  setSelectedScript: (script: string) => void;
  setSimLoading: (loading: boolean) => void;
  setActiveTab: (tab: string) => void;
  clearData: () => void;
}

// ============================================================================
// Constants
// ============================================================================

const MAX_HISTORY = 500;
const MAX_LOGS = 100;

// ============================================================================
// Store Implementation
// ============================================================================

export const useDashboardStore = create<DashboardState>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    connected: false,
    metrics: {},
    odom: {},
    targets: {},
    controllerParams: {},
    metricsHistory: {},
    windHistory: [],
    wind: null,
    formation: null,
    systemStatus: null,
    rosGraph: null,
    logs: [],
    simStatus: { running: false, pid: null, script: null },
    availableScripts: [],
    selectedScript: 'run_formation_demo.sh',
    simLoading: false,
    simulationEnded: false,
    simulationEndReason: '',
    activeTab: 'dashboard',

    // Actions
    setConnected: (connected) => set({ connected }),

    updateMetrics: (agentId, data) => set((state) => {
      const agentHistory = state.metricsHistory[agentId] || [];
      const newHistory = [...agentHistory, data].slice(-MAX_HISTORY);

      return {
        metrics: { ...state.metrics, [agentId]: data },
        metricsHistory: { ...state.metricsHistory, [agentId]: newHistory },
      };
    }),

    updateOdom: (agentId, data) => set((state) => ({
      odom: { ...state.odom, [agentId]: data },
    })),

    updateTarget: (agentId, data) => set((state) => ({
      targets: { ...state.targets, [agentId]: data },
    })),

    updateControllerParams: (agentId, data) => set((state) => ({
      controllerParams: { ...state.controllerParams, [agentId]: data },
    })),

    updateWind: (data) => set((state) => ({
      wind: data,
      windHistory: [...state.windHistory, data].slice(-MAX_HISTORY),
    })),

    updateFormation: (data) => set({ formation: data }),

    updateSystemStatus: (data) => set({ systemStatus: data }),

    updateRosGraph: (data) => set({ rosGraph: data }),

    addLog: (entry) => set((state) => ({
      logs: [entry, ...state.logs].slice(0, MAX_LOGS),
    })),

    handleSnapshot: (snapshot) => set({
      metrics: snapshot.metrics || {},
      odom: snapshot.odom || {},
      targets: snapshot.target || {},
      controllerParams: snapshot.controller_params || {},
      formation: snapshot.formation,
      wind: snapshot.wind,
      rosGraph: snapshot.ros_graph || null,
      systemStatus: snapshot.status,
      logs: snapshot.diagnostics
        ? Object.entries(snapshot.diagnostics).map(([agentId, diag]) => ({
            timestamp: diag.timestamp,
            level: 'info' as const,
            message: `Agent ${agentId} diagnostics loaded`,
            agent: agentId,
          })).sort((a, b) => b.timestamp - a.timestamp)
        : [],
    }),

    handleSimulationEvent: (event) => {
      const state = get();

      if (event.event === 'stopped') {
        set({
          simulationEnded: true,
          simulationEndReason: event.reason || 'unknown',
        });
        state.addLog({
          timestamp: event.timestamp || Date.now() / 1000,
          level: 'warning',
          message: `Simulation ended: ${event.reason || 'unknown'}`,
        });
      } else if (event.event === 'started') {
        set({
          simulationEnded: false,
          simulationEndReason: '',
        });
        state.addLog({
          timestamp: event.timestamp || Date.now() / 1000,
          level: 'success',
          message: `Simulation started: ${event.script || 'unknown'}`,
        });
      }
    },

    setSimStatus: (status) => set({ simStatus: status }),
    setAvailableScripts: (scripts) => set({ availableScripts: scripts }),
    setSelectedScript: (script) => set({ selectedScript: script }),
    setSimLoading: (loading) => set({ simLoading: loading }),
    setActiveTab: (tab) => set({ activeTab: tab }),

    clearData: () => set({
      metrics: {},
      odom: {},
      targets: {},
      metricsHistory: {},
      windHistory: [],
      wind: null,
      formation: null,
      logs: [],
    }),
  }))
);

// ============================================================================
// Selectors
// ============================================================================

export const useMetrics = () => useDashboardStore((state) => state.metrics);
export const useOdom = () => useDashboardStore((state) => state.odom);
export const useWind = () => useDashboardStore((state) => state.wind);
export const useFormation = () => useDashboardStore((state) => state.formation);
export const useSystemStatus = () => useDashboardStore((state) => state.systemStatus);
export const useRosGraph = () => useDashboardStore((state) => state.rosGraph);
export const useControllerParams = () => useDashboardStore((state) => state.controllerParams);
export const useMetricsHistory = () => useDashboardStore((state) => state.metricsHistory);
export const useWindHistory = () => useDashboardStore((state) => state.windHistory);
export const useLogs = () => useDashboardStore((state) => state.logs);
export const useConnected = () => useDashboardStore((state) => state.connected);
export const useSimStatus = () => useDashboardStore((state) => state.simStatus);
export const useTargets = () => useDashboardStore((state) => state.targets);

// Derived selectors
export const useActiveAgents = () => useDashboardStore((state) =>
  Object.keys(state.metrics).sort()
);

export const useAgentCount = () => useDashboardStore((state) =>
  Object.keys(state.metrics).length
);

export const useSettledCount = () => useDashboardStore((state) =>
  Object.values(state.metrics).filter(m => m.is_settled).length
);

export const useAverageError = () => useDashboardStore((state) => {
  const values = Object.values(state.metrics);
  if (values.length === 0) return 0;
  return values.reduce((sum, m) => sum + m.error_magnitude, 0) / values.length;
});

export const useGroupStats = () => useDashboardStore((state) => {
  const { metrics, controllerParams } = state;
  const groups: Record<string, { count: number; avgError: number; settled: number; agents: string[] }> = {};

  Object.entries(metrics).forEach(([agentId, m]) => {
    const params = controllerParams[agentId];
    const type = params?.controller_type || 'unknown';

    if (!groups[type]) {
      groups[type] = { count: 0, avgError: 0, settled: 0, agents: [] };
    }
    groups[type].count++;
    groups[type].avgError += m.error_magnitude;
    groups[type].agents.push(agentId);
    if (m.is_settled) groups[type].settled++;
  });

  // Calculate averages
  Object.keys(groups).forEach((type) => {
    if (groups[type].count > 0) {
      groups[type].avgError /= groups[type].count;
    }
  });

  return groups;
});
