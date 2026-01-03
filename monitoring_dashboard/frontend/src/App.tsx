/**
 * Main Application Entry Point
 *
 * Multi-Agent Formation Control Dashboard
 * Real-time monitoring for drone swarm simulations
 *
 * Features:
 * - Light/Dark theme support
 * - Thesis export mode for academic publications
 * - Real-time ROS2 data visualization
 */
import { useEffect } from 'react';
import { Toaster } from 'sonner';
import { DashboardLayout } from './components/DashboardLayout';
import { SimulationMetricsPanel } from './components/SimulationMetricsPanel';
import { RosGraphPanel } from './components/RosGraphPanel';
import { DiagnosticsPanel } from './components/DiagnosticsPanel';
import { useWebSocket } from './hooks';
import {
  useDashboardStore,
  useConnected,
  useSystemStatus,
  useLogs,
  useMetrics,
  useMetricsHistory,
  useWindHistory,
  useWind,
  useFormation,
  useOdom,
  useTargets,
  useControllerParams,
  useRosGraph,
} from './store';
import { initializeTheme, useTheme } from './store/theme';

function App() {
  // Initialize theme on mount
  useEffect(() => {
    initializeTheme();
  }, []);

  // Initialize WebSocket connection
  useWebSocket({ autoConnect: true });

  // Theme state
  const theme = useTheme();

  // Store selectors
  const connected = useConnected();
  const systemStatus = useSystemStatus();
  const logs = useLogs();
  const metrics = useMetrics();
  const metricsHistory = useMetricsHistory();
  const windHistory = useWindHistory();
  const wind = useWind();
  const formation = useFormation();
  const odom = useOdom();
  const targets = useTargets();
  const controllerParams = useControllerParams();
  const rosGraph = useRosGraph();

  const { simulationEnded, simulationEndReason } = useDashboardStore();

  return (
    <>
      {/* Toast notifications - Theme aware */}
      <Toaster
        position="top-right"
        toastOptions={{
          className: 'font-sans',
        }}
        theme={theme}
        richColors
        closeButton
      />

      {/* Main Dashboard */}
      <DashboardLayout
        connected={connected}
        systemStatus={systemStatus}
        simulationEnded={simulationEnded}
        simulationEndReason={simulationEndReason}
        leftPanel={
          <SimulationMetricsPanel
            metrics={metrics}
            wind={wind}
            formation={formation}
            status={systemStatus}
            metricsHistory={metricsHistory}
            windHistory={windHistory}
            odomData={odom}
            targetData={targets}
            controllerParams={controllerParams}
          />
        }
        rightPanel={
          <RosGraphPanel
            graphData={rosGraph as any}
            controllerParams={controllerParams}
          />
        }
        bottomPanel={
          <DiagnosticsPanel logs={logs as any} />
        }
      />
    </>
  );
}

export default App;
