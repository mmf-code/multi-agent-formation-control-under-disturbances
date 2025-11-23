import { useEffect, useState } from 'react';
import { DashboardLayout } from './components/DashboardLayout';
import { SimulationMetricsPanel } from './components/SimulationMetricsPanel';
import { RosGraphPanel } from './components/RosGraphPanel';
import { DiagnosticsPanel } from './components/DiagnosticsPanel';
import { LogEntry } from './components/EventLog';
import {
  wsClient,
  MetricsData,
  OdometryData,
  SnapshotData,
  DiagnosticsData,
  WindData,
  FormationState,
  SystemStatus,
  ControllerParams,
  RosGraphData
} from './api/ws';

const MAX_HISTORY = 500; // Keep last 500 samples per agent

function App() {
  const [connected, setConnected] = useState(false);
  const [metrics, setMetrics] = useState<Record<string, MetricsData>>({});
  const [odom, setOdom] = useState<Record<string, OdometryData>>({});
  const [targets, setTargets] = useState<Record<string, { position: { x: number; y: number; z: number } }>>({});
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [rosGraph, setRosGraph] = useState<RosGraphData | null>(null);
  const [wind, setWind] = useState<WindData | null>(null);
  const [formation, setFormation] = useState<FormationState | null>(null);
  const [systemStatus, setSystemStatus] = useState<SystemStatus | null>(null);
  const [controllerParams, setControllerParams] = useState<Record<string, ControllerParams>>({});
  const [simulationEnded, setSimulationEnded] = useState(false);
  const [simulationEndReason, setSimulationEndReason] = useState<string>('');

  // History for charts
  const [metricsHistory, setMetricsHistory] = useState<Record<string, MetricsData[]>>({});
  const [windHistory, setWindHistory] = useState<WindData[]>([]);

  useEffect(() => {
    // Connection status
    const unsubStatus = wsClient.onConnectionStatusChange((isConnected) => {
      setConnected(isConnected);
    });

    // Data updates
    const unsubData = wsClient.onUpdate((dataType, agentId, data) => {
      if (dataType === 'snapshot') {
        handleSnapshot(data as SnapshotData);
      } else if (dataType === 'metrics' && agentId) {
        setMetrics((prev) => ({ ...prev, [agentId]: data }));
        // Add to history
        setMetricsHistory((prev) => {
          const agentHistory = prev[agentId] || [];
          const newHistory = [...agentHistory, data].slice(-MAX_HISTORY);
          return { ...prev, [agentId]: newHistory };
        });
      } else if (dataType === 'odom' && agentId) {
        setOdom((prev) => ({ ...prev, [agentId]: data }));
      } else if (dataType === 'target' && agentId) {
        setTargets((prev) => ({ ...prev, [agentId]: data }));
      } else if (dataType === 'diagnostics' && agentId) {
        const diagData = data as DiagnosticsData;
        setLogs((prev) => [{
          timestamp: diagData.timestamp,
          level: 'info' as const,
          message: `Agent ${agentId} diagnostics updated`,
          agent: agentId
        }, ...prev].slice(0, 100));
      } else if (dataType === 'ros_graph') {
        setRosGraph(data);
      } else if (dataType === 'wind') {
        setWind(data);
        // Add to wind history
        setWindHistory((prev) => [...prev, data].slice(-MAX_HISTORY));
      } else if (dataType === 'formation') {
        setFormation(data);
      } else if (dataType === 'status') {
        setSystemStatus(data);
      } else if (dataType === 'controller_params' && agentId) {
        setControllerParams((prev) => ({ ...prev, [agentId]: data }));
      } else if (dataType === 'simulation_event') {
        const event = data as any;
        if (event.event === 'stopped') {
          setSimulationEnded(true);
          setSimulationEndReason(event.reason || 'unknown');
          setLogs((prev) => [{
            timestamp: event.timestamp || Date.now() / 1000,
            level: 'warning' as const,
            message: `Simulation ended: ${event.reason || 'unknown'}`,
          }, ...prev].slice(0, 100));
        } else if (event.event === 'started') {
          setSimulationEnded(false);
          setSimulationEndReason('');
          setLogs((prev) => [{
            timestamp: event.timestamp || Date.now() / 1000,
            level: 'success' as const,
            message: `Simulation started: ${event.script || 'unknown'}`,
          }, ...prev].slice(0, 100));
        }
      }
    });

    wsClient.connect();

    return () => {
      unsubStatus();
      unsubData();
      wsClient.disconnect();
    };
  }, []);

  // Subscribe to live updates once connected
  useEffect(() => {
    if (connected) {
      wsClient.subscribe(['metrics', 'odom', 'wind', 'formation', 'diagnostics', 'controller_params']);
    }
  }, [connected]);

  const handleSnapshot = (snapshot: SnapshotData) => {
    if (snapshot.metrics) setMetrics(snapshot.metrics);
    if (snapshot.odom) setOdom(snapshot.odom);
    if (snapshot.target) setTargets(snapshot.target);
    if (snapshot.diagnostics) {
      const diagLogs: LogEntry[] = Object.entries(snapshot.diagnostics).map(([agentId, diag]) => ({
        timestamp: diag.timestamp,
        level: 'info' as const,
        message: `Agent ${agentId} diagnostics loaded`,
        agent: agentId
      }));
      setLogs(diagLogs.sort((a, b) => b.timestamp - a.timestamp));
    }
    if (snapshot.ros_graph) setRosGraph(snapshot.ros_graph);
    if (snapshot.wind) setWind(snapshot.wind);
    if (snapshot.formation) setFormation(snapshot.formation);
    if (snapshot.status) setSystemStatus(snapshot.status);
    if (snapshot.controller_params) setControllerParams(snapshot.controller_params);
  };

  return (
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
          rosGraph={rosGraph}
        />
      }
      rightPanel={<RosGraphPanel graphData={rosGraph} controllerParams={controllerParams} />}
      bottomPanel={<DiagnosticsPanel logs={logs} />}
    />
  );
}

export default App;
