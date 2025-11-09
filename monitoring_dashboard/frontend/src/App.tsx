/**
 * Main App Component
 * Multi-Agent Formation Control Monitoring Dashboard
 */
import React, { useState, useEffect, useCallback } from 'react';
import { Activity } from 'lucide-react';
import {
  wsClient,
  MetricsData,
  OdometryData,
  WindData,
  FormationState,
  SystemStatus,
  SnapshotData,
} from './api/ws';
import { KpiCards } from './components/KpiCards';
import { TopicStatus } from './components/TopicStatus';
import { TimeSeries } from './components/TimeSeries';
import { FormationMap } from './components/FormationMap';
import { EventLog, LogEntry } from './components/EventLog';

function App() {
  // State
  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState<SystemStatus | null>(null);

  // Latest data
  const [metrics, setMetrics] = useState<Record<string, MetricsData>>({});
  const [odom, setOdom] = useState<Record<string, OdometryData>>({});
  const [targets, setTargets] = useState<Record<string, any>>({});
  const [wind, setWind] = useState<WindData | null>(null);
  const [formation, setFormation] = useState<FormationState | null>(null);

  // History data
  const [metricsHistory, setMetricsHistory] = useState<Record<string, MetricsData[]>>({});
  const [windHistory, setWindHistory] = useState<WindData[]>([]);

  // UI state
  const [selectedTopics, setSelectedTopics] = useState<Set<string>>(
    new Set(['metrics', 'odom', 'wind', 'formation'])
  );
  const [selectedAgents, setSelectedAgents] = useState<Set<string>>(new Set());
  const [selectedAgent, setSelectedAgent] = useState<string | null>(null);
  const [chartType, setChartType] = useState<'error' | 'metrics' | 'wind' | 'overshoot'>('error');

  // Event log
  const [logs, setLogs] = useState<LogEntry[]>([]);

  const addLog = useCallback((level: LogEntry['level'], message: string, agent?: string) => {
    setLogs((prev) => [
      ...prev,
      { timestamp: Date.now() / 1000, level, message, agent },
    ]);
  }, []);

  // WebSocket connection
  useEffect(() => {
    wsClient.connect();

    const unsubscribe = wsClient.onUpdate((dataType, agentId, data) => {
      if (dataType === 'snapshot') {
        handleSnapshot(data as SnapshotData);
      } else if (dataType === 'metrics' && agentId) {
        handleMetricsUpdate(agentId, data);
      } else if (dataType === 'odom' && agentId) {
        handleOdomUpdate(agentId, data);
      } else if (dataType === 'wind') {
        handleWindUpdate(data);
      } else if (dataType === 'formation') {
        handleFormationUpdate(data);
      } else if (dataType === 'target' && agentId) {
        setTargets((prev) => ({ ...prev, [agentId]: data }));
      }
    });

    // Connection status check
    const statusInterval = setInterval(() => {
      setConnected(wsClient.connected);
    }, 1000);

    return () => {
      unsubscribe();
      clearInterval(statusInterval);
      wsClient.disconnect();
    };
  }, []);

  // Subscribe to topics when selection changes
  useEffect(() => {
    if (wsClient.connected) {
      wsClient.subscribe(
        Array.from(selectedTopics),
        selectedAgents.size > 0 ? Array.from(selectedAgents) : undefined
      );
    }
  }, [selectedTopics, selectedAgents]);

  // Handlers
  const handleSnapshot = (snapshot: SnapshotData) => {
    setMetrics(snapshot.metrics);
    setOdom(snapshot.odom);
    setTargets(snapshot.target);
    if (snapshot.wind) setWind(snapshot.wind);
    if (snapshot.formation) setFormation(snapshot.formation);
    if (snapshot.status) setStatus(snapshot.status);

    addLog('info', 'Dashboard connected and synchronized');
  };

  const handleMetricsUpdate = (agentId: string, data: MetricsData) => {
    setMetrics((prev) => ({ ...prev, [agentId]: data }));

    // Add to history
    setMetricsHistory((prev) => {
      const history = prev[agentId] || [];
      return {
        ...prev,
        [agentId]: [...history, data].slice(-500), // Keep last 500 points
      };
    });

    // Log settling events
    if (data.is_settled && (!metrics[agentId] || !metrics[agentId].is_settled)) {
      addLog('success', 'Agent settled within threshold', agentId);
    }
  };

  const handleOdomUpdate = (agentId: string, data: OdometryData) => {
    setOdom((prev) => ({ ...prev, [agentId]: data }));
  };

  const handleWindUpdate = (data: WindData) => {
    setWind(data);
    setWindHistory((prev) => [...prev, data].slice(-500));
  };

  const handleFormationUpdate = (data: FormationState) => {
    const prevShape = formation?.shape;
    setFormation(data);

    if (prevShape && prevShape !== data.shape) {
      addLog('info', `Formation changed to ${data.shape}`);
    }
  };

  // Topic/Agent selection handlers
  const handleTopicToggle = (topic: string) => {
    setSelectedTopics((prev) => {
      const next = new Set(prev);
      if (next.has(topic)) {
        next.delete(topic);
      } else {
        next.add(topic);
      }
      return next;
    });
  };

  const handleAgentToggle = (agent: string) => {
    setSelectedAgents((prev) => {
      const next = new Set(prev);
      if (next.has(agent)) {
        next.delete(agent);
        if (selectedAgent === agent) setSelectedAgent(null);
      } else {
        next.add(agent);
      }
      return next;
    });
  };

  return (
    <div className="min-h-screen bg-gray-900 text-white">
      {/* Header */}
      <header className="bg-gray-800 border-b border-gray-700 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <Activity className="w-8 h-8 text-blue-400" />
            <div>
              <h1 className="text-2xl font-bold">Formation Control Monitor</h1>
              <p className="text-sm text-gray-400">Real-time multi-agent simulation dashboard</p>
            </div>
          </div>

          <div className="flex items-center space-x-4">
            <div className={`flex items-center space-x-2 px-3 py-1 rounded-lg ${
              connected ? 'bg-green-900' : 'bg-red-900'
            }`}>
              <div className={`w-2 h-2 rounded-full ${
                connected ? 'bg-green-400 animate-pulse' : 'bg-red-400'
              }`}></div>
              <span className={`text-sm font-medium ${
                connected ? 'text-green-300' : 'text-red-300'
              }`}>
                {connected ? 'Connected' : 'Disconnected'}
              </span>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="p-6 space-y-6">
        {/* Topic & Agent Selection */}
        <TopicStatus
          status={status}
          selectedTopics={selectedTopics}
          selectedAgents={selectedAgents}
          onTopicToggle={handleTopicToggle}
          onAgentToggle={handleAgentToggle}
        />

        {/* KPI Cards */}
        {selectedTopics.has('metrics') && Object.keys(metrics).length > 0 && (
          <div>
            <h2 className="text-xl font-semibold mb-4">Performance Metrics</h2>
            <KpiCards metrics={metrics} selectedAgent={selectedAgent} />
          </div>
        )}

        {/* Formation Map */}
        {selectedTopics.has('odom') && Object.keys(odom).length > 0 && (
          <FormationMap
            odomData={odom}
            targetData={targets}
            formationState={formation}
            selectedAgents={selectedAgents}
          />
        )}

        {/* Charts */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Chart Type Selector */}
          <div className="lg:col-span-2">
            <div className="flex items-center space-x-2">
              <span className="text-sm text-gray-400">Chart Type:</span>
              <div className="flex space-x-2">
                {(['error', 'metrics', 'wind', 'overshoot'] as const).map((type) => (
                  <button
                    key={type}
                    onClick={() => setChartType(type)}
                    className={`px-4 py-2 rounded-lg text-sm font-medium transition-all ${
                      chartType === type
                        ? 'bg-blue-600 text-white'
                        : 'bg-gray-700 text-gray-400 hover:bg-gray-600'
                    }`}
                  >
                    {type === 'error' && 'Position Error'}
                    {type === 'metrics' && 'IAE/ITAE'}
                    {type === 'wind' && 'Wind Velocity'}
                    {type === 'overshoot' && 'Overshoot & RMSE'}
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* Time Series Chart */}
          {(chartType === 'wind' ? selectedTopics.has('wind') : selectedTopics.has('metrics')) && (
            <div className="lg:col-span-2">
              <TimeSeries
                metricsHistory={metricsHistory}
                windHistory={windHistory}
                selectedAgent={selectedAgent}
                chartType={chartType}
              />
            </div>
          )}
        </div>

        {/* Event Log */}
        <EventLog logs={logs} />
      </main>
    </div>
  );
}

export default App;
