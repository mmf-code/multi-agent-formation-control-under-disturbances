/**
 * Simulation Metrics Panel - Robotics Dashboard
 * Design: Foxglove-inspired modular data visualization
 */
import React, { useState, useMemo } from 'react';
import {
  Activity,
  Users,
  BarChart2,
  Map as MapIcon,
  Settings,
  LayoutGrid,
  TrendingUp,
  TrendingDown,
  CheckCircle2,
  Clock,
  Navigation,
  Gauge,
} from 'lucide-react';
import {
  MetricsData,
  WindData,
  FormationState,
  SystemStatus,
  OdometryData,
  ControllerParams,
  RosGraphData,
} from '../api/ws';
import { FormationMap } from './FormationMap';
import { TimeSeries } from './TimeSeries';
import { SystemStatePanel } from './SystemStatePanel';
import { ControllerParamsPanel } from './ControllerParamsPanel';

// ============================================================================
// Types
// ============================================================================

interface SimulationMetricsPanelProps {
  metrics: Record<string, MetricsData>;
  wind: WindData | null;
  formation: FormationState | null;
  status: SystemStatus | null;
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  odomData: Record<string, OdometryData>;
  targetData: Record<string, { position: { x: number; y: number; z: number } }>;
  controllerParams?: Record<string, ControllerParams>;
  rosGraph?: RosGraphData | null;
}

type TabType = 'overview' | 'charts' | 'agents' | 'map' | 'params' | 'system';

// ============================================================================
// Main Component
// ============================================================================

export const SimulationMetricsPanel: React.FC<SimulationMetricsPanelProps> = ({
  metrics,
  wind,
  formation,
  status,
  metricsHistory,
  windHistory,
  odomData,
  targetData,
  controllerParams = {},
}) => {
  const [activeTab, setActiveTab] = useState<TabType>('overview');

  const tabs: { id: TabType; label: string; icon: React.ReactNode }[] = [
    { id: 'overview', label: 'Overview', icon: <LayoutGrid className="w-3.5 h-3.5" /> },
    { id: 'charts', label: 'Analytics', icon: <BarChart2 className="w-3.5 h-3.5" /> },
    { id: 'map', label: 'Formation', icon: <MapIcon className="w-3.5 h-3.5" /> },
    { id: 'agents', label: 'Agents', icon: <Users className="w-3.5 h-3.5" /> },
    { id: 'params', label: 'Params', icon: <Settings className="w-3.5 h-3.5" /> },
    { id: 'system', label: 'System', icon: <Activity className="w-3.5 h-3.5" /> },
  ];

  return (
    <div className="h-full flex flex-col">
      {/* Tab Navigation */}
      <nav className="flex gap-1 mb-3 pb-2" style={{ borderBottom: '1px solid var(--color-border-subtle)' }}>
        {tabs.map((tab) => (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`flex items-center gap-1.5 px-2.5 py-1.5 rounded text-xs font-medium transition-colors ${
              activeTab === tab.id
                ? 'bg-blue-500/15 text-blue-400'
                : 'text-zinc-500 hover:text-zinc-300 hover:bg-zinc-800/50'
            }`}
          >
            {tab.icon}
            <span className="hidden sm:inline">{tab.label}</span>
          </button>
        ))}
      </nav>

      {/* Content */}
      <div className="flex-1 overflow-y-auto overflow-x-hidden">
        {activeTab === 'overview' && (
          <OverviewPanel
            metrics={metrics}
            wind={wind}
            status={status}
            controllerParams={controllerParams}
          />
        )}

        {activeTab === 'charts' && (
          <ChartsPanel
            metricsHistory={metricsHistory}
            windHistory={windHistory}
            controllerParams={controllerParams}
          />
        )}

        {activeTab === 'map' && (
          <div className="panel h-[480px]">
            <FormationMap
              odomData={odomData}
              targetData={targetData}
              formationState={formation}
              selectedAgents={new Set()}
              windData={wind}
              metricsData={metrics}
            />
          </div>
        )}

        {activeTab === 'agents' && (
          <AgentsPanel metrics={metrics} controllerParams={controllerParams} />
        )}

        {activeTab === 'params' && (
          <ControllerParamsPanel controllerParams={controllerParams} />
        )}

        {activeTab === 'system' && (
          <SystemStatePanel metrics={metrics} wind={wind} status={status} />
        )}
      </div>
    </div>
  );
};

// ============================================================================
// Overview Panel
// ============================================================================

const OverviewPanel: React.FC<{
  metrics: Record<string, MetricsData>;
  wind: WindData | null;
  status: SystemStatus | null;
  controllerParams: Record<string, ControllerParams>;
}> = ({ metrics, wind, status, controllerParams }) => {
  const agentCount = Object.keys(metrics).length;
  const settledCount = Object.values(metrics).filter((m) => m.is_settled).length;
  const avgError = agentCount > 0
    ? Object.values(metrics).reduce((sum, m) => sum + m.error_magnitude, 0) / agentCount
    : 0;
  const avgRmse = agentCount > 0
    ? Object.values(metrics).reduce((sum, m) => sum + (m.rmse_total || 0), 0) / agentCount
    : 0;

  const windSpeed = wind ? Math.sqrt(wind.velocity.x ** 2 + wind.velocity.y ** 2) : 0;
  const windDirection = wind ? (Math.atan2(wind.velocity.y, wind.velocity.x) * 180 / Math.PI + 360) % 360 : 0;

  const groupStats = useMemo(() => {
    const groups: Record<string, { count: number; avgError: number; avgRmse: number; settled: number }> = {};
    Object.entries(metrics).forEach(([agentId, m]) => {
      const params = controllerParams[agentId];
      const type = params?.controller_type || 'unknown';
      if (!groups[type]) groups[type] = { count: 0, avgError: 0, avgRmse: 0, settled: 0 };
      groups[type].count++;
      groups[type].avgError += m.error_magnitude;
      groups[type].avgRmse += m.rmse_total || 0;
      if (m.is_settled) groups[type].settled++;
    });
    Object.keys(groups).forEach((type) => {
      if (groups[type].count > 0) {
        groups[type].avgError /= groups[type].count;
        groups[type].avgRmse /= groups[type].count;
      }
    });
    return groups;
  }, [metrics, controllerParams]);

  return (
    <div className="space-y-4">
      {/* Key Metrics */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-2">
        <MetricCard
          label="Agents"
          value={agentCount}
          subValue={`${settledCount} settled`}
          icon={<Users className="w-4 h-4" />}
          color="blue"
        />
        <MetricCard
          label="Avg Error"
          value={`${avgError.toFixed(4)}`}
          subValue="meters"
          icon={avgError < 0.1 ? <TrendingDown className="w-4 h-4" /> : <TrendingUp className="w-4 h-4" />}
          color={avgError < 0.1 ? 'green' : avgError < 0.3 ? 'yellow' : 'red'}
        />
        <MetricCard
          label="Avg RMSE"
          value={`${avgRmse.toFixed(4)}`}
          subValue="meters"
          icon={<Gauge className="w-4 h-4" />}
          color={avgRmse < 0.05 ? 'green' : avgRmse < 0.15 ? 'yellow' : 'red'}
        />
        <MetricCard
          label="Wind"
          value={`${windSpeed.toFixed(2)}`}
          subValue={`${windDirection.toFixed(0)}° m/s`}
          icon={<Navigation className="w-4 h-4" style={{ transform: `rotate(${windDirection}deg)` }} />}
          color="cyan"
        />
      </div>

      {/* Controller Comparison */}
      <div className="panel">
        <div className="panel-header">
          <span className="panel-title">Controller Performance</span>
        </div>
        <div className="panel-content">
          {Object.keys(groupStats).length > 0 ? (
            <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
              {Object.entries(groupStats).map(([type, stats]) => (
                <ControllerCard key={type} type={type} stats={stats} />
              ))}
            </div>
          ) : (
            <div className="py-8 text-center text-zinc-500 text-sm">No controller data</div>
          )}
        </div>
      </div>

      {/* Wind & System Status */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-3">
        {/* Wind Panel */}
        <div className="panel">
          <div className="panel-header">
            <span className="panel-title">Wind Disturbance</span>
          </div>
          <div className="panel-content">
            {wind ? (
              <div className="grid grid-cols-2 gap-3">
                <div className="text-center py-3 rounded" style={{ background: 'var(--color-bg-tertiary)' }}>
                  <div className="text-2xl font-mono font-semibold text-cyan-400">{windSpeed.toFixed(2)}</div>
                  <div className="data-label mt-1">Speed (m/s)</div>
                </div>
                <div className="text-center py-3 rounded" style={{ background: 'var(--color-bg-tertiary)' }}>
                  <div className="flex items-center justify-center gap-1">
                    <Navigation className="w-5 h-5 text-amber-400" style={{ transform: `rotate(${windDirection}deg)` }} />
                    <span className="text-2xl font-mono font-semibold text-amber-400">{windDirection.toFixed(0)}°</span>
                  </div>
                  <div className="data-label mt-1">Direction</div>
                </div>
              </div>
            ) : (
              <div className="py-6 text-center text-zinc-500 text-sm">No wind disturbance</div>
            )}
          </div>
        </div>

        {/* System Status */}
        <div className="panel">
          <div className="panel-header">
            <span className="panel-title">System Status</span>
          </div>
          <div className="panel-content space-y-2">
            <StatusRow label="ROS2 Bridge" active={status?.ros_ok} />
            <StatusRow label="Formation" active={status?.formation_active} />
            <StatusRow label="Wind Publisher" active={status?.wind_active} />
            <div className="pt-2 mt-2 grid grid-cols-2 gap-2" style={{ borderTop: '1px solid var(--color-border-subtle)' }}>
              <div className="flex justify-between text-xs">
                <span className="text-zinc-500">Topics</span>
                <span className="font-mono text-zinc-300">{status?.available_topics?.length || 0}</span>
              </div>
              <div className="flex justify-between text-xs">
                <span className="text-zinc-500">Agents</span>
                <span className="font-mono text-zinc-300">{status?.active_agents?.length || 0}</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Agent Grid */}
      <div className="panel">
        <div className="panel-header">
          <span className="panel-title">Agent Status</span>
        </div>
        <div className="panel-content">
          {Object.keys(metrics).length > 0 ? (
            <div className="grid grid-cols-3 sm:grid-cols-5 md:grid-cols-9 gap-2">
              {Object.entries(metrics).map(([agentId, m]) => {
                const params = controllerParams[agentId];
                const agentNum = parseInt(agentId.replace('agent_', ''), 10);
                return (
                  <AgentDot
                    key={agentId}
                    number={agentNum}
                    error={m.error_magnitude}
                    settled={m.is_settled}
                    type={params?.controller_type}
                  />
                );
              })}
            </div>
          ) : (
            <div className="py-6 text-center text-zinc-500 text-sm">No agents</div>
          )}
        </div>
      </div>
    </div>
  );
};

// ============================================================================
// Charts Panel
// ============================================================================

const ChartsPanel: React.FC<{
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  controllerParams: Record<string, ControllerParams>;
}> = ({ metricsHistory, windHistory, controllerParams }) => {
  const [chartType, setChartType] = useState<'error' | 'metrics' | 'wind' | 'overshoot'>('error');
  const [timeWindow, setTimeWindow] = useState<number>(60);
  const [viewMode, setViewMode] = useState<'individual' | 'aggregated'>('aggregated');
  const [selectedControllers, setSelectedControllers] = useState<Set<string>>(new Set(['pid', 'pid_fuzzy', 'pd']));

  const toggleController = (c: string) => {
    const s = new Set(selectedControllers);
    s.has(c) ? s.delete(c) : s.add(c);
    setSelectedControllers(s);
  };

  return (
    <div className="space-y-3">
      {/* Controls */}
      <div className="panel">
        <div className="panel-content flex flex-wrap items-center gap-2">
          {/* Chart Type */}
          <div className="flex gap-0.5 p-0.5 rounded" style={{ background: 'var(--color-bg-tertiary)' }}>
            {(['error', 'metrics', 'wind', 'overshoot'] as const).map((t) => (
              <button
                key={t}
                onClick={() => setChartType(t)}
                className={`px-2.5 py-1 rounded text-xs font-medium transition-colors ${
                  chartType === t
                    ? 'bg-blue-500/20 text-blue-400'
                    : 'text-zinc-500 hover:text-zinc-300'
                }`}
              >
                {t === 'error' ? 'Error' : t === 'metrics' ? 'IAE/ITAE' : t === 'wind' ? 'Wind' : 'Overshoot'}
              </button>
            ))}
          </div>

          {chartType !== 'wind' && (
            <button
              onClick={() => setViewMode(viewMode === 'individual' ? 'aggregated' : 'individual')}
              className={`px-2.5 py-1 rounded text-xs font-medium border transition-colors ${
                viewMode === 'aggregated'
                  ? 'bg-green-500/10 text-green-400 border-green-500/30'
                  : 'text-zinc-400 border-zinc-700 hover:border-zinc-600'
              }`}
            >
              {viewMode === 'aggregated' ? 'By Controller' : 'Individual'}
            </button>
          )}

          {viewMode === 'aggregated' && chartType !== 'wind' && (
            <div className="flex gap-1">
              {['pid', 'pid_fuzzy', 'pd'].map((c) => (
                <button
                  key={c}
                  onClick={() => toggleController(c)}
                  className={`px-2 py-1 rounded text-xs font-medium border transition-colors ${
                    selectedControllers.has(c)
                      ? c === 'pid_fuzzy'
                        ? 'bg-green-500/10 text-green-400 border-green-500/30'
                        : c === 'pid'
                        ? 'bg-blue-500/10 text-blue-400 border-blue-500/30'
                        : 'bg-amber-500/10 text-amber-400 border-amber-500/30'
                      : 'text-zinc-600 border-zinc-800'
                  }`}
                >
                  {c === 'pid_fuzzy' ? 'Hybrid' : c.toUpperCase()}
                </button>
              ))}
            </div>
          )}

          <select
            value={timeWindow}
            onChange={(e) => setTimeWindow(Number(e.target.value))}
            className="ml-auto text-xs px-2 py-1 rounded"
            style={{ background: 'var(--color-bg-tertiary)', border: '1px solid var(--color-border-subtle)', color: 'var(--color-text-secondary)' }}
          >
            <option value={30}>30s</option>
            <option value={60}>60s</option>
            <option value={120}>2min</option>
            <option value={0}>All</option>
          </select>
        </div>
      </div>

      {/* Chart */}
      <div className="panel h-72">
        <TimeSeries
          metricsHistory={metricsHistory}
          windHistory={windHistory}
          selectedAgent={null}
          chartType={chartType}
          timeWindow={timeWindow}
          viewMode={viewMode}
          controllerParams={controllerParams}
          selectedControllers={selectedControllers}
        />
      </div>
    </div>
  );
};

// ============================================================================
// Agents Panel
// ============================================================================

const AgentsPanel: React.FC<{
  metrics: Record<string, MetricsData>;
  controllerParams: Record<string, ControllerParams>;
}> = ({ metrics, controllerParams }) => {
  const [sortBy, setSortBy] = useState<'name' | 'error' | 'rmse'>('name');

  const sortedAgents = useMemo(() => {
    const entries = Object.entries(metrics);
    entries.sort((a, b) => {
      if (sortBy === 'error') return a[1].error_magnitude - b[1].error_magnitude;
      if (sortBy === 'rmse') return (a[1].rmse_total || 0) - (b[1].rmse_total || 0);
      return a[0].localeCompare(b[0]);
    });
    return entries;
  }, [metrics, sortBy]);

  return (
    <div className="space-y-3">
      {/* Sort Controls */}
      <div className="flex items-center gap-2">
        <span className="text-xs text-zinc-500">Sort:</span>
        {(['name', 'error', 'rmse'] as const).map((f) => (
          <button
            key={f}
            onClick={() => setSortBy(f)}
            className={`px-2 py-1 rounded text-xs font-medium transition-colors ${
              sortBy === f ? 'bg-blue-500/15 text-blue-400' : 'text-zinc-500 hover:text-zinc-300'
            }`}
          >
            {f === 'name' ? 'Name' : f.toUpperCase()}
          </button>
        ))}
        <span className="ml-auto text-xs text-zinc-600">{sortedAgents.length} agents</span>
      </div>

      {/* Table */}
      <div className="panel overflow-hidden">
        <table className="w-full text-sm">
          <thead>
            <tr style={{ borderBottom: '1px solid var(--color-border-subtle)' }}>
              <th className="px-3 py-2 text-left text-xs font-medium text-zinc-500">Agent</th>
              <th className="px-3 py-2 text-left text-xs font-medium text-zinc-500">Controller</th>
              <th className="px-3 py-2 text-right text-xs font-medium text-zinc-500">Error</th>
              <th className="px-3 py-2 text-right text-xs font-medium text-zinc-500">RMSE</th>
              <th className="px-3 py-2 text-right text-xs font-medium text-zinc-500">Settling</th>
              <th className="px-3 py-2 text-center text-xs font-medium text-zinc-500">Status</th>
            </tr>
          </thead>
          <tbody>
            {sortedAgents.map(([agentId, m]) => {
              const params = controllerParams[agentId];
              const agentNum = parseInt(agentId.replace('agent_', ''), 10);
              return (
                <tr key={agentId} className="hover:bg-zinc-800/30" style={{ borderBottom: '1px solid var(--color-border-subtle)' }}>
                  <td className="px-3 py-2">
                    <div className="flex items-center gap-2">
                      <span className="w-5 h-5 rounded text-xs font-mono flex items-center justify-center"
                            style={{ background: 'var(--color-bg-tertiary)' }}>
                        {agentNum}
                      </span>
                      <span className="text-zinc-300">{agentId}</span>
                    </div>
                  </td>
                  <td className="px-3 py-2">
                    <ControllerBadge type={params?.controller_type} />
                  </td>
                  <td className="px-3 py-2 text-right">
                    <span className={`font-mono ${getErrorColor(m.error_magnitude)}`}>
                      {m.error_magnitude.toFixed(4)}
                    </span>
                  </td>
                  <td className="px-3 py-2 text-right font-mono text-zinc-400">{(m.rmse_total || 0).toFixed(4)}</td>
                  <td className="px-3 py-2 text-right font-mono text-zinc-400">{m.settling_time.toFixed(2)}s</td>
                  <td className="px-3 py-2 text-center">
                    {m.is_settled ? (
                      <span className="inline-flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-medium bg-green-500/10 text-green-400">
                        <CheckCircle2 className="w-3 h-3" /> Settled
                      </span>
                    ) : (
                      <span className="inline-flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-medium bg-amber-500/10 text-amber-400">
                        <Clock className="w-3 h-3" /> Moving
                      </span>
                    )}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
        {sortedAgents.length === 0 && (
          <div className="py-12 text-center text-zinc-500 text-sm">No agents</div>
        )}
      </div>
    </div>
  );
};

// ============================================================================
// Sub-Components
// ============================================================================

const MetricCard: React.FC<{
  label: string;
  value: string | number;
  subValue?: string;
  icon: React.ReactNode;
  color: 'blue' | 'green' | 'yellow' | 'red' | 'cyan';
}> = ({ label, value, subValue, icon, color }) => {
  const colors = {
    blue: 'text-blue-400',
    green: 'text-green-400',
    yellow: 'text-amber-400',
    red: 'text-red-400',
    cyan: 'text-cyan-400',
  };

  return (
    <div className="panel p-3">
      <div className="flex items-center justify-between mb-1">
        <span className="data-label">{label}</span>
        <span className={`${colors[color]} opacity-60`}>{icon}</span>
      </div>
      <div className={`text-xl font-mono font-semibold ${colors[color]}`}>{value}</div>
      {subValue && <div className="text-[11px] text-zinc-500 mt-0.5">{subValue}</div>}
    </div>
  );
};

const ControllerCard: React.FC<{
  type: string;
  stats: { count: number; avgError: number; avgRmse: number; settled: number };
}> = ({ type, stats }) => {
  const config: Record<string, { color: string; label: string }> = {
    pid_fuzzy: { color: 'green', label: 'PID + Fuzzy' },
    hybrid: { color: 'green', label: 'Hybrid' },
    pid: { color: 'blue', label: 'PID' },
    pd: { color: 'amber', label: 'PD' },
  };
  const { color, label } = config[type] || { color: 'zinc', label: type.toUpperCase() };

  return (
    <div className="p-3 rounded" style={{ background: 'var(--color-bg-tertiary)' }}>
      <div className="flex items-center justify-between mb-2">
        <span className={`text-sm font-semibold text-${color}-400`}>{label}</span>
        <span className="text-[11px] text-zinc-500">{stats.count} agents</span>
      </div>
      <div className="grid grid-cols-2 gap-2">
        <div>
          <div className="data-label">Avg Error</div>
          <div className={`text-base font-mono font-semibold ${getErrorColor(stats.avgError)}`}>
            {stats.avgError.toFixed(4)}
          </div>
        </div>
        <div>
          <div className="data-label">Settled</div>
          <div className="text-base font-mono font-semibold text-zinc-200">
            {stats.settled}/{stats.count}
          </div>
        </div>
      </div>
      {/* Progress bar */}
      <div className="mt-2 h-1 rounded-full overflow-hidden" style={{ background: 'var(--color-bg-primary)' }}>
        <div
          className={`h-full rounded-full ${stats.settled === stats.count ? 'bg-green-500' : 'bg-blue-500'}`}
          style={{ width: `${(stats.settled / stats.count) * 100}%` }}
        />
      </div>
    </div>
  );
};

const ControllerBadge: React.FC<{ type?: string }> = ({ type }) => {
  if (!type) return <span className="text-zinc-600">-</span>;
  const config: Record<string, string> = {
    pid_fuzzy: 'bg-green-500/10 text-green-400 border-green-500/30',
    hybrid: 'bg-green-500/10 text-green-400 border-green-500/30',
    pid: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
    pd: 'bg-amber-500/10 text-amber-400 border-amber-500/30',
  };
  const label = type === 'pid_fuzzy' || type === 'hybrid' ? 'Hybrid' : type.toUpperCase();
  return (
    <span className={`inline-flex px-1.5 py-0.5 rounded text-[10px] font-medium border ${config[type] || 'bg-zinc-800 text-zinc-500 border-zinc-700'}`}>
      {label}
    </span>
  );
};

const StatusRow: React.FC<{ label: string; active?: boolean }> = ({ label, active }) => (
  <div className="flex justify-between items-center py-1">
    <span className="text-sm text-zinc-400">{label}</span>
    <span className={`flex items-center gap-1.5 text-xs font-medium ${active ? 'text-green-400' : 'text-zinc-600'}`}>
      <span className={`status-dot ${active ? 'status-dot-success' : ''}`} style={{ background: active ? undefined : 'var(--color-border-default)' }} />
      {active ? 'Active' : 'Inactive'}
    </span>
  </div>
);

const AgentDot: React.FC<{
  number: number;
  error: number;
  settled: boolean;
  type?: string;
}> = ({ number, error, settled, type }) => {
  const borderColors: Record<string, string> = {
    pid_fuzzy: 'border-green-500',
    hybrid: 'border-green-500',
    pid: 'border-blue-500',
    pd: 'border-amber-500',
  };

  return (
    <div
      className={`relative w-9 h-9 rounded border-2 flex items-center justify-center cursor-default ${borderColors[type || ''] || 'border-zinc-700'}`}
      style={{ background: 'var(--color-bg-tertiary)' }}
      title={`Agent ${number}: ${error.toFixed(3)}m`}
    >
      <span className="text-xs font-mono font-semibold text-zinc-400">{number}</span>
      <span className={`absolute -top-1 -right-1 w-2.5 h-2.5 rounded-full ${getErrorBg(error)} ${!settled ? 'animate-pulse' : ''}`} />
    </div>
  );
};

const getErrorColor = (e: number) => e < 0.1 ? 'text-green-400' : e < 0.3 ? 'text-amber-400' : 'text-red-400';
const getErrorBg = (e: number) => e < 0.1 ? 'bg-green-500' : e < 0.3 ? 'bg-amber-500' : 'bg-red-500';

export default SimulationMetricsPanel;
