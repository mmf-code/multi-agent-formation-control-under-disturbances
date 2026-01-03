/**
 * Dashboard View - Main summary panel with all key metrics
 */
import React from 'react';
import {
  Activity,
  Wind,
  Users,
  Radio,
  AlertTriangle,
  CheckCircle,
  XCircle,
  Cpu,
  Zap,
} from 'lucide-react';
import {
  useMetrics,
  useWind,
  useSystemStatus,
  useRosGraph,
  useGroupStats,
  useAgentCount,
  useSettledCount,
  useAverageError,
} from '../../store';
import { StatusCard, Panel, Badge, ControllerBadge, MetricDisplay, ProgressBar } from '../ui';
import type { ControllerType } from '../../types';

// ============================================================================
// Main Component
// ============================================================================

export const DashboardView: React.FC = () => {
  const metrics = useMetrics();
  const wind = useWind();
  const status = useSystemStatus();
  const rosGraph = useRosGraph();
  const groupStats = useGroupStats();
  const agentCount = useAgentCount();
  const settledCount = useSettledCount();
  const avgError = useAverageError();

  // Topic stats from ROS graph
  const topicStats = React.useMemo(() => {
    if (!rosGraph) return { total: 0, nodes: 0 };
    return {
      total: rosGraph.edges?.length || 0,
      nodes: rosGraph.nodes?.length || 0,
    };
  }, [rosGraph]);

  // Error stats
  const errorStats = React.useMemo(() => {
    const entries = Object.entries(metrics);
    if (entries.length === 0) return { max: 0, min: 0, maxAgent: '-' };

    let max = 0;
    let maxAgent = '-';
    let min = Infinity;

    entries.forEach(([id, m]) => {
      if (m.error_magnitude > max) {
        max = m.error_magnitude;
        maxAgent = id;
      }
      if (m.error_magnitude < min) {
        min = m.error_magnitude;
      }
    });

    return { max, min: min === Infinity ? 0 : min, maxAgent };
  }, [metrics]);

  // Wind speed and direction
  const windSpeed = wind
    ? Math.sqrt(wind.velocity.x ** 2 + wind.velocity.y ** 2)
    : 0;
  const windDirection = wind
    ? (Math.atan2(wind.velocity.y, wind.velocity.x) * 180 / Math.PI + 360) % 360
    : 0;

  return (
    <div className="space-y-4 animate-fade-in">
      {/* System Status Row */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-3">
        <StatusCard
          label="System"
          value={status?.ros_ok ? 'Online' : 'Offline'}
          icon={status?.ros_ok ? <CheckCircle className="w-4 h-4" /> : <XCircle className="w-4 h-4" />}
          color={status?.ros_ok ? 'green' : 'red'}
          pulse={status?.ros_ok}
        />
        <StatusCard
          label="Formation"
          value={status?.formation_active ? 'Active' : 'Idle'}
          icon={<Activity className="w-4 h-4" />}
          color={status?.formation_active ? 'blue' : 'gray'}
          pulse={status?.formation_active}
        />
        <StatusCard
          label="Agents"
          value={`${settledCount}/${agentCount}`}
          subtext="settled"
          icon={<Users className="w-4 h-4" />}
          color="purple"
        />
        <StatusCard
          label="Topics"
          value={topicStats.total.toString()}
          subtext={`${topicStats.nodes} nodes`}
          icon={<Radio className="w-4 h-4" />}
          color="cyan"
        />
      </div>

      {/* Controller Groups Comparison */}
      <Panel
        title="Controller Performance Comparison"
        icon={<Cpu className="w-4 h-4" />}
      >
        <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-4">
          {Object.entries(groupStats).map(([type, stats]) => (
            <ControllerGroupCard
              key={type}
              type={type as ControllerType}
              count={stats.count}
              avgError={stats.avgError}
              settled={stats.settled}
            />
          ))}
          {Object.keys(groupStats).length === 0 && (
            <div className="col-span-full text-center text-zinc-500 py-8">
              <Cpu className="w-8 h-8 mx-auto mb-2 opacity-50" />
              <p>No controller data available</p>
              <p className="text-xs mt-1">Start a simulation to see performance data</p>
            </div>
          )}
        </div>
      </Panel>

      {/* Error Summary & Wind */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Error Summary */}
        <Panel
          title="Error Summary"
          icon={<AlertTriangle className="w-4 h-4 text-amber-400" />}
        >
          <div className="space-y-3">
            <div className="flex justify-between items-center">
              <span className="text-xs text-zinc-500">Average Error</span>
              <span className={`font-mono font-semibold ${
                avgError < 0.3 ? 'text-emerald-400' :
                avgError < 0.7 ? 'text-amber-400' : 'text-red-400'
              }`}>
                {avgError.toFixed(3)} m
              </span>
            </div>
            <ProgressBar
              value={avgError}
              max={1}
              color={avgError < 0.3 ? 'green' : avgError < 0.7 ? 'yellow' : 'red'}
              size="sm"
            />

            <div className="grid grid-cols-2 gap-4 pt-2 border-t border-zinc-800">
              <MetricDisplay
                label="Max Error"
                value={errorStats.max}
                unit="m"
                color="red"
                highlight
              />
              <MetricDisplay
                label="Min Error"
                value={errorStats.min}
                unit="m"
                color="green"
                highlight
              />
            </div>

            {errorStats.maxAgent !== '-' && (
              <div className="flex items-center justify-between pt-2 border-t border-zinc-800">
                <span className="text-xs text-zinc-500">Worst Performing Agent</span>
                <Badge color="red" size="sm">{errorStats.maxAgent}</Badge>
              </div>
            )}
          </div>
        </Panel>

        {/* Wind Conditions */}
        <Panel
          title="Wind Conditions"
          icon={<Wind className="w-4 h-4 text-cyan-400" />}
        >
          {wind ? (
            <div className="space-y-3">
              <div className="grid grid-cols-2 gap-4">
                <div className="text-center p-3 bg-zinc-800/50 rounded-lg">
                  <div className="text-2xl font-semibold font-mono text-cyan-400">
                    {windSpeed.toFixed(2)}
                  </div>
                  <div className="text-xs text-zinc-500 mt-1">Speed (m/s)</div>
                </div>
                <div className="text-center p-3 bg-zinc-800/50 rounded-lg">
                  <div className="text-2xl font-semibold font-mono text-amber-400">
                    {windDirection.toFixed(0)}°
                  </div>
                  <div className="text-xs text-zinc-500 mt-1">Direction</div>
                </div>
              </div>

              <div className="grid grid-cols-2 gap-3 pt-2 border-t border-zinc-800">
                <MetricDisplay
                  label="Force X"
                  value={wind.force?.x || 0}
                  unit="N"
                  precision={2}
                />
                <MetricDisplay
                  label="Force Y"
                  value={wind.force?.y || 0}
                  unit="N"
                  precision={2}
                />
              </div>
            </div>
          ) : (
            <div className="text-center py-6 text-zinc-500">
              <Wind className="w-8 h-8 mx-auto mb-2 opacity-50" />
              <p>No wind data</p>
              <p className="text-xs mt-1">Wind disturbances will appear here when active</p>
            </div>
          )}
        </Panel>
      </div>

      {/* ROS2 Topics Health */}
      <Panel
        title="ROS2 System Health"
        icon={<Radio className="w-4 h-4 text-purple-400" />}
      >
        <div className="grid grid-cols-3 gap-4">
          <div className="text-center p-4 bg-purple-950/30 rounded-lg">
            <div className="text-3xl font-semibold text-purple-400 font-mono">{topicStats.nodes}</div>
            <div className="text-xs text-zinc-500 mt-1">Active Nodes</div>
          </div>
          <div className="text-center p-4 bg-cyan-950/30 rounded-lg">
            <div className="text-3xl font-semibold text-cyan-400 font-mono">{topicStats.total}</div>
            <div className="text-xs text-zinc-500 mt-1">Topic Connections</div>
          </div>
          <div className="text-center p-4 bg-emerald-950/30 rounded-lg">
            <div className="text-3xl font-semibold text-emerald-400 font-mono flex items-center justify-center gap-1">
              <Zap className="w-5 h-5" />
              {agentCount * 3}
            </div>
            <div className="text-xs text-zinc-500 mt-1">Est. Messages/s</div>
          </div>
        </div>
      </Panel>
    </div>
  );
};

// ============================================================================
// Controller Group Card
// ============================================================================

interface ControllerGroupCardProps {
  type: ControllerType;
  count: number;
  avgError: number;
  settled: number;
}

const ControllerGroupCard: React.FC<ControllerGroupCardProps> = ({
  type,
  count,
  avgError,
  settled,
}) => {
  const errorColor = avgError < 0.3 ? 'text-emerald-400' :
                     avgError < 0.7 ? 'text-amber-400' : 'text-red-400';

  return (
    <div className="bg-zinc-900/50 rounded-lg p-4 border border-zinc-800">
      <div className="flex items-center justify-between mb-3">
        <ControllerBadge type={type} size="sm" />
        <span className="text-xs text-zinc-500">{count} agents</span>
      </div>

      <div className="grid grid-cols-2 gap-4">
        <div>
          <div className="text-xs text-zinc-500 mb-1">Avg Error</div>
          <div className={`font-mono font-semibold text-lg ${errorColor}`}>
            {avgError.toFixed(3)}
            <span className="text-xs text-zinc-500 ml-1">m</span>
          </div>
        </div>
        <div>
          <div className="text-xs text-zinc-500 mb-1">Settled</div>
          <div className="font-mono font-semibold text-lg text-zinc-200">
            {settled}
            <span className="text-xs text-zinc-500">/{count}</span>
          </div>
        </div>
      </div>

      <div className="mt-3">
        <ProgressBar
          value={settled}
          max={count}
          color={settled === count ? 'green' : 'blue'}
          size="sm"
        />
      </div>
    </div>
  );
};

export default DashboardView;
