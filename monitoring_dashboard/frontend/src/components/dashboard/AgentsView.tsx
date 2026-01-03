/**
 * Agents View - List of all agents with detailed metrics
 */
import React, { useState, useMemo } from 'react';
import { Users, ArrowUpDown, Filter, CheckCircle, Clock, Crosshair } from 'lucide-react';
import { useMetrics, useControllerParams } from '../../store';
import { Badge, ControllerBadge, EmptyState } from '../ui';
import { getAgentDisplayName, type ControllerType } from '../../types';

// ============================================================================
// Types
// ============================================================================

type SortField = 'name' | 'error' | 'rmse' | 'settling';
type SortDirection = 'asc' | 'desc';

// ============================================================================
// Main Component
// ============================================================================

export const AgentsView: React.FC = () => {
  const metrics = useMetrics();
  const controllerParams = useControllerParams();

  const [sortField, setSortField] = useState<SortField>('name');
  const [sortDirection, setSortDirection] = useState<SortDirection>('asc');
  const [filterGroup, setFilterGroup] = useState<string>('all');
  const [expandedAgent, setExpandedAgent] = useState<string | null>(null);

  // Get unique controller types
  const controllerTypes = useMemo(() => {
    const types = new Set<string>();
    Object.values(controllerParams).forEach((p) => types.add(p.controller_type));
    return Array.from(types);
  }, [controllerParams]);

  // Sort and filter agents
  const sortedAgents = useMemo(() => {
    let entries = Object.entries(metrics);

    // Filter
    if (filterGroup !== 'all') {
      entries = entries.filter(([id]) => {
        const params = controllerParams[id];
        return params?.controller_type === filterGroup;
      });
    }

    // Sort
    entries.sort((a, b) => {
      let comparison = 0;
      switch (sortField) {
        case 'name':
          comparison = a[0].localeCompare(b[0]);
          break;
        case 'error':
          comparison = a[1].error_magnitude - b[1].error_magnitude;
          break;
        case 'rmse':
          comparison = (a[1].rmse_total || 0) - (b[1].rmse_total || 0);
          break;
        case 'settling':
          comparison = a[1].settling_time - b[1].settling_time;
          break;
      }
      return sortDirection === 'asc' ? comparison : -comparison;
    });

    return entries;
  }, [metrics, controllerParams, sortField, sortDirection, filterGroup]);

  const toggleSort = (field: SortField) => {
    if (sortField === field) {
      setSortDirection((d) => (d === 'asc' ? 'desc' : 'asc'));
    } else {
      setSortField(field);
      setSortDirection('asc');
    }
  };

  return (
    <div className="space-y-4 animate-fade-in">
      {/* Controls */}
      <div className="flex flex-wrap items-center gap-3">
        {/* Sort Buttons */}
        <div className="flex items-center gap-1 bg-zinc-900/50 rounded-lg p-1 border border-zinc-800">
          <SortButton
            label="Name"
            field="name"
            currentField={sortField}
            direction={sortDirection}
            onClick={toggleSort}
          />
          <SortButton
            label="Error"
            field="error"
            currentField={sortField}
            direction={sortDirection}
            onClick={toggleSort}
          />
          <SortButton
            label="RMSE"
            field="rmse"
            currentField={sortField}
            direction={sortDirection}
            onClick={toggleSort}
          />
          <SortButton
            label="Settling"
            field="settling"
            currentField={sortField}
            direction={sortDirection}
            onClick={toggleSort}
          />
        </div>

        {/* Filter Dropdown */}
        <div className="flex items-center gap-2">
          <Filter className="w-4 h-4 text-zinc-500" />
          <select
            value={filterGroup}
            onChange={(e) => setFilterGroup(e.target.value)}
            className="bg-zinc-900 border border-zinc-800 rounded-lg px-3 py-1.5 text-xs text-zinc-300 focus:border-blue-500 focus:outline-none"
          >
            <option value="all">All Controllers</option>
            {controllerTypes.map((type) => (
              <option key={type} value={type}>
                {type === 'pid_fuzzy' ? 'PID + Fuzzy' : type.toUpperCase()}
              </option>
            ))}
          </select>
        </div>

        {/* Count */}
        <div className="ml-auto text-xs text-zinc-500">
          {sortedAgents.length} agent{sortedAgents.length !== 1 ? 's' : ''}
        </div>
      </div>

      {/* Agent List */}
      <div className="space-y-2">
        {sortedAgents.map(([id, m]) => {
          const params = controllerParams[id];
          const isExpanded = expandedAgent === id;

          return (
            <AgentCard
              key={id}
              agentId={id}
              metrics={m}
              controllerType={params?.controller_type as ControllerType}
              isExpanded={isExpanded}
              onToggle={() => setExpandedAgent(isExpanded ? null : id)}
            />
          );
        })}

        {sortedAgents.length === 0 && (
          <EmptyState
            icon={<Users className="w-12 h-12" />}
            title="No agents active"
            description="Start a simulation to see agent metrics"
          />
        )}
      </div>
    </div>
  );
};

// ============================================================================
// Sort Button
// ============================================================================

interface SortButtonProps {
  label: string;
  field: SortField;
  currentField: SortField;
  direction: SortDirection;
  onClick: (field: SortField) => void;
}

const SortButton: React.FC<SortButtonProps> = ({
  label,
  field,
  currentField,
  direction,
  onClick,
}) => {
  const isActive = currentField === field;

  return (
    <button
      onClick={() => onClick(field)}
      className={`flex items-center gap-1 px-2.5 py-1 rounded-md text-xs font-medium transition-all ${
        isActive
          ? 'bg-blue-600/20 text-blue-400'
          : 'text-zinc-400 hover:text-zinc-200 hover:bg-zinc-800'
      }`}
    >
      {label}
      {isActive && (
        <ArrowUpDown
          className={`w-3 h-3 transition-transform ${direction === 'desc' ? 'rotate-180' : ''}`}
        />
      )}
    </button>
  );
};

// ============================================================================
// Agent Card
// ============================================================================

interface AgentCardProps {
  agentId: string;
  metrics: {
    error_magnitude: number;
    rmse_total: number;
    rmse_x: number;
    rmse_y: number;
    rmse_z: number;
    settling_time: number;
    is_settled: boolean;
    iae_x: number;
    iae_y: number;
    itae_x: number;
    itae_y: number;
    max_overshoot_x: number;
    max_overshoot_y: number;
    current_x: number;
    current_y: number;
    current_z: number;
    target_x: number;
    target_y: number;
    target_z: number;
  };
  controllerType?: ControllerType;
  isExpanded: boolean;
  onToggle: () => void;
}

const AgentCard: React.FC<AgentCardProps> = ({
  agentId,
  metrics,
  controllerType,
  isExpanded,
  onToggle,
}) => {
  const displayName = getAgentDisplayName(agentId);
  const errorColor = metrics.error_magnitude < 0.3 ? 'text-emerald-400' :
                     metrics.error_magnitude < 0.7 ? 'text-amber-400' : 'text-red-400';

  return (
    <div
      className={`bg-zinc-900/50 rounded-lg border transition-all ${
        isExpanded ? 'border-blue-500/50' : 'border-zinc-800 hover:border-zinc-700'
      }`}
    >
      {/* Header */}
      <div
        className="flex items-center justify-between p-3 cursor-pointer"
        onClick={onToggle}
      >
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 rounded-lg bg-zinc-800 flex items-center justify-center">
            <Users className="w-4 h-4 text-zinc-400" />
          </div>
          <div>
            <div className="font-medium text-sm text-zinc-200">{displayName}</div>
            <div className="flex items-center gap-2 mt-0.5">
              <span className={`text-xs font-mono ${errorColor}`}>
                {metrics.error_magnitude.toFixed(3)}m
              </span>
              <span className="text-zinc-600">|</span>
              <span className="text-xs text-zinc-500 font-mono">
                RMSE: {metrics.rmse_total.toFixed(3)}
              </span>
            </div>
          </div>
        </div>

        <div className="flex items-center gap-2">
          {controllerType && <ControllerBadge type={controllerType} size="xs" />}
          <Badge
            color={metrics.is_settled ? 'green' : 'yellow'}
            size="xs"
            pulse={!metrics.is_settled}
          >
            {metrics.is_settled ? (
              <><CheckCircle className="w-3 h-3" /> SETTLED</>
            ) : (
              <><Clock className="w-3 h-3" /> MOVING</>
            )}
          </Badge>
        </div>
      </div>

      {/* Expanded Details */}
      {isExpanded && (
        <div className="border-t border-zinc-800 p-4 animate-fade-in">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            {/* Position */}
            <div>
              <div className="text-xs text-zinc-500 mb-2 flex items-center gap-1">
                <Crosshair className="w-3 h-3" /> Position
              </div>
              <div className="space-y-1">
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">X:</span>
                  <span className="font-mono text-zinc-300">{metrics.current_x.toFixed(3)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Y:</span>
                  <span className="font-mono text-zinc-300">{metrics.current_y.toFixed(3)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Z:</span>
                  <span className="font-mono text-zinc-300">{metrics.current_z.toFixed(3)}</span>
                </div>
              </div>
            </div>

            {/* RMSE */}
            <div>
              <div className="text-xs text-zinc-500 mb-2">RMSE</div>
              <div className="space-y-1">
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">X:</span>
                  <span className="font-mono text-zinc-300">{metrics.rmse_x.toFixed(4)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Y:</span>
                  <span className="font-mono text-zinc-300">{metrics.rmse_y.toFixed(4)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Total:</span>
                  <span className="font-mono text-blue-400">{metrics.rmse_total.toFixed(4)}</span>
                </div>
              </div>
            </div>

            {/* IAE/ITAE */}
            <div>
              <div className="text-xs text-zinc-500 mb-2">Integral Metrics</div>
              <div className="space-y-1">
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">IAE X:</span>
                  <span className="font-mono text-zinc-300">{metrics.iae_x.toFixed(2)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">IAE Y:</span>
                  <span className="font-mono text-zinc-300">{metrics.iae_y.toFixed(2)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">ITAE X:</span>
                  <span className="font-mono text-zinc-300">{metrics.itae_x.toFixed(2)}</span>
                </div>
              </div>
            </div>

            {/* Performance */}
            <div>
              <div className="text-xs text-zinc-500 mb-2">Performance</div>
              <div className="space-y-1">
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Settling:</span>
                  <span className="font-mono text-zinc-300">{metrics.settling_time.toFixed(2)}s</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Overshoot X:</span>
                  <span className="font-mono text-zinc-300">{(metrics.max_overshoot_x * 100).toFixed(1)}%</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-zinc-500">Overshoot Y:</span>
                  <span className="font-mono text-zinc-300">{(metrics.max_overshoot_y * 100).toFixed(1)}%</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default AgentsView;
