/**
 * Controller Parameters Panel
 * Displays real-time controller configuration organized by categories
 */
import { useState, useMemo } from 'react';
import { ControllerParams } from '../api/ws';
import { Settings, Sliders, Wind, Gauge, ChevronDown, ChevronRight } from 'lucide-react';

interface ControllerParamsPanelProps {
  controllerParams: Record<string, ControllerParams>;
  selectedAgent?: string | null;
}

export function ControllerParamsPanel({ controllerParams, selectedAgent }: ControllerParamsPanelProps) {
  const [expandedSections, setExpandedSections] = useState<Set<string>>(new Set(['summary', 'pid']));
  const [viewMode, setViewMode] = useState<'summary' | 'detailed'>('summary');

  const agents = Object.keys(controllerParams).sort();
  const displayAgents = selectedAgent && controllerParams[selectedAgent]
    ? [selectedAgent]
    : agents;

  // Group agents by controller type
  const groupedAgents = useMemo(() => {
    const groups: Record<string, string[]> = {};
    displayAgents.forEach(agentId => {
      const type = controllerParams[agentId]?.controller_type || 'unknown';
      if (!groups[type]) groups[type] = [];
      groups[type].push(agentId);
    });
    return groups;
  }, [displayAgents, controllerParams]);

  // Summary statistics
  const summary = useMemo(() => {
    const types: Record<string, number> = {};
    let avgKp = 0, avgKi = 0, avgKd = 0;
    let fuzzyEnabled = 0;

    displayAgents.forEach(agentId => {
      const params = controllerParams[agentId];
      if (!params) return;

      types[params.controller_type] = (types[params.controller_type] || 0) + 1;
      avgKp += params.pid_kp;
      avgKi += params.pid_ki;
      avgKd += params.pid_kd;
      if (params.fuzzy_enable) fuzzyEnabled++;
    });

    const count = displayAgents.length || 1;
    return {
      types,
      avgKp: avgKp / count,
      avgKi: avgKi / count,
      avgKd: avgKd / count,
      fuzzyEnabled,
      total: displayAgents.length
    };
  }, [displayAgents, controllerParams]);

  const toggleSection = (section: string) => {
    setExpandedSections(prev => {
      const next = new Set(prev);
      if (next.has(section)) next.delete(section);
      else next.add(section);
      return next;
    });
  };

  if (displayAgents.length === 0) {
    return (
      <div className="bg-gray-800 rounded-lg p-6 border border-gray-700">
        <div className="flex items-center space-x-2 mb-4">
          <Settings className="w-5 h-5 text-blue-400" />
          <h3 className="text-lg font-semibold">Controller Parameters</h3>
        </div>
        <div className="text-gray-400 text-sm">
          No controller parameters available
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {/* Header with view toggle */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Settings className="w-5 h-5 text-blue-400" />
          <h3 className="text-lg font-semibold">Controller Parameters</h3>
        </div>
        <div className="flex space-x-1 bg-gray-800 rounded p-1">
          <button
            onClick={() => setViewMode('summary')}
            className={`px-3 py-1 rounded text-xs font-medium ${
              viewMode === 'summary' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'
            }`}
          >
            Summary
          </button>
          <button
            onClick={() => setViewMode('detailed')}
            className={`px-3 py-1 rounded text-xs font-medium ${
              viewMode === 'detailed' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'
            }`}
          >
            Detailed
          </button>
        </div>
      </div>

      {viewMode === 'summary' ? (
        <SummaryView
          summary={summary}
          groupedAgents={groupedAgents}
          controllerParams={controllerParams}
        />
      ) : (
        <DetailedView
          displayAgents={displayAgents}
          controllerParams={controllerParams}
          expandedSections={expandedSections}
          toggleSection={toggleSection}
        />
      )}
    </div>
  );
}

// Summary View Component
function SummaryView({
  summary,
  groupedAgents,
  controllerParams
}: {
  summary: any;
  groupedAgents: Record<string, string[]>;
  controllerParams: Record<string, ControllerParams>;
}) {
  return (
    <div className="space-y-4">
      {/* Controller Types Overview */}
      <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
        <h4 className="text-sm font-semibold text-gray-300 mb-3 flex items-center">
          <Gauge className="w-4 h-4 mr-2 text-purple-400" />
          Controller Distribution
        </h4>
        <div className="grid grid-cols-2 gap-3">
          {Object.entries(summary.types).map(([type, count]) => (
            <div key={type} className={`rounded-lg p-3 ${
              type === 'hybrid' ? 'bg-emerald-900/30 border border-emerald-800' :
              type === 'pid' ? 'bg-blue-900/30 border border-blue-800' :
              'bg-gray-700'
            }`}>
              <div className="text-xs text-gray-400 uppercase">{type === 'hybrid' ? 'PID + Fuzzy' : type}</div>
              <div className="text-2xl font-bold">{count as number}</div>
              <div className="text-xs text-gray-500">agents</div>
            </div>
          ))}
        </div>
      </div>

      {/* Average PID Gains */}
      <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
        <h4 className="text-sm font-semibold text-gray-300 mb-3 flex items-center">
          <Sliders className="w-4 h-4 mr-2 text-blue-400" />
          Average PID Gains
        </h4>
        <div className="grid grid-cols-3 gap-4">
          <div className="bg-gray-900 rounded-lg p-3 text-center">
            <div className="text-xs text-gray-400 mb-1">Kp (Proportional)</div>
            <div className="text-xl font-bold text-green-400">{summary.avgKp.toFixed(2)}</div>
          </div>
          <div className="bg-gray-900 rounded-lg p-3 text-center">
            <div className="text-xs text-gray-400 mb-1">Ki (Integral)</div>
            <div className="text-xl font-bold text-yellow-400">{summary.avgKi.toFixed(2)}</div>
          </div>
          <div className="bg-gray-900 rounded-lg p-3 text-center">
            <div className="text-xs text-gray-400 mb-1">Kd (Derivative)</div>
            <div className="text-xl font-bold text-purple-400">{summary.avgKd.toFixed(2)}</div>
          </div>
        </div>
      </div>

      {/* Fuzzy Logic Status */}
      <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
        <h4 className="text-sm font-semibold text-gray-300 mb-3 flex items-center">
          <Wind className="w-4 h-4 mr-2 text-cyan-400" />
          Fuzzy Logic & Wind Compensation
        </h4>
        <div className="grid grid-cols-2 gap-4">
          <div className="bg-gray-900 rounded-lg p-3">
            <div className="text-xs text-gray-400">Fuzzy Enabled</div>
            <div className="text-xl font-bold text-cyan-400">
              {summary.fuzzyEnabled} <span className="text-sm text-gray-500">/ {summary.total}</span>
            </div>
          </div>
          <div className="bg-gray-900 rounded-lg p-3">
            <div className="text-xs text-gray-400">Wind Compensation</div>
            <div className="text-xl font-bold text-orange-400">
              {Object.values(controllerParams).filter((p: ControllerParams) => p.feedforward_enable_wind).length}
              <span className="text-sm text-gray-500"> / {summary.total}</span>
            </div>
          </div>
        </div>
      </div>

      {/* Group Details */}
      {Object.entries(groupedAgents).map(([type, agents]) => (
        <div key={type} className="bg-gray-800 rounded-lg border border-gray-700 p-4">
          <h4 className={`text-sm font-semibold mb-3 ${
            type === 'hybrid' ? 'text-emerald-400' : 'text-blue-400'
          }`}>
            {type === 'hybrid' ? 'PID + Fuzzy Group' : `${type.toUpperCase()} Group`}
          </h4>
          <div className="space-y-2">
            {agents.map(agentId => {
              const params = controllerParams[agentId];
              return (
                <div key={agentId} className="flex items-center justify-between bg-gray-900 rounded px-3 py-2">
                  <span className="text-sm font-medium text-gray-300">{agentId}</span>
                  <div className="flex items-center space-x-3 text-xs text-gray-400">
                    <span>Kp: <span className="text-green-400">{params.pid_kp.toFixed(1)}</span></span>
                    <span>Ki: <span className="text-yellow-400">{params.pid_ki.toFixed(1)}</span></span>
                    <span>Kd: <span className="text-purple-400">{params.pid_kd.toFixed(1)}</span></span>
                    {params.fuzzy_enable && (
                      <span className="text-cyan-400">Fuzzy</span>
                    )}
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      ))}
    </div>
  );
}

// Detailed View Component
function DetailedView({
  displayAgents,
  controllerParams,
  expandedSections,
  toggleSection
}: {
  displayAgents: string[];
  controllerParams: Record<string, ControllerParams>;
  expandedSections: Set<string>;
  toggleSection: (section: string) => void;
}) {
  return (
    <div className="space-y-3">
      {displayAgents.map((agentId) => {
        const params = controllerParams[agentId];
        if (!params) return null;
        const isExpanded = expandedSections.has(agentId);

        return (
          <div key={agentId} className="bg-gray-800 rounded-lg border border-gray-700">
            {/* Agent Header */}
            <div
              className="flex items-center justify-between p-3 cursor-pointer hover:bg-gray-700/50"
              onClick={() => toggleSection(agentId)}
            >
              <div className="flex items-center space-x-3">
                {isExpanded ? <ChevronDown className="w-4 h-4 text-gray-400" /> : <ChevronRight className="w-4 h-4 text-gray-400" />}
                <span className="font-semibold text-blue-400">{agentId}</span>
                <span className={`text-xs px-2 py-0.5 rounded ${
                  params.controller_type === 'hybrid'
                    ? 'bg-emerald-900 text-emerald-300'
                    : 'bg-blue-900 text-blue-300'
                }`}>
                  {params.controller_type.toUpperCase()}
                </span>
              </div>
              <div className="flex items-center space-x-2 text-xs text-gray-400">
                <span>{params.control_frequency_hz} Hz</span>
              </div>
            </div>

            {/* Expanded Content */}
            {isExpanded && (
              <div className="border-t border-gray-700 p-4 space-y-4">
                {/* PID Parameters */}
                <div>
                  <h5 className="text-sm font-semibold text-gray-300 mb-2 flex items-center">
                    <Sliders className="w-4 h-4 mr-2 text-blue-400" />
                    PID Gains
                  </h5>
                  <div className="grid grid-cols-3 gap-3">
                    <ParamDisplay label="Kp" value={params.pid_kp} color="text-green-400" />
                    <ParamDisplay label="Ki" value={params.pid_ki} color="text-yellow-400" />
                    <ParamDisplay label="Kd" value={params.pid_kd} color="text-purple-400" />
                  </div>
                </div>

                {/* Fuzzy Parameters */}
                {params.fuzzy_enable && (
                  <div>
                    <h5 className="text-sm font-semibold text-gray-300 mb-2 flex items-center">
                      <Wind className="w-4 h-4 mr-2 text-cyan-400" />
                      Fuzzy Logic
                      <span className="ml-2 text-xs text-green-400">ENABLED</span>
                    </h5>
                    <div className="grid grid-cols-2 gap-3">
                      <ParamDisplay label="Wind Scalar" value={params.fuzzy_wind_scalar} color="text-cyan-400" />
                    </div>
                  </div>
                )}

                {/* Hybrid Mixing */}
                {params.controller_type === 'hybrid' && (
                  <div>
                    <h5 className="text-sm font-semibold text-gray-300 mb-2">Hybrid Mixing</h5>
                    <div className="grid grid-cols-2 gap-3">
                      <ParamDisplay label="PID Weight" value={params.mix_k_pid} color="text-blue-400" />
                      <ParamDisplay label="Fuzzy Weight" value={params.mix_k_fuzzy} color="text-orange-400" />
                    </div>
                  </div>
                )}

                {/* Feed-forward */}
                {(params.feedforward_enable_drag || params.feedforward_enable_wind) && (
                  <div>
                    <h5 className="text-sm font-semibold text-gray-300 mb-2">Feed-Forward Compensation</h5>
                    <div className="grid grid-cols-2 gap-3">
                      {params.feedforward_enable_drag && (
                        <ParamDisplay label="Drag Comp." value={params.feedforward_k_drag} color="text-pink-400" />
                      )}
                      {params.feedforward_enable_wind && (
                        <ParamDisplay label="Wind Comp." value={params.feedforward_k_wind} color="text-teal-400" />
                      )}
                    </div>
                  </div>
                )}

                {/* Output Limits */}
                <div>
                  <h5 className="text-sm font-semibold text-gray-300 mb-2">Output Limits</h5>
                  <div className="grid grid-cols-2 gap-3 text-xs">
                    <div className="bg-gray-900 rounded p-2">
                      <span className="text-gray-400">X: </span>
                      <span className="text-gray-200">[{params.output_limit_x_min.toFixed(1)}, {params.output_limit_x_max.toFixed(1)}]</span>
                    </div>
                    <div className="bg-gray-900 rounded p-2">
                      <span className="text-gray-400">Y: </span>
                      <span className="text-gray-200">[{params.output_limit_y_min.toFixed(1)}, {params.output_limit_y_max.toFixed(1)}]</span>
                    </div>
                  </div>
                </div>

                {/* Timing */}
                <div className="flex items-center space-x-4 text-xs text-gray-400 pt-2 border-t border-gray-700">
                  <span><strong>Control Freq:</strong> {params.control_frequency_hz} Hz</span>
                  <span><strong>dt:</strong> {params.dt.toFixed(4)} s</span>
                </div>
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}

interface ParamDisplayProps {
  label: string;
  value: number;
  color?: string;
}

function ParamDisplay({ label, value, color = 'text-white' }: ParamDisplayProps) {
  return (
    <div className="bg-gray-900 rounded px-3 py-2">
      <div className="text-xs text-gray-400 mb-1">{label}</div>
      <div className={`text-lg font-bold ${color}`}>
        {value.toFixed(2)}
      </div>
    </div>
  );
}
