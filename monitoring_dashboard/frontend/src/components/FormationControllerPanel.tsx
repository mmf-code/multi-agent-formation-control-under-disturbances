import React from 'react';
import { ControllerParams } from '../api/ws';

interface FormationControllerPanelProps {
  controllerParams: Record<string, ControllerParams>;
}

interface FormationGroup {
  id: number;
  name: string;
  agents: number[];
  expectedController: string;
  color: string;
  targetPosition: string;
}

const FORMATION_GROUPS: FormationGroup[] = [
  {
    id: 0,
    name: 'Formation 0',
    agents: [0, 1, 2],
    expectedController: 'PID+Fuzzy',
    color: '#ec4899', // Pink
    targetPosition: '(5, -4, 1)',
  },
  {
    id: 1,
    name: 'Formation 1',
    agents: [3, 4, 5],
    expectedController: 'PD',
    color: '#06b6d4', // Cyan
    targetPosition: '(5, 0, 1)',
  },
  {
    id: 2,
    name: 'Formation 2',
    agents: [6, 7, 8],
    expectedController: 'PID',
    color: '#eab308', // Yellow
    targetPosition: '(5, 4, 1)',
  },
];

const FormationControllerPanel: React.FC<FormationControllerPanelProps> = ({ controllerParams }) => {
  const getControllerTypeLabel = (type: string): string => {
    const typeMap: Record<string, string> = {
      'pid': 'PID',
      'pd': 'PD',
      'p': 'P',
      'pi': 'PI',
      'fuzzy': 'Fuzzy',
      'hybrid': 'Hybrid',
    };
    return typeMap[type.toLowerCase()] || type;
  };

  const getControllerTypeBadgeColor = (type: string): string => {
    const colorMap: Record<string, string> = {
      'pid': 'bg-blue-600',
      'pd': 'bg-green-600',
      'p': 'bg-yellow-600',
      'pi': 'bg-purple-600',
      'fuzzy': 'bg-orange-600',
      'hybrid': 'bg-pink-600',
    };
    return colorMap[type.toLowerCase()] || 'bg-gray-600';
  };

  const renderParameterRow = (label: string, value: number | boolean | undefined, unit: string = '') => {
    if (value === undefined) return null;

    const displayValue = typeof value === 'boolean'
      ? (value ? '✓' : '✗')
      : typeof value === 'number'
        ? value.toFixed(4)
        : value;

    return (
      <div className="flex justify-between py-1 text-sm">
        <span className="text-gray-400">{label}:</span>
        <span className="text-white font-mono">{displayValue}{unit}</span>
      </div>
    );
  };

  const renderAgentParameters = (agentId: number) => {
    const params = controllerParams[`agent_${agentId}`];

    if (!params) {
      return (
        <div className="bg-gray-800 rounded p-3 border border-gray-700">
          <div className="text-gray-500 text-sm">No data for Agent {agentId}</div>
        </div>
      );
    }

    return (
      <div className="bg-gray-800 rounded p-3 border border-gray-700 hover:border-gray-600 transition-colors">
        <div className="flex items-center justify-between mb-2">
          <h4 className="font-semibold text-gray-200">Agent {agentId}</h4>
          <span className={`px-2 py-1 rounded text-xs font-semibold ${getControllerTypeBadgeColor(params.controller_type)}`}>
            {getControllerTypeLabel(params.controller_type)}
          </span>
        </div>

        {/* PID Gains */}
        <div className="mb-2">
          <div className="text-xs font-semibold text-gray-400 mb-1">PID Gains</div>
          <div className="grid grid-cols-3 gap-2 text-xs">
            <div className="bg-red-900/20 border border-red-700/30 rounded p-1.5">
              <div className="text-red-400 font-semibold">Kp</div>
              <div className="text-white font-mono">{params.pid_kp.toFixed(3)}</div>
            </div>
            <div className="bg-green-900/20 border border-green-700/30 rounded p-1.5">
              <div className="text-green-400 font-semibold">Ki</div>
              <div className="text-white font-mono">{params.pid_ki.toFixed(3)}</div>
            </div>
            <div className="bg-blue-900/20 border border-blue-700/30 rounded p-1.5">
              <div className="text-blue-400 font-semibold">Kd</div>
              <div className="text-white font-mono">{params.pid_kd.toFixed(3)}</div>
            </div>
          </div>
        </div>

        {/* Fuzzy Logic (if enabled) */}
        {params.fuzzy_enable && (
          <div className="mb-2 bg-orange-900/10 border border-orange-700/30 rounded p-2">
            <div className="text-xs font-semibold text-orange-400 mb-1">Fuzzy Logic</div>
            {renderParameterRow('Wind Scalar', params.fuzzy_wind_scalar)}
          </div>
        )}

        {/* Hybrid Mixing (if applicable) */}
        {params.controller_type.toLowerCase() === 'hybrid' && (
          <div className="mb-2 bg-pink-900/10 border border-pink-700/30 rounded p-2">
            <div className="text-xs font-semibold text-pink-400 mb-1">Hybrid Mixing</div>
            {renderParameterRow('PID Weight', params.mix_k_pid)}
            {renderParameterRow('Fuzzy Weight', params.mix_k_fuzzy)}
          </div>
        )}

        {/* Feed-forward Compensation */}
        {(params.feedforward_enable_drag || params.feedforward_enable_wind) && (
          <div className="mb-2 bg-purple-900/10 border border-purple-700/30 rounded p-2">
            <div className="text-xs font-semibold text-purple-400 mb-1">Feed-forward</div>
            {params.feedforward_enable_drag && renderParameterRow('Drag K', params.feedforward_k_drag)}
            {params.feedforward_enable_wind && renderParameterRow('Wind K', params.feedforward_k_wind)}
          </div>
        )}

        {/* Control Frequency */}
        <div className="text-xs text-gray-500 mt-2 pt-2 border-t border-gray-700">
          {params.control_frequency_hz.toFixed(1)} Hz | dt: {params.dt.toFixed(4)}s
        </div>
      </div>
    );
  };

  const renderFormationGroup = (group: FormationGroup) => {
    const hasData = group.agents.some(agentId => controllerParams[`agent_${agentId}`]);

    return (
      <div key={group.id} className="bg-gray-900 rounded-lg p-4 border-2 transition-all"
           style={{ borderColor: hasData ? group.color : '#374151' }}>
        {/* Formation Header */}
        <div className="mb-4 pb-3 border-b border-gray-700">
          <div className="flex items-center justify-between">
            <div>
              <h3 className="text-lg font-bold" style={{ color: group.color }}>
                {group.name}
              </h3>
              <div className="text-sm text-gray-400 mt-1">
                Agents: {group.agents.join(', ')} | Controller: {group.expectedController}
              </div>
              <div className="text-xs text-gray-500 mt-0.5">
                Target: {group.targetPosition}
              </div>
            </div>
            {hasData && (
              <div className="text-green-400 text-sm">
                ● Active
              </div>
            )}
            {!hasData && (
              <div className="text-gray-500 text-sm">
                ○ No Data
              </div>
            )}
          </div>
        </div>

        {/* Agent Parameters Grid */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
          {group.agents.map(agentId => renderAgentParameters(agentId))}
        </div>

        {/* Formation Summary */}
        {hasData && (
          <div className="mt-3 pt-3 border-t border-gray-700">
            <div className="grid grid-cols-3 gap-3 text-xs">
              {group.agents.map(agentId => {
                const params = controllerParams[`agent_${agentId}`];
                if (!params) return null;
                return (
                  <div key={agentId} className="bg-gray-800 rounded p-2">
                    <div className="text-gray-400 mb-1">Agent {agentId} Avg</div>
                    <div className="font-mono text-white">
                      Kp: {params.pid_kp.toFixed(2)} |
                      Ki: {params.pid_ki.toFixed(2)} |
                      Kd: {params.pid_kd.toFixed(2)}
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between mb-2">
        <h2 className="text-xl font-bold text-white">Formation Controller Parameters</h2>
        <div className="text-sm text-gray-400">
          3 Formations | 9 Agents Total
        </div>
      </div>

      {FORMATION_GROUPS.map(group => renderFormationGroup(group))}
    </div>
  );
};

export default FormationControllerPanel;
