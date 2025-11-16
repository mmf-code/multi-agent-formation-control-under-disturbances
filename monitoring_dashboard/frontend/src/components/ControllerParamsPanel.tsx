/**
 * Controller Parameters Panel
 * Displays real-time controller configuration for each agent
 */
import { ControllerParams } from '../api/ws';
import { Settings } from 'lucide-react';

interface ControllerParamsPanelProps {
  controllerParams: Record<string, ControllerParams>;
  selectedAgent?: string | null;
}

export function ControllerParamsPanel({ controllerParams, selectedAgent }: ControllerParamsPanelProps) {
  const agents = Object.keys(controllerParams).sort();
  const displayAgents = selectedAgent && controllerParams[selectedAgent]
    ? [selectedAgent]
    : agents;

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
    <div className="bg-gray-800 rounded-lg p-6 border border-gray-700">
      <div className="flex items-center space-x-2 mb-4">
        <Settings className="w-5 h-5 text-blue-400" />
        <h3 className="text-lg font-semibold">Controller Parameters</h3>
      </div>

      <div className="space-y-6">
        {displayAgents.map((agentId) => {
          const params = controllerParams[agentId];
          if (!params) return null;

          return (
            <div key={agentId} className="border border-gray-700 rounded-lg p-4">
              {/* Agent Header */}
              <div className="flex items-center justify-between mb-3 pb-2 border-b border-gray-700">
                <h4 className="font-semibold text-blue-400">{agentId}</h4>
                <span className="text-xs bg-blue-900 text-blue-200 px-2 py-1 rounded">
                  {params.controller_type.toUpperCase()}
                </span>
              </div>

              {/* PID Parameters */}
              <div className="mb-3">
                <h5 className="text-sm font-semibold text-gray-300 mb-2">PID Gains</h5>
                <div className="grid grid-cols-3 gap-4">
                  <ParamDisplay label="Kp" value={params.pid_kp} color="text-green-400" />
                  <ParamDisplay label="Ki" value={params.pid_ki} color="text-yellow-400" />
                  <ParamDisplay label="Kd" value={params.pid_kd} color="text-purple-400" />
                </div>
              </div>

              {/* Fuzzy Parameters */}
              {params.fuzzy_enable && (
                <div className="mb-3">
                  <h5 className="text-sm font-semibold text-gray-300 mb-2">
                    Fuzzy Logic
                    <span className="ml-2 text-xs text-green-400">● ENABLED</span>
                  </h5>
                  <div className="grid grid-cols-2 gap-4">
                    <ParamDisplay
                      label="Wind Scalar"
                      value={params.fuzzy_wind_scalar}
                      color="text-cyan-400"
                    />
                  </div>
                </div>
              )}

              {/* Hybrid Mixing */}
              {params.controller_type === 'hybrid' && (
                <div className="mb-3">
                  <h5 className="text-sm font-semibold text-gray-300 mb-2">Hybrid Mixing</h5>
                  <div className="grid grid-cols-2 gap-4">
                    <ParamDisplay
                      label="PID Weight"
                      value={params.mix_k_pid}
                      color="text-blue-400"
                    />
                    <ParamDisplay
                      label="Fuzzy Weight"
                      value={params.mix_k_fuzzy}
                      color="text-orange-400"
                    />
                  </div>
                </div>
              )}

              {/* Feed-forward */}
              {(params.feedforward_enable_drag || params.feedforward_enable_wind) && (
                <div className="mb-3">
                  <h5 className="text-sm font-semibold text-gray-300 mb-2">Feed-Forward</h5>
                  <div className="grid grid-cols-2 gap-4">
                    {params.feedforward_enable_drag && (
                      <ParamDisplay
                        label="Drag Comp."
                        value={params.feedforward_k_drag}
                        color="text-pink-400"
                      />
                    )}
                    {params.feedforward_enable_wind && (
                      <ParamDisplay
                        label="Wind Comp."
                        value={params.feedforward_k_wind}
                        color="text-teal-400"
                      />
                    )}
                  </div>
                </div>
              )}

              {/* Timing & Limits */}
              <div className="grid grid-cols-2 gap-4 text-xs text-gray-400">
                <div>
                  <span className="font-semibold">Control Freq:</span> {params.control_frequency_hz} Hz
                </div>
                <div>
                  <span className="font-semibold">dt:</span> {params.dt.toFixed(4)} s
                </div>
              </div>
            </div>
          );
        })}
      </div>
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
