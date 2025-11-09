/**
 * KPI Cards Component
 * Displays key performance indicators for each agent
 */
import React from 'react';
import { MetricsData } from '../api/ws';

interface KpiCardsProps {
  metrics: Record<string, MetricsData>;
  selectedAgent: string | null;
}

export const KpiCards: React.FC<KpiCardsProps> = ({ metrics, selectedAgent }) => {
  const agentMetrics = selectedAgent && metrics[selectedAgent]
    ? [metrics[selectedAgent]]
    : Object.values(metrics);

  if (agentMetrics.length === 0) {
    return (
      <div className="p-4 bg-gray-800 rounded-lg text-gray-400">
        No metrics data available
      </div>
    );
  }

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
      {agentMetrics.map((m) => (
        <div key={m.agent_id} className="bg-gray-800 rounded-lg p-4 border border-gray-700">
          <h3 className="text-sm font-semibold text-gray-400 mb-3 uppercase tracking-wide">
            {m.agent_id}
          </h3>

          <div className="space-y-3">
            {/* Position Error */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">Error (m)</span>
              <span className={`text-lg font-bold ${
                m.error_magnitude < 0.1 ? 'text-green-400' :
                m.error_magnitude < 0.5 ? 'text-yellow-400' : 'text-red-400'
              }`}>
                {m.error_magnitude.toFixed(3)}
              </span>
            </div>

            {/* RMSE */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">RMSE Total</span>
              <span className="text-sm font-medium text-blue-400">
                {m.rmse_total.toFixed(3)}
              </span>
            </div>

            {/* IAE */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">IAE (X, Y)</span>
              <span className="text-sm font-medium text-purple-400">
                {m.iae_x.toFixed(2)}, {m.iae_y.toFixed(2)}
              </span>
            </div>

            {/* ITAE */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">ITAE (X, Y)</span>
              <span className="text-sm font-medium text-pink-400">
                {m.itae_x.toFixed(2)}, {m.itae_y.toFixed(2)}
              </span>
            </div>

            {/* Settling Time */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">Settling Time</span>
              <span className="text-sm font-medium text-cyan-400">
                {m.settling_time >= 0 ? `${m.settling_time.toFixed(2)}s` : 'N/A'}
              </span>
            </div>

            {/* Overshoot */}
            <div className="flex justify-between items-center">
              <span className="text-xs text-gray-500">Max Overshoot</span>
              <span className="text-sm font-medium text-orange-400">
                X: {m.max_overshoot_x.toFixed(3)}m
              </span>
            </div>

            {/* Settled Status */}
            <div className="flex justify-between items-center pt-2 border-t border-gray-700">
              <span className="text-xs text-gray-500">Status</span>
              <span className={`text-xs font-semibold px-2 py-1 rounded ${
                m.is_settled
                  ? 'bg-green-900 text-green-300'
                  : 'bg-yellow-900 text-yellow-300'
              }`}>
                {m.is_settled ? 'SETTLED' : 'TRACKING'}
              </span>
            </div>
          </div>

          {/* Position Info */}
          <div className="mt-4 pt-3 border-t border-gray-700 text-xs text-gray-500">
            <div className="flex justify-between">
              <span>Current:</span>
              <span>({m.current_x.toFixed(2)}, {m.current_y.toFixed(2)}, {m.current_z.toFixed(2)})</span>
            </div>
            <div className="flex justify-between mt-1">
              <span>Target:</span>
              <span>({m.target_x.toFixed(2)}, {m.target_y.toFixed(2)}, {m.target_z.toFixed(2)})</span>
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};
