import React from 'react';
import { MetricsData, WindData, SystemStatus } from '../api/ws';

interface SystemStatePanelProps {
    metrics: Record<string, MetricsData>;
    wind: WindData | null;
    status: SystemStatus | null;
}

export const SystemStatePanel: React.FC<SystemStatePanelProps> = ({ metrics, wind, status }) => {
    // Group agents by formation group (mock logic for now as it's not in metrics yet)
    // In a real scenario, we'd get this from a config endpoint or infer it
    const agents = Object.keys(metrics).sort();

    return (
        <div className="space-y-6">
            {/* System Status */}
            <div className="bg-gray-800 rounded-lg border border-gray-700 overflow-hidden">
                <div className="px-4 py-3 bg-gray-800/50 border-b border-gray-700">
                    <h3 className="text-sm font-semibold text-gray-200">System Status</h3>
                </div>
                <div className="p-4 grid grid-cols-2 gap-4">
                    <div>
                        <div className="text-xs text-gray-500 uppercase">Formation Active</div>
                        <div className={`font-mono text-sm ${status?.formation_active ? 'text-green-400' : 'text-gray-400'}`}>
                            {status?.formation_active ? 'YES' : 'NO'}
                        </div>
                    </div>
                    <div>
                        <div className="text-xs text-gray-500 uppercase">Timestamp</div>
                        <div className="font-mono text-sm text-gray-300">
                            {status?.timestamp ? new Date(status.timestamp * 1000).toLocaleTimeString() : '-'}
                        </div>
                    </div>
                </div>
            </div>

            {/* Wind Configuration */}
            <div className="bg-gray-800 rounded-lg border border-gray-700 overflow-hidden">
                <div className="px-4 py-3 bg-gray-800/50 border-b border-gray-700">
                    <h3 className="text-sm font-semibold text-gray-200">Wind Configuration</h3>
                </div>
                <div className="p-4">
                    {wind ? (
                        <div className="grid grid-cols-2 gap-4">
                            <div>
                                <div className="text-xs text-gray-500 uppercase">Velocity Vector</div>
                                <div className="font-mono text-sm text-blue-300">
                                    [{wind.velocity.x.toFixed(2)}, {wind.velocity.y.toFixed(2)}] m/s
                                </div>
                            </div>
                            <div>
                                <div className="text-xs text-gray-500 uppercase">Force Vector</div>
                                <div className="font-mono text-sm text-orange-300">
                                    [{wind.force?.x.toFixed(2) || '0.00'}, {wind.force?.y.toFixed(2) || '0.00'}] N
                                </div>
                            </div>
                        </div>
                    ) : (
                        <div className="text-sm text-gray-500 italic">No wind data available</div>
                    )}
                </div>
            </div>

            {/* Agent Configuration Table */}
            <div className="bg-gray-800 rounded-lg border border-gray-700 overflow-hidden">
                <div className="px-4 py-3 bg-gray-800/50 border-b border-gray-700">
                    <h3 className="text-sm font-semibold text-gray-200">Agent Configuration</h3>
                </div>
                <div className="overflow-x-auto">
                    <table className="w-full text-sm text-left">
                        <thead className="text-xs text-gray-400 uppercase bg-gray-900/50">
                            <tr>
                                <th className="px-4 py-2">Agent ID</th>
                                <th className="px-4 py-2">Group</th>
                                <th className="px-4 py-2">Controller</th>
                                <th className="px-4 py-2">Status</th>
                            </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-700">
                            {agents.map(agentId => {
                                // Mock group assignment based on ID
                                const idNum = parseInt(agentId.split('_')[1] || '0');
                                const group = Math.floor(idNum / 3);
                                const controllerType = group === 0 ? 'PID+Fuzzy' : group === 1 ? 'PD' : 'PID';

                                return (
                                    <tr key={agentId} className="hover:bg-gray-700/30 transition-colors">
                                        <td className="px-4 py-2 font-medium text-gray-200">{agentId}</td>
                                        <td className="px-4 py-2 text-gray-400">Group {group}</td>
                                        <td className="px-4 py-2">
                                            <span className={`px-2 py-0.5 rounded text-xs border ${controllerType === 'PID+Fuzzy' ? 'border-purple-500/30 text-purple-400 bg-purple-500/10' :
                                                controllerType === 'PD' ? 'border-blue-500/30 text-blue-400 bg-blue-500/10' :
                                                    'border-green-500/30 text-green-400 bg-green-500/10'
                                                }`}>
                                                {controllerType}
                                            </span>
                                        </td>
                                        <td className="px-4 py-2">
                                            <span className={`text-xs ${metrics[agentId].is_settled ? 'text-green-400' : 'text-yellow-400'}`}>
                                                {metrics[agentId].is_settled ? 'Settled' : 'Moving'}
                                            </span>
                                        </td>
                                    </tr>
                                );
                            })}
                            {agents.length === 0 && (
                                <tr>
                                    <td colSpan={4} className="px-4 py-4 text-center text-gray-500 italic">
                                        No agents connected
                                    </td>
                                </tr>
                            )}
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    );
};
