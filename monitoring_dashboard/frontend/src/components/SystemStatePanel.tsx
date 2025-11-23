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
            <div className="glass-panel rounded-lg overflow-hidden animate-fade-in">
                <div className="px-4 py-3 border-b border-gray-700/50 bg-space-900/30">
                    <h3 className="text-sm font-semibold text-neon-blue tracking-wider uppercase">System Status</h3>
                </div>
                <div className="p-4 grid grid-cols-2 gap-4">
                    <div>
                        <div className="text-xs text-gray-400 uppercase tracking-wide mb-1">Formation Active</div>
                        <div className={`font-mono text-sm font-bold ${status?.formation_active ? 'text-neon-green drop-shadow-[0_0_5px_rgba(10,255,104,0.5)]' : 'text-gray-500'}`}>
                            {status?.formation_active ? 'ACTIVE' : 'INACTIVE'}
                        </div>
                    </div>
                    <div>
                        <div className="text-xs text-gray-400 uppercase tracking-wide mb-1">Timestamp</div>
                        <div className="font-mono text-sm text-gray-300">
                            {status?.timestamp ? new Date(status.timestamp * 1000).toLocaleTimeString() : '-'}
                        </div>
                    </div>
                </div>
            </div>

            {/* Wind Configuration */}
            <div className="glass-panel rounded-lg overflow-hidden animate-slide-up" style={{ animationDelay: '0.1s' }}>
                <div className="px-4 py-3 border-b border-gray-700/50 bg-space-900/30">
                    <h3 className="text-sm font-semibold text-neon-blue tracking-wider uppercase">Wind Configuration</h3>
                </div>
                <div className="p-4">
                    {wind ? (
                        <div className="grid grid-cols-2 gap-4">
                            <div>
                                <div className="text-xs text-gray-400 uppercase tracking-wide mb-1">Velocity Vector</div>
                                <div className="font-mono text-sm text-neon-blue">
                                    [{wind.velocity.x.toFixed(2)}, {wind.velocity.y.toFixed(2)}] m/s
                                </div>
                            </div>
                            <div>
                                <div className="text-xs text-gray-400 uppercase tracking-wide mb-1">Force Vector</div>
                                <div className="font-mono text-sm text-neon-purple">
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
            <div className="glass-panel rounded-lg overflow-hidden animate-slide-up" style={{ animationDelay: '0.2s' }}>
                <div className="px-4 py-3 border-b border-gray-700/50 bg-space-900/30">
                    <h3 className="text-sm font-semibold text-neon-blue tracking-wider uppercase">Agent Configuration</h3>
                </div>
                <div className="overflow-x-auto">
                    <table className="w-full text-sm text-left">
                        <thead className="text-xs text-gray-400 uppercase bg-space-950/50">
                            <tr>
                                <th className="px-4 py-2">Agent ID</th>
                                <th className="px-4 py-2">Group</th>
                                <th className="px-4 py-2">Controller</th>
                                <th className="px-4 py-2">Status</th>
                            </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-700/30">
                            {agents.map((agentId) => {
                                // Mock group assignment based on ID
                                const idNum = parseInt(agentId.split('_')[1] || '0');
                                const group = Math.floor(idNum / 3);
                                const controllerType = group === 0 ? 'PID+Fuzzy' : group === 1 ? 'PD' : 'PID';

                                return (
                                    <tr key={agentId} className="hover:bg-neon-blue/5 transition-colors">
                                        <td className="px-4 py-2 font-medium text-gray-200">{agentId}</td>
                                        <td className="px-4 py-2 text-gray-400">Group {group}</td>
                                        <td className="px-4 py-2">
                                            <span className={`px-2 py-0.5 rounded text-xs border ${controllerType === 'PID+Fuzzy' ? 'border-neon-purple/30 text-neon-purple bg-neon-purple/10' :
                                                controllerType === 'PD' ? 'border-neon-blue/30 text-neon-blue bg-neon-blue/10' :
                                                    'border-neon-green/30 text-neon-green bg-neon-green/10'
                                                }`}>
                                                {controllerType}
                                            </span>
                                        </td>
                                        <td className="px-4 py-2">
                                            <span className={`text-xs font-medium ${metrics[agentId].is_settled ? 'text-neon-green' : 'text-neon-red animate-pulse'}`}>
                                                {metrics[agentId].is_settled ? 'SETTLED' : 'MOVING'}
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
