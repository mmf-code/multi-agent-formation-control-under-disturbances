import React, { useState, useMemo } from 'react';
import {
    Activity,
    Wind,
    Users,
    BarChart2,
    Map as MapIcon,
    Settings,
    LayoutDashboard,
    AlertTriangle,
    CheckCircle,
    XCircle,
    Radio,
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

interface SimulationMetricsPanelProps {
    metrics: Record<string, MetricsData>;
    wind: WindData | null;
    formation: FormationState | null;
    status: SystemStatus | null;
    metricsHistory: Record<string, MetricsData[]>;
    windHistory: WindData[];
    odomData: Record<string, OdometryData>;
    targetData: Record<string, any>;
    controllerParams?: Record<string, ControllerParams>;
    rosGraph?: RosGraphData | null;
}

type TabType = 'dashboard' | 'overview' | 'charts' | 'agents' | 'map' | 'system' | 'params';

export const SimulationMetricsPanel: React.FC<SimulationMetricsPanelProps> = ({
    metrics,
    wind,
    formation: _formation,
    status,
    metricsHistory,
    windHistory,
    odomData,
    targetData,
    controllerParams = {},
    rosGraph,
}) => {
    const [activeTab, setActiveTab] = useState<TabType>('dashboard');

    const tabs: { id: TabType; label: string; icon: React.ReactNode }[] = [
        { id: 'dashboard', label: 'Dashboard', icon: <LayoutDashboard className="w-4 h-4" /> },
        { id: 'overview', label: 'Overview', icon: <Activity className="w-4 h-4" /> },
        { id: 'map', label: 'Map', icon: <MapIcon className="w-4 h-4" /> },
        { id: 'charts', label: 'Charts', icon: <BarChart2 className="w-4 h-4" /> },
        { id: 'agents', label: 'Agents', icon: <Users className="w-4 h-4" /> },
        { id: 'params', label: 'Params', icon: <Settings className="w-4 h-4" /> },
        { id: 'system', label: 'System', icon: <Radio className="w-4 h-4" /> },
    ];

    return (
        <div className="h-full flex flex-col bg-gray-900 text-white">
            {/* Tabs */}
            <div className="flex border-b border-gray-800 overflow-x-auto">
                {tabs.map((tab) => (
                    <button
                        key={tab.id}
                        onClick={() => setActiveTab(tab.id)}
                        className={`flex items-center px-3 py-2.5 text-sm font-medium transition-colors whitespace-nowrap ${
                            activeTab === tab.id
                                ? 'text-blue-400 border-b-2 border-blue-400 bg-gray-800/50'
                                : 'text-gray-400 hover:text-gray-200 hover:bg-gray-800/30'
                        }`}
                    >
                        <span className="mr-1.5">{tab.icon}</span>
                        {tab.label}
                    </button>
                ))}
            </div>

            {/* Content */}
            <div className="flex-1 overflow-y-auto p-4">
                {activeTab === 'dashboard' && (
                    <DashboardSection
                        metrics={metrics}
                        wind={wind}
                        status={status}
                        rosGraph={rosGraph}
                        controllerParams={controllerParams}
                    />
                )}

                {activeTab === 'overview' && (
                    <OverviewSection
                        metrics={metrics}
                        wind={wind}
                        status={status}
                    />
                )}

                {activeTab === 'map' && (
                    <div className="h-full flex flex-col">
                        <FormationMap
                            odomData={odomData}
                            targetData={targetData}
                            formationState={_formation}
                            selectedAgents={new Set()}
                            windData={wind}
                            metricsData={metrics}
                        />
                    </div>
                )}

                {activeTab === 'system' && (
                    <SystemStatePanel
                        metrics={metrics}
                        wind={wind}
                        status={status}
                    />
                )}

                {activeTab === 'charts' && (
                    <ChartsSection
                        metricsHistory={metricsHistory}
                        windHistory={windHistory}
                        controllerParams={controllerParams}
                    />
                )}

                {activeTab === 'agents' && (
                    <AgentsSection metrics={metrics} controllerParams={controllerParams} />
                )}

                {activeTab === 'params' && (
                    <ControllerParamsPanel
                        controllerParams={controllerParams}
                    />
                )}
            </div>
        </div>
    );
};

// Dashboard Section - Summary view with all key info
const DashboardSection: React.FC<{
    metrics: Record<string, MetricsData>;
    wind: WindData | null;
    status: SystemStatus | null;
    rosGraph?: RosGraphData | null;
    controllerParams?: Record<string, ControllerParams>;
}> = ({ metrics, wind, status, rosGraph, controllerParams = {} }) => {
    const agentCount = Object.keys(metrics).length;
    const settledCount = Object.values(metrics).filter(m => m.is_settled).length;
    const avgError = agentCount > 0
        ? Object.values(metrics).reduce((sum, m) => sum + m.error_magnitude, 0) / agentCount
        : 0;

    // Group metrics by controller type
    const groupStats = useMemo(() => {
        const groups: Record<string, { count: number; avgError: number; settled: number }> = {};

        Object.entries(metrics).forEach(([agentId, m]) => {
            const params = controllerParams[agentId];
            const type = params?.controller_type || 'unknown';

            if (!groups[type]) {
                groups[type] = { count: 0, avgError: 0, settled: 0 };
            }
            groups[type].count++;
            groups[type].avgError += m.error_magnitude;
            if (m.is_settled) groups[type].settled++;
        });

        // Calculate averages
        Object.keys(groups).forEach(type => {
            if (groups[type].count > 0) {
                groups[type].avgError /= groups[type].count;
            }
        });

        return groups;
    }, [metrics, controllerParams]);

    // Topic stats
    const topicStats = useMemo(() => {
        if (!rosGraph) return { total: 0, active: 0, nodes: 0 };
        const topics = rosGraph.edges?.length || 0;
        const nodes = rosGraph.nodes?.length || 0;
        return { total: topics, active: topics, nodes };
    }, [rosGraph]);

    // Error stats
    const errorStats = useMemo(() => {
        const errors = Object.values(metrics);
        if (errors.length === 0) return { max: 0, min: 0, maxAgent: '-' };

        let max = 0;
        let maxAgent = '-';
        let min = Infinity;

        Object.entries(metrics).forEach(([id, m]) => {
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

    return (
        <div className="space-y-4">
            {/* System Status Row */}
            <div className="grid grid-cols-4 gap-3">
                <StatusCard
                    label="System"
                    value={status?.ros_ok ? 'Online' : 'Offline'}
                    icon={status?.ros_ok ? <CheckCircle className="w-4 h-4" /> : <XCircle className="w-4 h-4" />}
                    color={status?.ros_ok ? 'green' : 'red'}
                />
                <StatusCard
                    label="Formation"
                    value={status?.formation_active ? 'Active' : 'Idle'}
                    icon={<Activity className="w-4 h-4" />}
                    color={status?.formation_active ? 'blue' : 'gray'}
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
            <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
                <h3 className="text-sm font-semibold text-gray-200 mb-3">Controller Groups Performance</h3>
                <div className="grid grid-cols-2 gap-4">
                    {Object.entries(groupStats).map(([type, stats]) => (
                        <div key={type} className="bg-gray-900 rounded-lg p-3">
                            <div className="flex items-center justify-between mb-2">
                                <span className={`text-xs font-bold uppercase px-2 py-0.5 rounded ${
                                    type === 'hybrid' ? 'bg-emerald-900 text-emerald-300' :
                                    type === 'pid' ? 'bg-blue-900 text-blue-300' :
                                    'bg-gray-700 text-gray-300'
                                }`}>
                                    {type === 'hybrid' ? 'PID + Fuzzy' : type.toUpperCase()}
                                </span>
                                <span className="text-xs text-gray-500">{stats.count} agents</span>
                            </div>
                            <div className="grid grid-cols-2 gap-2 text-sm">
                                <div>
                                    <div className="text-xs text-gray-500">Avg Error</div>
                                    <div className={`font-mono font-bold ${
                                        stats.avgError < 0.3 ? 'text-green-400' :
                                        stats.avgError < 0.7 ? 'text-yellow-400' : 'text-red-400'
                                    }`}>
                                        {stats.avgError.toFixed(3)}m
                                    </div>
                                </div>
                                <div>
                                    <div className="text-xs text-gray-500">Settled</div>
                                    <div className="font-mono font-bold text-gray-200">
                                        {stats.settled}/{stats.count}
                                    </div>
                                </div>
                            </div>
                        </div>
                    ))}
                    {Object.keys(groupStats).length === 0 && (
                        <div className="col-span-2 text-center text-gray-500 py-4">
                            No controller data available
                        </div>
                    )}
                </div>
            </div>

            {/* Error Summary */}
            <div className="grid grid-cols-2 gap-3">
                <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
                    <h3 className="text-sm font-semibold text-gray-200 mb-3 flex items-center">
                        <AlertTriangle className="w-4 h-4 mr-2 text-yellow-400" />
                        Error Summary
                    </h3>
                    <div className="space-y-2">
                        <div className="flex justify-between items-center">
                            <span className="text-xs text-gray-400">Average Error</span>
                            <span className={`font-mono font-bold ${
                                avgError < 0.3 ? 'text-green-400' : avgError < 0.7 ? 'text-yellow-400' : 'text-red-400'
                            }`}>
                                {avgError.toFixed(3)} m
                            </span>
                        </div>
                        <div className="flex justify-between items-center">
                            <span className="text-xs text-gray-400">Max Error</span>
                            <span className="font-mono text-red-400">{errorStats.max.toFixed(3)} m</span>
                        </div>
                        <div className="flex justify-between items-center">
                            <span className="text-xs text-gray-400">Max Error Agent</span>
                            <span className="font-mono text-gray-300">{errorStats.maxAgent}</span>
                        </div>
                        <div className="flex justify-between items-center">
                            <span className="text-xs text-gray-400">Min Error</span>
                            <span className="font-mono text-green-400">{errorStats.min.toFixed(3)} m</span>
                        </div>
                    </div>
                </div>

                {/* Wind Conditions */}
                <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
                    <h3 className="text-sm font-semibold text-gray-200 mb-3 flex items-center">
                        <Wind className="w-4 h-4 mr-2 text-cyan-400" />
                        Wind Conditions
                    </h3>
                    {wind ? (
                        <div className="space-y-2">
                            <div className="flex justify-between items-center">
                                <span className="text-xs text-gray-400">Speed</span>
                                <span className="font-mono text-cyan-300">
                                    {Math.sqrt(wind.velocity.x ** 2 + wind.velocity.y ** 2).toFixed(2)} m/s
                                </span>
                            </div>
                            <div className="flex justify-between items-center">
                                <span className="text-xs text-gray-400">Direction</span>
                                <span className="font-mono text-orange-300">
                                    {((Math.atan2(wind.velocity.y, wind.velocity.x) * 180 / Math.PI + 360) % 360).toFixed(0)}°
                                </span>
                            </div>
                            <div className="flex justify-between items-center">
                                <span className="text-xs text-gray-400">Force X</span>
                                <span className="font-mono text-gray-300">{(wind.force?.x || 0).toFixed(2)} N</span>
                            </div>
                            <div className="flex justify-between items-center">
                                <span className="text-xs text-gray-400">Force Y</span>
                                <span className="font-mono text-gray-300">{(wind.force?.y || 0).toFixed(2)} N</span>
                            </div>
                        </div>
                    ) : (
                        <div className="text-gray-500 text-sm">No wind data</div>
                    )}
                </div>
            </div>

            {/* Topic Health */}
            <div className="bg-gray-800 rounded-lg border border-gray-700 p-4">
                <h3 className="text-sm font-semibold text-gray-200 mb-3 flex items-center">
                    <Radio className="w-4 h-4 mr-2 text-purple-400" />
                    ROS2 Topics Health
                </h3>
                <div className="grid grid-cols-3 gap-4">
                    <div className="text-center">
                        <div className="text-2xl font-bold text-purple-400">{topicStats.nodes}</div>
                        <div className="text-xs text-gray-500">Active Nodes</div>
                    </div>
                    <div className="text-center">
                        <div className="text-2xl font-bold text-cyan-400">{topicStats.total}</div>
                        <div className="text-xs text-gray-500">Topic Connections</div>
                    </div>
                    <div className="text-center">
                        <div className="text-2xl font-bold text-green-400">{agentCount * 3}</div>
                        <div className="text-xs text-gray-500">Est. Messages/s</div>
                    </div>
                </div>
            </div>
        </div>
    );
};

// Status Card Component
const StatusCard: React.FC<{
    label: string;
    value: string;
    subtext?: string;
    icon: React.ReactNode;
    color: 'green' | 'red' | 'blue' | 'gray' | 'purple' | 'cyan' | 'yellow';
}> = ({ label, value, subtext, icon, color }) => {
    const colorClasses = {
        green: 'text-green-400 bg-green-900/30 border-green-800',
        red: 'text-red-400 bg-red-900/30 border-red-800',
        blue: 'text-blue-400 bg-blue-900/30 border-blue-800',
        gray: 'text-gray-400 bg-gray-800/50 border-gray-700',
        purple: 'text-purple-400 bg-purple-900/30 border-purple-800',
        cyan: 'text-cyan-400 bg-cyan-900/30 border-cyan-800',
        yellow: 'text-yellow-400 bg-yellow-900/30 border-yellow-800',
    };

    return (
        <div className={`rounded-lg border p-3 ${colorClasses[color]}`}>
            <div className="flex items-center justify-between mb-1">
                <span className="text-xs text-gray-400 uppercase">{label}</span>
                {icon}
            </div>
            <div className="text-lg font-bold">{value}</div>
            {subtext && <div className="text-xs text-gray-500">{subtext}</div>}
        </div>
    );
};

const OverviewSection: React.FC<{
    metrics: Record<string, MetricsData>;
    wind: WindData | null;
    status: SystemStatus | null;
}> = ({ metrics, wind, status }) => {
    const agentCount = Object.keys(metrics).length;
    const settledCount = Object.values(metrics).filter(m => m.is_settled).length;
    const avgError = agentCount > 0
        ? Object.values(metrics).reduce((sum, m) => sum + m.error_magnitude, 0) / agentCount
        : 0;

    return (
        <div className="space-y-4">
            {/* Status Cards */}
            <div className="grid grid-cols-2 gap-3">
                <div className="bg-gray-800 p-3 rounded-lg border border-gray-700">
                    <div className="text-xs text-gray-400 uppercase tracking-wider mb-1">Run State</div>
                    <div className="text-lg font-bold text-white">
                        {status?.formation_active ? 'Running' : 'Idle'}
                    </div>
                </div>
                <div className="bg-gray-800 p-3 rounded-lg border border-gray-700">
                    <div className="text-xs text-gray-400 uppercase tracking-wider mb-1">Settled Agents</div>
                    <div className="text-lg font-bold text-white">
                        {settledCount} <span className="text-gray-500 text-sm">/ {agentCount}</span>
                    </div>
                </div>
            </div>

            {/* Error Summary */}
            <div className="bg-gray-800 p-4 rounded-lg border border-gray-700">
                <div className="flex items-center justify-between mb-2">
                    <h3 className="text-sm font-semibold text-gray-200">Avg Formation Error</h3>
                    <span className={`text-xs font-bold px-2 py-0.5 rounded ${
                        avgError < 0.5 ? 'bg-green-900 text-green-300' : 'bg-yellow-900 text-yellow-300'
                    }`}>
                        {avgError.toFixed(3)} m
                    </span>
                </div>
                <div className="w-full bg-gray-700 h-2 rounded-full overflow-hidden">
                    <div
                        className="bg-blue-500 h-full transition-all duration-500"
                        style={{ width: `${Math.min(avgError * 20, 100)}%` }}
                    />
                </div>
            </div>

            {/* Wind Summary */}
            {wind && (
                <div className="bg-gray-800 p-4 rounded-lg border border-gray-700">
                    <h3 className="text-sm font-semibold text-gray-200 mb-3 flex items-center">
                        <Wind className="w-4 h-4 mr-2 text-gray-400" />
                        Wind Conditions
                    </h3>
                    <div className="grid grid-cols-2 gap-4">
                        <div>
                            <div className="text-xs text-gray-400">Speed</div>
                            <div className="text-xl font-mono text-blue-300">
                                {Math.sqrt(wind.velocity.x ** 2 + wind.velocity.y ** 2).toFixed(2)}
                                <span className="text-xs text-gray-500 ml-1">m/s</span>
                            </div>
                        </div>
                        <div>
                            <div className="text-xs text-gray-400">Direction</div>
                            <div className="text-xl font-mono text-orange-300">
                                {((Math.atan2(wind.velocity.y, wind.velocity.x) * 180 / Math.PI + 360) % 360).toFixed(0)}°
                            </div>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
};

const ChartsSection: React.FC<{
    metricsHistory: Record<string, MetricsData[]>;
    windHistory: WindData[];
    controllerParams?: Record<string, ControllerParams>;
}> = ({ metricsHistory, windHistory, controllerParams = {} }) => {
    const [chartType, setChartType] = useState<'error' | 'metrics' | 'wind' | 'overshoot'>('error');
    const [timeWindow, setTimeWindow] = useState<number>(60);
    const [viewMode, setViewMode] = useState<'individual' | 'aggregated'>('individual');

    return (
        <div className="space-y-4">
            <div className="flex items-center justify-between gap-3">
                <div className="flex space-x-2 overflow-x-auto pb-2">
                    {(['error', 'metrics', 'wind', 'overshoot'] as const).map((type) => (
                        <button
                            key={type}
                            onClick={() => setChartType(type)}
                            className={`px-3 py-1 rounded text-xs font-medium transition-all whitespace-nowrap ${
                                chartType === type
                                    ? 'bg-blue-600 text-white'
                                    : 'bg-gray-700 text-gray-400 hover:bg-gray-600'
                            }`}
                        >
                            {type === 'error' && 'Position Error'}
                            {type === 'metrics' && 'IAE/ITAE'}
                            {type === 'wind' && 'Wind Velocity'}
                            {type === 'overshoot' && 'Overshoot'}
                        </button>
                    ))}
                </div>
                <div className="flex items-center gap-2">
                    {chartType !== 'wind' && (
                        <button
                            onClick={() => setViewMode(viewMode === 'individual' ? 'aggregated' : 'individual')}
                            className={`flex items-center gap-1.5 px-3 py-1.5 rounded text-xs font-medium transition-all whitespace-nowrap ${
                                viewMode === 'aggregated'
                                    ? 'bg-emerald-600 text-white shadow-lg shadow-emerald-600/50'
                                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                            }`}
                            title="Toggle between individual agents and controller group averages"
                        >
                            {viewMode === 'aggregated' ? (
                                <>
                                    <Users className="w-3.5 h-3.5" />
                                    <span>Controller Groups</span>
                                </>
                            ) : (
                                <>
                                    <Users className="w-3.5 h-3.5" />
                                    <span>Individual Agents</span>
                                </>
                            )}
                        </button>
                    )}
                    <select
                        value={timeWindow}
                        onChange={(e) => setTimeWindow(Number(e.target.value))}
                        className="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs"
                    >
                        <option value={30}>Last 30s</option>
                        <option value={60}>Last 60s</option>
                        <option value={120}>Last 2min</option>
                        <option value={0}>Full Run</option>
                    </select>
                </div>
            </div>
            <div className="h-72">
                <TimeSeries
                    metricsHistory={metricsHistory}
                    windHistory={windHistory}
                    selectedAgent={null}
                    chartType={chartType}
                    timeWindow={timeWindow}
                    viewMode={viewMode}
                    controllerParams={controllerParams}
                />
            </div>
        </div>
    );
};

const AgentsSection: React.FC<{
    metrics: Record<string, MetricsData>;
    controllerParams?: Record<string, ControllerParams>;
}> = ({ metrics, controllerParams = {} }) => {
    const [sortBy, setSortBy] = useState<'name' | 'error'>('name');
    const [filterGroup, setFilterGroup] = useState<string>('all');

    const sortedAgents = useMemo(() => {
        let entries = Object.entries(metrics);

        // Filter by group
        if (filterGroup !== 'all') {
            entries = entries.filter(([id]) => {
                const params = controllerParams[id];
                return params?.controller_type === filterGroup;
            });
        }

        // Sort
        if (sortBy === 'error') {
            entries.sort((a, b) => b[1].error_magnitude - a[1].error_magnitude);
        } else {
            entries.sort((a, b) => a[0].localeCompare(b[0]));
        }

        return entries;
    }, [metrics, controllerParams, sortBy, filterGroup]);

    const controllerTypes = useMemo(() => {
        const types = new Set<string>();
        Object.values(controllerParams).forEach(p => types.add(p.controller_type));
        return Array.from(types);
    }, [controllerParams]);

    return (
        <div className="space-y-3">
            {/* Filters */}
            <div className="flex items-center space-x-3">
                <select
                    value={sortBy}
                    onChange={(e) => setSortBy(e.target.value as 'name' | 'error')}
                    className="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs"
                >
                    <option value="name">Sort by Name</option>
                    <option value="error">Sort by Error</option>
                </select>
                <select
                    value={filterGroup}
                    onChange={(e) => setFilterGroup(e.target.value)}
                    className="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs"
                >
                    <option value="all">All Groups</option>
                    {controllerTypes.map(type => (
                        <option key={type} value={type}>{type.toUpperCase()}</option>
                    ))}
                </select>
            </div>

            {/* Agent List */}
            {sortedAgents.map(([id, m]) => {
                const params = controllerParams[id];
                return (
                    <div key={id} className="bg-gray-800 p-3 rounded border border-gray-700 flex items-center justify-between">
                        <div className="flex items-center space-x-3">
                            <div>
                                <div className="font-medium text-sm text-gray-200">{id}</div>
                                <div className="text-xs text-gray-500">
                                    Err: {m.error_magnitude.toFixed(3)}m | RMSE: {(m.rmse_total || 0).toFixed(3)}
                                </div>
                            </div>
                        </div>
                        <div className="flex items-center space-x-2">
                            {params && (
                                <span className={`px-2 py-0.5 rounded text-xs font-bold ${
                                    params.controller_type === 'hybrid'
                                        ? 'bg-emerald-900/50 text-emerald-400'
                                        : 'bg-blue-900/50 text-blue-400'
                                }`}>
                                    {params.controller_type === 'hybrid' ? 'PID+Fuzzy' : 'PID'}
                                </span>
                            )}
                            <div className={`px-2 py-1 rounded text-xs font-bold ${
                                m.is_settled ? 'bg-green-900/50 text-green-400' : 'bg-yellow-900/50 text-yellow-400'
                            }`}>
                                {m.is_settled ? 'SETTLED' : 'MOVING'}
                            </div>
                        </div>
                    </div>
                );
            })}
            {sortedAgents.length === 0 && (
                <div className="text-center text-gray-500 py-8 text-sm">No agents active</div>
            )}
        </div>
    );
};
