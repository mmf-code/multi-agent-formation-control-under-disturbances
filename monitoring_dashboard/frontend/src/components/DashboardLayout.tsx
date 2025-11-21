import React, { useState, useCallback, useEffect } from 'react';
import { Activity, Play, Square, Settings, Network, Terminal, ChevronDown, ChevronUp, GripVertical, Loader2 } from 'lucide-react';

interface SimulationStatus {
    running: boolean;
    pid: number | null;
    script: string | null;
}

interface ScriptInfo {
    name: string;
    path: string;
}

interface DashboardLayoutProps {
    connected: boolean;
    systemStatus: {
        ros_ok: boolean;
        active_agents: string[];
        active_topics?: string[];
    } | null;
    leftPanel: React.ReactNode;
    rightPanel: React.ReactNode;
    bottomPanel: React.ReactNode;
}

export const DashboardLayout: React.FC<DashboardLayoutProps> = ({
    connected,
    systemStatus,
    leftPanel,
    rightPanel,
    bottomPanel,
}) => {
    const [leftWidth, setLeftWidth] = useState(50); // percentage
    const [bottomHeight, setBottomHeight] = useState(180); // pixels
    const [bottomCollapsed, setBottomCollapsed] = useState(false);
    const [isResizingH, setIsResizingH] = useState(false);
    const [isResizingV, setIsResizingV] = useState(false);

    // Simulation control state
    const [simStatus, setSimStatus] = useState<SimulationStatus>({ running: false, pid: null, script: null });
    const [availableScripts, setAvailableScripts] = useState<ScriptInfo[]>([]);
    const [selectedScript, setSelectedScript] = useState<string>('run_formation_demo.sh');
    const [simLoading, setSimLoading] = useState(false);
    const [showScriptDropdown, setShowScriptDropdown] = useState(false);

    // API base URL (handle both dev proxy and production)
    const getApiUrl = (path: string) => {
        // In production, we're served from /ui/, but API is at root
        if (window.location.pathname.startsWith('/ui')) {
            return path; // Backend serves API at root
        }
        return path;
    };

    // Fetch simulation status and available scripts
    useEffect(() => {
        const fetchSimData = async () => {
            try {
                const [statusRes, scriptsRes] = await Promise.all([
                    fetch(getApiUrl('/api/simulation/status')),
                    fetch(getApiUrl('/api/simulation/scripts'))
                ]);
                if (statusRes.ok) {
                    const status = await statusRes.json();
                    setSimStatus(status);
                }
                if (scriptsRes.ok) {
                    const data = await scriptsRes.json();
                    setAvailableScripts(data.scripts || []);
                }
            } catch (e) {
                console.error('Failed to fetch simulation data:', e);
            }
        };

        fetchSimData();
        const interval = setInterval(fetchSimData, 3000); // Poll every 3s
        return () => clearInterval(interval);
    }, []);

    const startSimulation = async () => {
        setSimLoading(true);
        try {
            const url = getApiUrl(`/api/simulation/start?script=${encodeURIComponent(selectedScript)}`);
            console.log('Starting simulation:', url, selectedScript);
            const res = await fetch(url, {
                method: 'POST'
            });
            if (res.ok) {
                const data = await res.json();
                setSimStatus({ running: true, pid: data.pid, script: data.script });
            } else {
                const err = await res.json();
                alert(`Failed to start: ${err.error}`);
            }
        } catch (e) {
            console.error('Start simulation error:', e);
        } finally {
            setSimLoading(false);
        }
    };

    const stopSimulation = async () => {
        setSimLoading(true);
        try {
            const res = await fetch(getApiUrl('/api/simulation/stop'), { method: 'POST' });
            if (res.ok) {
                setSimStatus({ running: false, pid: null, script: null });
            }
        } catch (e) {
            console.error('Stop simulation error:', e);
        } finally {
            setSimLoading(false);
        }
    };

    const handleHorizontalResize = useCallback((e: React.MouseEvent) => {
        e.preventDefault();
        setIsResizingH(true);
        const startX = e.clientX;
        const startWidth = leftWidth;

        const onMouseMove = (e: MouseEvent) => {
            const delta = e.clientX - startX;
            const containerWidth = window.innerWidth;
            const newWidth = startWidth + (delta / containerWidth) * 100;
            setLeftWidth(Math.max(25, Math.min(75, newWidth)));
        };

        const onMouseUp = () => {
            setIsResizingH(false);
            document.removeEventListener('mousemove', onMouseMove);
            document.removeEventListener('mouseup', onMouseUp);
        };

        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
    }, [leftWidth]);

    const handleVerticalResize = useCallback((e: React.MouseEvent) => {
        e.preventDefault();
        setIsResizingV(true);
        const startY = e.clientY;
        const startHeight = bottomHeight;

        const onMouseMove = (e: MouseEvent) => {
            const delta = startY - e.clientY;
            setBottomHeight(Math.max(100, Math.min(400, startHeight + delta)));
        };

        const onMouseUp = () => {
            setIsResizingV(false);
            document.removeEventListener('mousemove', onMouseMove);
            document.removeEventListener('mouseup', onMouseUp);
        };

        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
    }, [bottomHeight]);

    const topicCount = systemStatus?.active_topics?.length || 0;

    return (
        <div className={`flex flex-col h-screen bg-gray-950 text-gray-100 overflow-hidden font-sans selection:bg-blue-500/30 ${isResizingH || isResizingV ? 'select-none' : ''}`}>
            {/* Global Header */}
            <header className="h-14 bg-gray-900 border-b border-gray-800 flex items-center justify-between px-4 shrink-0 z-10 shadow-sm">
                <div className="flex items-center space-x-4">
                    <div className="flex items-center space-x-2 text-blue-400">
                        <Activity className="w-5 h-5" />
                        <span className="font-bold text-lg tracking-tight text-gray-100">Formation Control Dashboard</span>
                    </div>
                    <div className="h-6 w-px bg-gray-700 mx-2" />

                    {/* Simulation Controls */}
                    <div className="flex items-center space-x-2">
                        {/* Script Selector */}
                        <div className="relative">
                            <button
                                onClick={() => setShowScriptDropdown(!showScriptDropdown)}
                                className="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs text-gray-300 hover:bg-gray-700 flex items-center space-x-1 max-w-40"
                                disabled={simStatus.running}
                            >
                                <span className="truncate">{selectedScript}</span>
                                <ChevronDown className="w-3 h-3 flex-shrink-0" />
                            </button>
                            {showScriptDropdown && (
                                <div className="absolute top-full left-0 mt-1 bg-gray-800 border border-gray-700 rounded shadow-lg z-50 max-h-48 overflow-y-auto min-w-48">
                                    {availableScripts.map(script => (
                                        <button
                                            key={script.name}
                                            onClick={() => {
                                                setSelectedScript(script.name);
                                                setShowScriptDropdown(false);
                                            }}
                                            className={`w-full text-left px-3 py-1.5 text-xs hover:bg-gray-700 ${
                                                selectedScript === script.name ? 'text-blue-400' : 'text-gray-300'
                                            }`}
                                        >
                                            {script.name}
                                        </button>
                                    ))}
                                    {availableScripts.length === 0 && (
                                        <div className="px-3 py-2 text-xs text-gray-500">No scripts found</div>
                                    )}
                                </div>
                            )}
                        </div>

                        {/* Control Buttons */}
                        <div className="flex items-center space-x-1 bg-gray-800 rounded-md p-1">
                            {simLoading ? (
                                <div className="p-1.5">
                                    <Loader2 className="w-4 h-4 animate-spin text-gray-400" />
                                </div>
                            ) : simStatus.running ? (
                                <button
                                    onClick={stopSimulation}
                                    className="p-1.5 hover:bg-gray-700 rounded text-red-400 transition-colors"
                                    title="Stop Simulation"
                                >
                                    <Square className="w-4 h-4 fill-current" />
                                </button>
                            ) : (
                                <button
                                    onClick={startSimulation}
                                    className="p-1.5 hover:bg-gray-700 rounded text-green-400 transition-colors"
                                    title={`Start ${selectedScript}`}
                                >
                                    <Play className="w-4 h-4 fill-current" />
                                </button>
                            )}
                        </div>

                        {/* Running indicator */}
                        {simStatus.running && (
                            <div className="flex items-center space-x-1 text-xs text-green-400">
                                <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse" />
                                <span className="hidden sm:inline">Running</span>
                            </div>
                        )}
                    </div>
                </div>

                <div className="flex items-center space-x-6">
                    {/* Status Indicators */}
                    <div className="flex items-center space-x-4 text-xs font-medium text-gray-400">
                        <div className="flex items-center space-x-2">
                            <Network className="w-3.5 h-3.5" />
                            <span>ROS2:</span>
                            <span className={systemStatus?.ros_ok ? 'text-green-400' : 'text-red-400'}>
                                {systemStatus?.ros_ok ? 'OK' : 'ERR'}
                            </span>
                        </div>
                        <div className="flex items-center space-x-2">
                            <Settings className="w-3.5 h-3.5" />
                            <span>Agents:</span>
                            <span className="text-gray-200">{systemStatus?.active_agents.length || 0}</span>
                        </div>
                        <div className="flex items-center space-x-2">
                            <span>Topics:</span>
                            <span className="text-gray-200">{topicCount}</span>
                        </div>
                    </div>

                    {/* Connection Status */}
                    <div className={`flex items-center space-x-2 px-2 py-1 rounded border ${connected
                        ? 'bg-green-500/10 border-green-500/20 text-green-400'
                        : 'bg-red-500/10 border-red-500/20 text-red-400'
                        }`}>
                        <div className={`w-1.5 h-1.5 rounded-full ${connected ? 'bg-green-400 animate-pulse' : 'bg-red-400'}`} />
                        <span className="text-xs font-semibold uppercase tracking-wider">
                            {connected ? 'Live' : 'Offline'}
                        </span>
                    </div>
                </div>
            </header>

            {/* Main Content Area - Resizable 2-Column Layout */}
            <div className="flex-1 flex overflow-hidden">
                {/* Left Column: Simulation & Metrics */}
                <div
                    className="flex flex-col border-r border-gray-800 bg-gray-900/50"
                    style={{ width: `${leftWidth}%` }}
                >
                    <div className="flex-1 overflow-y-auto scrollbar-thin scrollbar-thumb-gray-700 scrollbar-track-transparent">
                        {leftPanel}
                    </div>
                </div>

                {/* Horizontal Resize Handle */}
                <div
                    className={`w-1 bg-gray-800 hover:bg-blue-500 cursor-col-resize flex items-center justify-center transition-colors ${isResizingH ? 'bg-blue-500' : ''}`}
                    onMouseDown={handleHorizontalResize}
                >
                    <GripVertical className="w-3 h-3 text-gray-600" />
                </div>

                {/* Right Column: ROS2 Graph */}
                <div
                    className="flex flex-col bg-gray-950 relative"
                    style={{ width: `${100 - leftWidth}%` }}
                >
                    <div className="absolute inset-0 overflow-hidden">
                        {rightPanel}
                    </div>
                </div>
            </div>

            {/* Vertical Resize Handle */}
            {!bottomCollapsed && (
                <div
                    className={`h-1 bg-gray-800 hover:bg-blue-500 cursor-row-resize transition-colors ${isResizingV ? 'bg-blue-500' : ''}`}
                    onMouseDown={handleVerticalResize}
                />
            )}

            {/* Bottom Strip: Diagnostics */}
            <div
                className="bg-gray-900 border-t border-gray-800 flex flex-col shrink-0"
                style={{ height: bottomCollapsed ? '32px' : `${bottomHeight}px` }}
            >
                <div
                    className="flex items-center px-4 py-1 bg-gray-800/50 border-b border-gray-800 cursor-pointer hover:bg-gray-800 transition-colors"
                    onClick={() => setBottomCollapsed(!bottomCollapsed)}
                >
                    <Terminal className="w-3.5 h-3.5 mr-2 text-gray-400" />
                    <span className="text-xs font-medium text-gray-400 uppercase tracking-wider flex-1">System Diagnostics & Logs</span>
                    {bottomCollapsed ? <ChevronUp className="w-4 h-4 text-gray-400" /> : <ChevronDown className="w-4 h-4 text-gray-400" />}
                </div>
                {!bottomCollapsed && (
                    <div className="flex-1 overflow-y-auto p-0">
                        {bottomPanel}
                    </div>
                )}
            </div>
        </div>
    );
};
