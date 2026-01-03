/**
 * ROS Graph Panel - rqt_graph style visualization
 * Shows nodes (rectangles), topics (rounded), and directed connections
 * Theme-aware: supports light/dark modes
 */
import React, { useState, useMemo, useRef, useCallback } from 'react';
import { RosGraphData, ControllerParams } from '../api/ws';
import { ZoomIn, ZoomOut, Filter, Eye, EyeOff, Maximize2, Activity, Download, Network } from 'lucide-react';
import html2canvas from 'html2canvas';
import { useThesisMode } from '../store/theme';

// Friendly names for nodes
const NODE_FRIENDLY_NAMES: Record<string, string> = {
    'simple_drone_plugin': 'Drone Controller',
    'gazebo': 'Gazebo Simulator',
    'monitoring_dashboard_bridge': 'Dashboard Bridge',
    'simple_metrics_logger': 'Metrics Logger',
    'formation_coordinator_node': 'Formation Coordinator',
    'wind_plugin': 'Wind Generator',
};

interface Position {
    x: number;
    y: number;
}

interface LayoutNode {
    id: string;
    label: string;
    type: 'node' | 'topic';
    role?: string;
    controllerType?: string;
    agentId?: string;
    x: number;
    y: number;
    width: number;
    height: number;
}

interface LayoutEdge {
    id: string;
    source: string;
    target: string;
    sourcePos: Position;
    targetPos: Position;
}

interface RosGraphPanelProps {
    graphData: RosGraphData | null;
    controllerParams?: Record<string, ControllerParams>;
}

// System topics to hide by default
const HIDDEN_PATTERNS = [
    '/parameter_events',
    '/rosout',
    '/tf_static',
    '/robot_description',
    '/describe_parameters',
    '/get_parameter',
    '/list_parameters',
    '/set_parameters',
    '/_action/',
    '/bond',
    '/transition_event',
    '/events/read',
    '/events/write',
];

// Core topics to show in Strict Mode
const CORE_TOPICS = [
    'metrics',
    'odom',
    'formation',
    'wind',
    'target',
    'cmd_vel'
];

// ============================================================================
// Robotics Color Palette - Theme Aware
// ============================================================================

// Dark mode colors (neon/cyber style)
const DARK_COLORS = {
    // Controller types - Robotics inspired
    hybrid: { bg: 'rgba(6, 182, 212, 0.2)', border: '#06b6d4', text: '#06b6d4' },      // Cyan - Primary/Best
    pid_fuzzy: { bg: 'rgba(6, 182, 212, 0.2)', border: '#06b6d4', text: '#06b6d4' },  // Cyan
    pid: { bg: 'rgba(168, 85, 247, 0.2)', border: '#a855f7', text: '#a855f7' },        // Purple - Standard
    pd: { bg: 'rgba(251, 146, 60, 0.2)', border: '#fb923c', text: '#fb923c' },         // Orange - Basic

    // Roles
    controller: { bg: 'rgba(6, 182, 212, 0.15)', border: '#06b6d4', text: '#06b6d4' },
    sensor: { bg: 'rgba(34, 197, 94, 0.15)', border: '#22c55e', text: '#22c55e' },
    metrics: { bg: 'rgba(168, 85, 247, 0.15)', border: '#a855f7', text: '#a855f7' },
    agent_node: { bg: 'rgba(251, 146, 60, 0.15)', border: '#fb923c', text: '#fb923c' },
    system: { bg: 'rgba(100, 116, 139, 0.15)', border: '#64748b', text: '#94a3b8' },

    // UI
    topic: { bg: 'rgba(71, 85, 105, 0.3)', border: '#64748b', text: '#cbd5e1' },
    edge: '#64748b',
    edgeActive: '#06b6d4',
    text: '#e2e8f0',
    textMuted: '#94a3b8',
    bg: '#0f172a',
    bgPanel: 'rgba(15, 23, 42, 0.9)',
};

// Light mode colors (professional/academic style)
const LIGHT_COLORS = {
    // Controller types
    hybrid: { bg: 'rgba(6, 182, 212, 0.15)', border: '#0891b2', text: '#0891b2' },     // Cyan 600
    pid_fuzzy: { bg: 'rgba(6, 182, 212, 0.15)', border: '#0891b2', text: '#0891b2' }, // Cyan 600
    pid: { bg: 'rgba(147, 51, 234, 0.12)', border: '#7c3aed', text: '#7c3aed' },       // Violet 600
    pd: { bg: 'rgba(234, 88, 12, 0.12)', border: '#c2410c', text: '#c2410c' },         // Orange 700

    // Roles
    controller: { bg: 'rgba(6, 182, 212, 0.1)', border: '#0891b2', text: '#0891b2' },
    sensor: { bg: 'rgba(22, 163, 74, 0.1)', border: '#16a34a', text: '#16a34a' },
    metrics: { bg: 'rgba(147, 51, 234, 0.1)', border: '#7c3aed', text: '#7c3aed' },
    agent_node: { bg: 'rgba(234, 88, 12, 0.1)', border: '#c2410c', text: '#c2410c' },
    system: { bg: 'rgba(100, 116, 139, 0.1)', border: '#64748b', text: '#475569' },

    // UI
    topic: { bg: '#f1f5f9', border: '#94a3b8', text: '#334155' },
    edge: '#94a3b8',
    edgeActive: '#0891b2',
    text: '#1e293b',
    textMuted: '#64748b',
    bg: '#ffffff',
    bgPanel: 'rgba(255, 255, 255, 0.95)',
};

export const RosGraphPanel: React.FC<RosGraphPanelProps> = ({ graphData, controllerParams }) => {
    const thesisMode = useThesisMode();
    const colors = thesisMode ? LIGHT_COLORS : DARK_COLORS;

    // Helper to get friendly name and controller type for a node
    const getNodeInfo = (nodeName: string) => {
        const parts = nodeName.split('/').filter(Boolean);
        const baseName = parts[parts.length - 1];
        const agentMatch = nodeName.match(/agent_(\d+)/);
        const agentId = agentMatch ? `agent_${agentMatch[1]}` : undefined;

        let friendlyName = NODE_FRIENDLY_NAMES[baseName] || baseName;
        let controllerType: string | undefined;

        if (agentId && controllerParams?.[agentId]) {
            controllerType = controllerParams[agentId].controller_type;
            const ctrlLabel = controllerType === 'hybrid' || controllerType === 'pid_fuzzy' ? 'PID+Fuzzy' : controllerType?.toUpperCase() || '';
            friendlyName = `${agentId} (${ctrlLabel})`;
        } else if (agentId) {
            friendlyName = `${agentId} - ${friendlyName}`;
        }

        return { friendlyName, controllerType, agentId };
    };

    const containerRef = useRef<HTMLDivElement>(null);
    const [scale, setScale] = useState(0.7);
    const [pan, setPan] = useState({ x: 100, y: 80 });
    const [isDragging, setIsDragging] = useState(false);
    const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
    const [selectedItem, setSelectedItem] = useState<string | null>(null);

    // Filters
    const [showSystemTopics, setShowSystemTopics] = useState(false);
    const [filterAgentsOnly, setFilterAgentsOnly] = useState(false);
    const [strictMode, setStrictMode] = useState(true);
    const [searchQuery, setSearchQuery] = useState('');

    // Build rqt_graph style layout
    const layout = useMemo(() => {
        if (!graphData || graphData.nodes.length === 0) {
            return { nodes: [], edges: [], topics: [] };
        }

        const topicSet = new Map<string, { name: string; msgType: string; publishers: string[]; subscribers: string[] }>();

        graphData.edges.forEach(edge => {
            const topicName = edge.topicName || 'unknown';

            if (!showSystemTopics) {
                if (HIDDEN_PATTERNS.some(p => topicName.includes(p))) return;
            }

            if (strictMode) {
                if (!CORE_TOPICS.some(core => topicName.includes(core))) return;
            }

            if (!topicSet.has(topicName)) {
                topicSet.set(topicName, {
                    name: topicName,
                    msgType: edge.msgType || '',
                    publishers: [],
                    subscribers: []
                });
            }

            const topic = topicSet.get(topicName)!;
            if (!topic.publishers.includes(edge.source)) {
                topic.publishers.push(edge.source);
            }
            if (!topic.subscribers.includes(edge.target)) {
                topic.subscribers.push(edge.target);
            }
        });

        let visibleNodes = graphData.nodes.filter(n => {
            if (filterAgentsOnly && !n.name.includes('agent')) return false;
            if (searchQuery && !n.name.toLowerCase().includes(searchQuery.toLowerCase())) return false;
            return true;
        });

        const connectedNodeIds = new Set<string>();
        topicSet.forEach(topic => {
            topic.publishers.forEach(id => connectedNodeIds.add(id));
            topic.subscribers.forEach(id => connectedNodeIds.add(id));
        });

        if (!searchQuery) {
            visibleNodes = visibleNodes.filter(n => connectedNodeIds.has(n.id));
        }

        const NODE_WIDTH = 140;
        const NODE_HEIGHT = 36;
        const TOPIC_WIDTH = 160;
        const TOPIC_HEIGHT = 28;
        const COL_SPACING = 280;
        const ROW_SPACING = 50;

        const nodeRoles = new Map<string, 'publisher' | 'subscriber' | 'both'>();

        visibleNodes.forEach(node => {
            let isPub = false;
            let isSub = false;

            topicSet.forEach(topic => {
                if (topic.publishers.includes(node.id)) isPub = true;
                if (topic.subscribers.includes(node.id)) isSub = true;
            });

            if (isPub && isSub) nodeRoles.set(node.id, 'both');
            else if (isPub) nodeRoles.set(node.id, 'publisher');
            else if (isSub) nodeRoles.set(node.id, 'subscriber');
        });

        const publishers = visibleNodes.filter(n => nodeRoles.get(n.id) === 'publisher' || nodeRoles.get(n.id) === 'both');
        const subscribers = visibleNodes.filter(n => nodeRoles.get(n.id) === 'subscriber');
        const topics = Array.from(topicSet.values());

        const layoutNodes: LayoutNode[] = [];

        publishers.forEach((node, i) => {
            const info = getNodeInfo(node.name);
            layoutNodes.push({
                id: node.id,
                label: info.friendlyName,
                type: 'node',
                role: node.role,
                controllerType: info.controllerType,
                agentId: info.agentId,
                x: 0,
                y: i * ROW_SPACING,
                width: NODE_WIDTH,
                height: NODE_HEIGHT
            });
        });

        topics.forEach((topic, i) => {
            layoutNodes.push({
                id: `topic:${topic.name}`,
                label: topic.name.split('/').slice(-2).join('/'),
                type: 'topic',
                x: COL_SPACING,
                y: i * (ROW_SPACING * 0.8),
                width: TOPIC_WIDTH,
                height: TOPIC_HEIGHT
            });
        });

        const pureSubscribers = subscribers.filter(n => nodeRoles.get(n.id) === 'subscriber');
        pureSubscribers.forEach((node, i) => {
            const info = getNodeInfo(node.name);
            layoutNodes.push({
                id: node.id,
                label: info.friendlyName,
                type: 'node',
                role: node.role,
                controllerType: info.controllerType,
                agentId: info.agentId,
                x: COL_SPACING * 2,
                y: i * ROW_SPACING,
                width: NODE_WIDTH,
                height: NODE_HEIGHT
            });
        });

        const layoutEdges: LayoutEdge[] = [];
        let edgeId = 0;

        const getNodePos = (id: string): Position | null => {
            const node = layoutNodes.find(n => n.id === id);
            if (!node) return null;
            return { x: node.x + node.width / 2, y: node.y + node.height / 2 };
        };

        topics.forEach(topic => {
            const topicNode = layoutNodes.find(n => n.id === `topic:${topic.name}`);
            if (!topicNode) return;

            const topicPos = { x: topicNode.x + topicNode.width / 2, y: topicNode.y + topicNode.height / 2 };

            topic.publishers.forEach(pubId => {
                const pubPos = getNodePos(pubId);
                if (pubPos) {
                    layoutEdges.push({
                        id: `edge-${edgeId++}`,
                        source: pubId,
                        target: `topic:${topic.name}`,
                        sourcePos: { x: pubPos.x + NODE_WIDTH / 2, y: pubPos.y },
                        targetPos: { x: topicNode.x, y: topicPos.y }
                    });
                }
            });

            topic.subscribers.forEach(subId => {
                const subPos = getNodePos(subId);
                if (subPos) {
                    layoutEdges.push({
                        id: `edge-${edgeId++}`,
                        source: `topic:${topic.name}`,
                        target: subId,
                        sourcePos: { x: topicNode.x + topicNode.width, y: topicPos.y },
                        targetPos: { x: subPos.x - NODE_WIDTH / 2, y: subPos.y }
                    });
                }
            });
        });

        return { nodes: layoutNodes, edges: layoutEdges, topics };
    }, [graphData, showSystemTopics, filterAgentsOnly, searchQuery, strictMode, controllerParams]);

    // Export to PNG
    const exportToPNG = () => {
        if (containerRef.current) {
            html2canvas(containerRef.current, {
                scale: 2,
                backgroundColor: colors.bg,
            }).then(canvas => {
                const link = document.createElement('a');
                link.download = `ros_graph_${Date.now()}.png`;
                link.href = canvas.toDataURL();
                link.click();
            });
        }
    };

    // Mouse handlers
    const handleMouseDown = useCallback((e: React.MouseEvent) => {
        if ((e.target as HTMLElement).closest('.graph-item')) return;
        setIsDragging(true);
        setDragStart({ x: e.clientX - pan.x, y: e.clientY - pan.y });
    }, [pan]);

    const handleMouseMove = useCallback((e: React.MouseEvent) => {
        if (!isDragging) return;
        setPan({
            x: e.clientX - dragStart.x,
            y: e.clientY - dragStart.y
        });
    }, [isDragging, dragStart]);

    const handleMouseUp = useCallback(() => {
        setIsDragging(false);
    }, []);

    const handleWheel = useCallback((e: React.WheelEvent) => {
        e.preventDefault();
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        setScale(s => Math.max(0.2, Math.min(2, s * delta)));
    }, []);

    const resetView = useCallback(() => {
        setScale(0.7);
        setPan({ x: 100, y: 80 });
    }, []);

    // Get node color based on controller type or role
    const getNodeColor = (controllerType?: string, role?: string) => {
        if (controllerType) {
            const colorKey = controllerType as keyof typeof colors;
            if (colors[colorKey]) {
                return colors[colorKey] as { bg: string; border: string; text: string };
            }
        }
        if (role) {
            const roleKey = role as keyof typeof colors;
            if (colors[roleKey]) {
                return colors[roleKey] as { bg: string; border: string; text: string };
            }
        }
        return colors.system;
    };

    // Empty state
    if (!graphData) {
        return (
            <div
                className="w-full h-full flex items-center justify-center"
                style={{ background: 'var(--color-bg-secondary)' }}
            >
                <div className="text-center animate-pulse">
                    <Network className="w-12 h-12 mx-auto mb-3" style={{ color: 'var(--color-text-muted)' }} />
                    <div className="text-lg mb-2 font-medium" style={{ color: 'var(--color-text-secondary)' }}>
                        Waiting for ROS2 graph data...
                    </div>
                    <div className="text-sm" style={{ color: 'var(--color-text-muted)' }}>
                        Start the simulation to see the graph
                    </div>
                </div>
            </div>
        );
    }

    return (
        <div
            ref={containerRef}
            className="w-full h-full relative overflow-hidden"
            style={{ background: colors.bg }}
            onWheel={handleWheel}
        >
            {/* Background Grid */}
            <div className="absolute inset-0 bg-grid-pattern opacity-30 pointer-events-none" />

            {/* Toolbar */}
            <div className="absolute top-3 left-3 right-3 flex items-center justify-between z-20">
                <div
                    className="flex items-center space-x-3 px-3 py-1.5 rounded-lg shadow-sm border"
                    style={{ background: colors.bgPanel, borderColor: 'var(--color-border-subtle)' }}
                >
                    <h3 className="text-sm font-bold uppercase tracking-wider" style={{ color: colors.hybrid.text }}>
                        ROS2 Graph
                    </h3>
                    <span className="text-xs border-l pl-3" style={{ color: colors.textMuted, borderColor: 'var(--color-border-subtle)' }}>
                        {layout.nodes.filter(n => n.type === 'node').length} nodes |{' '}
                        {layout.nodes.filter(n => n.type === 'topic').length} topics
                    </span>
                </div>

                <div className="flex items-center space-x-2">
                    <input
                        type="text"
                        placeholder="Search..."
                        value={searchQuery}
                        onChange={(e) => setSearchQuery(e.target.value)}
                        className="rounded px-2 py-1 text-xs w-28 focus:outline-none transition-colors border"
                        style={{
                            background: colors.bgPanel,
                            borderColor: 'var(--color-border-subtle)',
                            color: colors.text,
                        }}
                    />

                    <button
                        onClick={() => setFilterAgentsOnly(!filterAgentsOnly)}
                        className="px-2 py-1 rounded text-xs border transition-all duration-300"
                        style={{
                            background: filterAgentsOnly ? colors.hybrid.bg : colors.bgPanel,
                            borderColor: filterAgentsOnly ? colors.hybrid.border : 'var(--color-border-subtle)',
                            color: filterAgentsOnly ? colors.hybrid.text : colors.textMuted,
                        }}
                    >
                        <Filter className="w-3.5 h-3.5 inline mr-1" />
                        Agents
                    </button>

                    <button
                        onClick={() => setShowSystemTopics(!showSystemTopics)}
                        className="px-2 py-1 rounded text-xs border transition-all duration-300"
                        style={{
                            background: showSystemTopics ? colors.pid.bg : colors.bgPanel,
                            borderColor: showSystemTopics ? colors.pid.border : 'var(--color-border-subtle)',
                            color: showSystemTopics ? colors.pid.text : colors.textMuted,
                        }}
                    >
                        {showSystemTopics ? <Eye className="w-3.5 h-3.5 inline mr-1" /> : <EyeOff className="w-3.5 h-3.5 inline mr-1" />}
                        System
                    </button>

                    <button
                        onClick={() => setStrictMode(!strictMode)}
                        className="px-2 py-1 rounded text-xs border transition-all duration-300"
                        style={{
                            background: strictMode ? colors.sensor.bg : colors.bgPanel,
                            borderColor: strictMode ? colors.sensor.border : 'var(--color-border-subtle)',
                            color: strictMode ? colors.sensor.text : colors.textMuted,
                        }}
                        title="Show only core formation topics"
                    >
                        <Activity className="w-3.5 h-3.5 inline mr-1" />
                        Focus
                    </button>

                    <div
                        className="flex items-center space-x-1 ml-2 p-1 rounded-lg border"
                        style={{ background: colors.bgPanel, borderColor: 'var(--color-border-subtle)' }}
                    >
                        <button
                            onClick={() => setScale(s => Math.min(s * 1.2, 2))}
                            className="p-1.5 rounded transition-colors hover:opacity-70"
                            style={{ color: colors.textMuted }}
                        >
                            <ZoomIn className="w-4 h-4" />
                        </button>
                        <button
                            onClick={() => setScale(s => Math.max(s / 1.2, 0.2))}
                            className="p-1.5 rounded transition-colors hover:opacity-70"
                            style={{ color: colors.textMuted }}
                        >
                            <ZoomOut className="w-4 h-4" />
                        </button>
                        <button
                            onClick={resetView}
                            className="p-1.5 rounded transition-colors hover:opacity-70"
                            title="Reset View"
                            style={{ color: colors.textMuted }}
                        >
                            <Maximize2 className="w-4 h-4" />
                        </button>
                        <button
                            onClick={exportToPNG}
                            className="p-1.5 rounded transition-colors hover:opacity-70"
                            title="Export to PNG"
                            style={{ color: colors.hybrid.text }}
                        >
                            <Download className="w-4 h-4" />
                        </button>
                    </div>
                </div>
            </div>

            {/* Legend */}
            <div
                className="absolute bottom-3 left-3 p-3 rounded-lg z-20 shadow-sm border"
                style={{ background: colors.bgPanel, borderColor: 'var(--color-border-subtle)' }}
            >
                <div className="text-xs font-bold mb-2 uppercase tracking-wide" style={{ color: colors.textMuted }}>
                    Controller Types
                </div>
                <div className="space-y-1.5">
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2" style={{ background: colors.hybrid.bg, borderColor: colors.hybrid.border }} />
                        <span className="text-xs" style={{ color: colors.text }}>PID + Fuzzy (Hybrid)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2" style={{ background: colors.pid.bg, borderColor: colors.pid.border }} />
                        <span className="text-xs" style={{ color: colors.text }}>PID Only</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2" style={{ background: colors.pd.bg, borderColor: colors.pd.border }} />
                        <span className="text-xs" style={{ color: colors.text }}>PD Only</span>
                    </div>
                    <div className="flex items-center space-x-2 pt-1 mt-1" style={{ borderTop: `1px solid var(--color-border-subtle)` }}>
                        <div className="w-8 h-4 rounded-full border-2" style={{ background: colors.topic.bg, borderColor: colors.topic.border }} />
                        <span className="text-xs" style={{ color: colors.text }}>Topic</span>
                    </div>
                </div>
            </div>

            {/* Graph Canvas */}
            <svg
                className={`w-full h-full ${isDragging ? 'cursor-grabbing' : 'cursor-grab'}`}
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
            >
                <defs>
                    <marker
                        id="arrow"
                        viewBox="0 0 10 10"
                        refX="9"
                        refY="5"
                        markerWidth="6"
                        markerHeight="6"
                        orient="auto-start-reverse"
                    >
                        <path d="M 0 0 L 10 5 L 0 10 z" fill={colors.edge} />
                    </marker>
                    <marker
                        id="arrow-active"
                        viewBox="0 0 10 10"
                        refX="9"
                        refY="5"
                        markerWidth="6"
                        markerHeight="6"
                        orient="auto-start-reverse"
                    >
                        <path d="M 0 0 L 10 5 L 0 10 z" fill={colors.edgeActive} />
                    </marker>
                </defs>

                <g transform={`translate(${pan.x},${pan.y}) scale(${scale})`}>
                    {/* Edges */}
                    {layout.edges.map(edge => {
                        const isActive = selectedItem === edge.source || selectedItem === edge.target;
                        return (
                            <line
                                key={edge.id}
                                x1={edge.sourcePos.x}
                                y1={edge.sourcePos.y}
                                x2={edge.targetPos.x}
                                y2={edge.targetPos.y}
                                stroke={isActive ? colors.edgeActive : colors.edge}
                                strokeWidth={isActive ? 2 : 1}
                                markerEnd={isActive ? 'url(#arrow-active)' : 'url(#arrow)'}
                                className="transition-all duration-150"
                                style={{ filter: isActive ? `drop-shadow(0 0 4px ${colors.edgeActive})` : 'none' }}
                            />
                        );
                    })}

                    {/* Nodes and Topics */}
                    {layout.nodes.map(node => {
                        const isSelected = selectedItem === node.id;
                        const nodeColors = node.type === 'topic'
                            ? colors.topic
                            : getNodeColor(node.controllerType, node.role);

                        if (node.type === 'topic') {
                            return (
                                <g
                                    key={node.id}
                                    className="graph-item cursor-pointer transition-all duration-200"
                                    onClick={() => setSelectedItem(isSelected ? null : node.id)}
                                    style={{ opacity: isSelected || !selectedItem ? 1 : 0.4 }}
                                >
                                    <rect
                                        x={node.x}
                                        y={node.y}
                                        width={node.width}
                                        height={node.height}
                                        rx={node.height / 2}
                                        fill={isSelected ? colors.hybrid.bg : nodeColors.bg}
                                        stroke={isSelected ? colors.edgeActive : nodeColors.border}
                                        strokeWidth={isSelected ? 2 : 1.5}
                                    />
                                    <text
                                        x={node.x + node.width / 2}
                                        y={node.y + node.height / 2 + 4}
                                        textAnchor="middle"
                                        fill={nodeColors.text}
                                        fontSize="10"
                                        fontWeight="500"
                                        className="pointer-events-none select-none font-mono"
                                    >
                                        {node.label.length > 20 ? node.label.substring(0, 18) + '...' : node.label}
                                    </text>
                                </g>
                            );
                        }

                        return (
                            <g
                                key={node.id}
                                className="graph-item cursor-pointer transition-all duration-200"
                                onClick={() => setSelectedItem(isSelected ? null : node.id)}
                                style={{ opacity: isSelected || !selectedItem ? 1 : 0.4 }}
                            >
                                <rect
                                    x={node.x}
                                    y={node.y}
                                    width={node.width}
                                    height={node.height}
                                    rx={4}
                                    fill={nodeColors.bg}
                                    stroke={nodeColors.border}
                                    strokeWidth={isSelected ? 2 : 1}
                                    style={{ filter: isSelected ? `drop-shadow(0 0 8px ${nodeColors.border})` : 'none' }}
                                />
                                <text
                                    x={node.x + node.width / 2}
                                    y={node.y + node.height / 2 + 4}
                                    textAnchor="middle"
                                    fill={colors.text}
                                    fontSize="11"
                                    fontWeight="600"
                                    className="pointer-events-none select-none"
                                >
                                    {node.label.length > 16 ? node.label.substring(0, 14) + '...' : node.label}
                                </text>
                            </g>
                        );
                    })}
                </g>
            </svg>

            {/* Selected item info */}
            {selectedItem && (
                <div
                    className="absolute bottom-3 right-3 p-3 rounded-lg z-20 max-w-64 animate-fade-in border"
                    style={{ background: colors.bgPanel, borderColor: 'var(--color-border-subtle)' }}
                >
                    <div className="text-xs font-bold mb-1 uppercase tracking-wide" style={{ color: colors.hybrid.text }}>
                        {selectedItem.startsWith('topic:') ? 'Topic' : 'Node'}
                    </div>
                    <div className="text-xs break-all font-mono" style={{ color: colors.textMuted }}>
                        {selectedItem.replace('topic:', '')}
                    </div>
                </div>
            )}
        </div>
    );
};
