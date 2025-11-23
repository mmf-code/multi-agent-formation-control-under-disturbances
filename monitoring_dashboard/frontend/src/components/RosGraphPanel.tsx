/**
 * ROS Graph Panel - rqt_graph style visualization
 * Shows nodes (rectangles), topics (rounded), and directed connections
 */
import React, { useState, useMemo, useRef, useCallback } from 'react';
import { RosGraphData, ControllerParams } from '../api/ws';
import { ZoomIn, ZoomOut, Filter, Eye, EyeOff, Maximize2, Activity, Download } from 'lucide-react';
import html2canvas from 'html2canvas';

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
    controllerType?: string; // 'pid' | 'hybrid' | 'pd'
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
    '/_action/',      // Action server internal topics
    '/bond',          // Lifecycle node bond topics
    '/transition_event', // Lifecycle transition events
    '/events/read',   // Parameter events
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

export const RosGraphPanel: React.FC<RosGraphPanelProps> = ({ graphData, controllerParams }) => {
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
            // Add controller type to name
            const ctrlLabel = controllerType === 'hybrid' ? 'PID+Fuzzy' : controllerType?.toUpperCase() || '';
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
    const [strictMode, setStrictMode] = useState(true); // Default to strict mode to reduce noise
    const [searchQuery, setSearchQuery] = useState('');

    // Build rqt_graph style layout: Nodes -> Topics -> Nodes
    const layout = useMemo(() => {
        if (!graphData || graphData.nodes.length === 0) {
            return { nodes: [], edges: [], topics: [] };
        }

        // Collect all unique topics from edges
        const topicSet = new Map<string, { name: string; msgType: string; publishers: string[]; subscribers: string[] }>();

        graphData.edges.forEach(edge => {
            const topicName = edge.topicName || 'unknown';

            // Filter out system topics
            if (!showSystemTopics) {
                if (HIDDEN_PATTERNS.some(p => topicName.includes(p))) return;
            }

            // Strict mode filtering
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

        // Filter nodes
        let visibleNodes = graphData.nodes.filter(n => {
            if (filterAgentsOnly && !n.name.includes('agent')) return false;
            if (searchQuery && !n.name.toLowerCase().includes(searchQuery.toLowerCase())) return false;
            return true;
        });

        // Get nodes that are connected to visible topics
        const connectedNodeIds = new Set<string>();
        topicSet.forEach(topic => {
            topic.publishers.forEach(id => connectedNodeIds.add(id));
            topic.subscribers.forEach(id => connectedNodeIds.add(id));
        });

        // Only show nodes that have connections (unless searching)
        if (!searchQuery) {
            visibleNodes = visibleNodes.filter(n => connectedNodeIds.has(n.id));
        }

        // Layout calculation - 3 columns: Publishers | Topics | Subscribers
        const NODE_WIDTH = 140;
        const NODE_HEIGHT = 36;
        const TOPIC_WIDTH = 160;
        const TOPIC_HEIGHT = 28;
        const COL_SPACING = 280;
        const ROW_SPACING = 50;

        // Identify pure publishers, pure subscribers, and nodes that do both
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

        // Separate into columns
        const publishers = visibleNodes.filter(n => nodeRoles.get(n.id) === 'publisher' || nodeRoles.get(n.id) === 'both');
        const subscribers = visibleNodes.filter(n => nodeRoles.get(n.id) === 'subscriber');
        const topics = Array.from(topicSet.values());

        // Position nodes
        const layoutNodes: LayoutNode[] = [];

        // Left column - Publishers
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

        // Middle column - Topics
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

        // Right column - Subscribers (that aren't also publishers)
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

        // Build edges: Node -> Topic, Topic -> Node
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

            // Publisher -> Topic edges
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

            // Topic -> Subscriber edges
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
    }, [graphData, showSystemTopics, filterAgentsOnly, searchQuery, strictMode]);

    // Export to PNG function
    const exportToPNG = () => {
        if (containerRef.current) {
            html2canvas(containerRef.current, {
                scale: 2, // Higher resolution
                backgroundColor: '#ffffff', // Ensure white background
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

    // Node color by controller type (prioritize) or role
    const getNodeColor = (controllerType?: string, role?: string) => {
        // Controller type colors (for agent nodes)
        if (controllerType) {
            switch (controllerType) {
                case 'hybrid': return { bg: 'rgba(10, 255, 104, 0.2)', border: '#0aff68', label: 'PID+Fuzzy' }; // Neon Green
                case 'pid': return { bg: 'rgba(0, 243, 255, 0.2)', border: '#00f3ff', label: 'PID' };          // Neon Blue
                case 'pd': return { bg: 'rgba(188, 19, 254, 0.2)', border: '#bc13fe', label: 'PD' };            // Neon Purple
            }
        }
        // Role-based colors
        switch (role) {
            case 'controller': return { bg: 'rgba(0, 243, 255, 0.1)', border: '#00f3ff', label: 'Controller' };
            case 'sensor': return { bg: 'rgba(10, 255, 104, 0.1)', border: '#0aff68', label: 'Sensor' };
            case 'metrics': return { bg: 'rgba(188, 19, 254, 0.1)', border: '#bc13fe', label: 'Metrics' };
            case 'agent_node': return { bg: 'rgba(245, 158, 11, 0.1)', border: '#f59e0b', label: 'Agent' };
            default: return { bg: 'rgba(31, 41, 55, 0.5)', border: '#4b5563', label: 'System' };
        }
    };

    if (!graphData) {
        return (
            <div className="w-full h-full flex items-center justify-center bg-space-950 text-gray-500">
                <div className="text-center animate-pulse">
                    <div className="text-lg mb-2 font-mono">Waiting for ROS2 graph data...</div>
                    <div className="text-sm">Start the simulation to see the graph</div>
                </div>
            </div>
        );
    }

    return (
        <div
            ref={containerRef}
            className="w-full h-full bg-white relative overflow-hidden"
            onWheel={handleWheel}
        >
            {/* Background Grid */}
            <div className="absolute inset-0 bg-grid-pattern opacity-5 pointer-events-none" />

            {/* Toolbar */}
            <div className="absolute top-3 left-3 right-3 flex items-center justify-between z-20">
                <div className="flex items-center space-x-3 bg-white/90 border border-gray-300 px-3 py-1.5 rounded-lg shadow-sm">
                    <h3 className="text-sm font-bold text-blue-600 uppercase tracking-wider">ROS2 Graph</h3>
                    <span className="text-xs text-gray-600 border-l border-gray-300 pl-3">
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
                        className="bg-white border border-gray-300 rounded px-2 py-1 text-xs w-28 text-gray-700 focus:border-blue-500 focus:outline-none transition-colors"
                    />

                    <button
                        onClick={() => setFilterAgentsOnly(!filterAgentsOnly)}
                        className={`px-2 py-1 rounded text-xs border transition-all duration-300 ${filterAgentsOnly
                            ? 'bg-blue-100 border-blue-500 text-blue-700'
                            : 'bg-white border-gray-300 text-gray-600 hover:border-gray-400'
                            }`}
                    >
                        <Filter className="w-3.5 h-3.5 inline mr-1" />
                        Agents
                    </button>

                    <button
                        onClick={() => setShowSystemTopics(!showSystemTopics)}
                        className={`px-2 py-1 rounded text-xs border transition-all duration-300 ${showSystemTopics
                            ? 'bg-purple-100 border-purple-500 text-purple-700'
                            : 'bg-white border-gray-300 text-gray-600 hover:border-gray-400'
                            }`}
                    >
                        {showSystemTopics ? <Eye className="w-3.5 h-3.5 inline mr-1" /> : <EyeOff className="w-3.5 h-3.5 inline mr-1" />}
                        System
                    </button>

                    <button
                        onClick={() => setStrictMode(!strictMode)}
                        className={`px-2 py-1 rounded text-xs border transition-all duration-300 ${strictMode
                            ? 'bg-green-100 border-green-500 text-green-700'
                            : 'bg-white border-gray-300 text-gray-600 hover:border-gray-400'
                            }`}
                        title="Show only core formation topics"
                    >
                        <Activity className="w-3.5 h-3.5 inline mr-1" />
                        Focus
                    </button>

                    <div className="flex items-center space-x-1 ml-2 glass-panel p-1 rounded-lg">
                        <button onClick={() => setScale(s => Math.min(s * 1.2, 2))} className="p-1.5 hover:bg-gray-100 rounded text-gray-600 transition-colors">
                            <ZoomIn className="w-4 h-4" />
                        </button>
                        <button onClick={() => setScale(s => Math.max(s / 1.2, 0.2))} className="p-1.5 hover:bg-gray-100 rounded text-gray-600 transition-colors">
                            <ZoomOut className="w-4 h-4" />
                        </button>
                        <button onClick={resetView} className="p-1.5 hover:bg-gray-100 rounded text-gray-600 transition-colors" title="Reset View">
                            <Maximize2 className="w-4 h-4" />
                        </button>
                        <button onClick={exportToPNG} className="p-1.5 hover:bg-blue-100 rounded text-blue-600 transition-colors" title="Export to PNG">
                            <Download className="w-4 h-4" />
                        </button>
                    </div>
                </div>
            </div>

            {/* Legend */}
            <div className="absolute bottom-3 left-3 bg-white/90 border border-gray-300 p-3 rounded-lg z-20 shadow-sm">
                <div className="text-xs font-bold text-gray-700 mb-2 uppercase tracking-wide">Controller Types</div>
                <div className="space-y-1.5">
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2 border-emerald-600 bg-emerald-100" />
                        <span className="text-xs text-gray-700">PID + Fuzzy (Hybrid)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2 border-blue-600 bg-blue-100" />
                        <span className="text-xs text-gray-700">PID Only</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded border-2 border-purple-600 bg-purple-100" />
                        <span className="text-xs text-gray-700">PD Only</span>
                    </div>
                    <div className="flex items-center space-x-2 pt-1 border-t border-gray-300 mt-1">
                        <div className="w-8 h-4 rounded-full bg-gray-100 border-2 border-gray-400" />
                        <span className="text-xs text-gray-700">Topic</span>
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
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#6b7280" />
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
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#2563eb" />
                    </marker>
                    <filter id="glow-blue" x="-20%" y="-20%" width="140%" height="140%">
                        <feGaussianBlur stdDeviation="2" result="blur" />
                        <feComposite in="SourceGraphic" in2="blur" operator="over" />
                    </filter>
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
                                stroke={isActive ? '#2563eb' : '#9ca3af'}
                                strokeWidth={isActive ? 2 : 1}
                                markerEnd={isActive ? 'url(#arrow-active)' : 'url(#arrow)'}
                                className="transition-all duration-150"
                                style={{ filter: isActive ? 'drop-shadow(0 0 2px #2563eb)' : 'none' }}
                            />
                        );
                    })}

                    {/* Nodes and Topics */}
                    {layout.nodes.map(node => {
                        const isSelected = selectedItem === node.id;
                        const colors = getNodeColor(node.controllerType, node.role);

                        if (node.type === 'topic') {
                            // Topic - rounded rectangle (pill shape)
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
                                        fill={isSelected ? '#e0f2fe' : '#f3f4f6'}
                                        stroke={isSelected ? '#2563eb' : '#9ca3af'}
                                        strokeWidth={isSelected ? 2 : 1.5}
                                    />
                                    <text
                                        x={node.x + node.width / 2}
                                        y={node.y + node.height / 2 + 4}
                                        textAnchor="middle"
                                        fill="#374151"
                                        fontSize="10"
                                        fontWeight="500"
                                        className="pointer-events-none select-none font-mono"
                                    >
                                        {node.label.length > 20 ? node.label.substring(0, 18) + '...' : node.label}
                                    </text>
                                </g>
                            );
                        }

                        // Node - rectangle
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
                                    fill={colors.bg}
                                    stroke={colors.border}
                                    strokeWidth={isSelected ? 2 : 1}
                                    style={{ filter: isSelected ? `drop-shadow(0 0 8px ${colors.border})` : 'none' }}
                                />
                                <text
                                    x={node.x + node.width / 2}
                                    y={node.y + node.height / 2 + 4}
                                    textAnchor="middle"
                                    fill="#1f2937"
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
                <div className="absolute bottom-3 right-3 glass-panel p-3 rounded-lg z-20 max-w-64 animate-slide-up">
                    <div className="text-xs font-bold text-neon-blue mb-1 uppercase tracking-wide">
                        {selectedItem.startsWith('topic:') ? 'Topic' : 'Node'}
                    </div>
                    <div className="text-xs text-gray-300 break-all font-mono">
                        {selectedItem.replace('topic:', '')}
                    </div>
                </div>
            )}
        </div>
    );
};
