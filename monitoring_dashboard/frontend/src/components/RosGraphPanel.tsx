/**
 * ROS Graph Panel - rqt_graph style visualization
 * Shows nodes (rectangles), topics (rounded), and directed connections
 */
import React, { useState, useMemo, useRef, useCallback } from 'react';
import { RosGraphData, ControllerParams } from '../api/ws';
import { ZoomIn, ZoomOut, Filter, Eye, EyeOff, Maximize2 } from 'lucide-react';

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
    }, [graphData, showSystemTopics, filterAgentsOnly, searchQuery]);

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
                case 'hybrid': return { bg: '#059669', border: '#10B981', label: 'PID+Fuzzy' }; // Green - Hybrid
                case 'pid': return { bg: '#2563EB', border: '#3B82F6', label: 'PID' };          // Blue - PID
                case 'pd': return { bg: '#7C3AED', border: '#8B5CF6', label: 'PD' };            // Purple - PD
            }
        }
        // Role-based colors
        switch (role) {
            case 'controller': return { bg: '#2563EB', border: '#3B82F6', label: 'Controller' };
            case 'sensor': return { bg: '#059669', border: '#10B981', label: 'Sensor' };
            case 'metrics': return { bg: '#7C3AED', border: '#8B5CF6', label: 'Metrics' };
            case 'agent_node': return { bg: '#D97706', border: '#F59E0B', label: 'Agent' };
            default: return { bg: '#4B5563', border: '#6B7280', label: 'System' };
        }
    };

    if (!graphData) {
        return (
            <div className="w-full h-full flex items-center justify-center bg-gray-950 text-gray-500">
                <div className="text-center">
                    <div className="text-lg mb-2">Waiting for ROS2 graph data...</div>
                    <div className="text-sm">Start the simulation to see the graph</div>
                </div>
            </div>
        );
    }

    return (
        <div
            ref={containerRef}
            className="w-full h-full bg-gray-950 relative overflow-hidden"
            onWheel={handleWheel}
        >
            {/* Toolbar */}
            <div className="absolute top-3 left-3 right-3 flex items-center justify-between z-20">
                <div className="flex items-center space-x-3">
                    <h3 className="text-sm font-bold text-gray-200">ROS2 Graph</h3>
                    <span className="text-xs text-gray-500">
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
                        className="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs w-28 text-gray-300"
                    />

                    <button
                        onClick={() => setFilterAgentsOnly(!filterAgentsOnly)}
                        className={`px-2 py-1 rounded text-xs ${
                            filterAgentsOnly ? 'bg-blue-600 text-white' : 'bg-gray-800 text-gray-400'
                        }`}
                    >
                        <Filter className="w-3.5 h-3.5 inline mr-1" />
                        Agents
                    </button>

                    <button
                        onClick={() => setShowSystemTopics(!showSystemTopics)}
                        className={`px-2 py-1 rounded text-xs ${
                            showSystemTopics ? 'bg-blue-600 text-white' : 'bg-gray-800 text-gray-400'
                        }`}
                    >
                        {showSystemTopics ? <Eye className="w-3.5 h-3.5 inline mr-1" /> : <EyeOff className="w-3.5 h-3.5 inline mr-1" />}
                        System
                    </button>

                    <div className="flex items-center space-x-1 ml-2">
                        <button onClick={() => setScale(s => Math.min(s * 1.2, 2))} className="p-1.5 bg-gray-800 rounded hover:bg-gray-700 text-gray-300">
                            <ZoomIn className="w-4 h-4" />
                        </button>
                        <button onClick={() => setScale(s => Math.max(s / 1.2, 0.2))} className="p-1.5 bg-gray-800 rounded hover:bg-gray-700 text-gray-300">
                            <ZoomOut className="w-4 h-4" />
                        </button>
                        <button onClick={resetView} className="p-1.5 bg-gray-800 rounded hover:bg-gray-700 text-gray-300" title="Reset View">
                            <Maximize2 className="w-4 h-4" />
                        </button>
                    </div>
                </div>
            </div>

            {/* Legend */}
            <div className="absolute bottom-3 left-3 bg-gray-900/95 p-3 rounded-lg border border-gray-700 z-20">
                <div className="text-xs font-bold text-gray-400 mb-2">Controller Types</div>
                <div className="space-y-1.5">
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded" style={{ background: '#059669', border: '1px solid #10B981' }} />
                        <span className="text-xs text-gray-300">PID + Fuzzy (Hybrid)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded" style={{ background: '#2563EB', border: '1px solid #3B82F6' }} />
                        <span className="text-xs text-gray-300">PID Only</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded" style={{ background: '#7C3AED', border: '1px solid #8B5CF6' }} />
                        <span className="text-xs text-gray-300">PD Only</span>
                    </div>
                    <div className="flex items-center space-x-2 pt-1 border-t border-gray-700 mt-1">
                        <div className="w-8 h-4 rounded-full bg-emerald-900 border border-emerald-700" />
                        <span className="text-xs text-gray-300">Topic</span>
                    </div>
                    <div className="flex items-center space-x-2">
                        <div className="w-8 h-4 rounded" style={{ background: '#4B5563', border: '1px solid #6B7280' }} />
                        <span className="text-xs text-gray-300">System Node</span>
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
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#6B7280" />
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
                        <path d="M 0 0 L 10 5 L 0 10 z" fill="#3B82F6" />
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
                                stroke={isActive ? '#3B82F6' : '#4B5563'}
                                strokeWidth={isActive ? 2.5 : 1.5}
                                markerEnd={isActive ? 'url(#arrow-active)' : 'url(#arrow)'}
                                className="transition-all duration-150"
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
                                    className="graph-item cursor-pointer"
                                    onClick={() => setSelectedItem(isSelected ? null : node.id)}
                                >
                                    <rect
                                        x={node.x}
                                        y={node.y}
                                        width={node.width}
                                        height={node.height}
                                        rx={node.height / 2}
                                        fill={isSelected ? '#065F46' : '#064E3B'}
                                        stroke={isSelected ? '#10B981' : '#047857'}
                                        strokeWidth={isSelected ? 2 : 1}
                                    />
                                    <text
                                        x={node.x + node.width / 2}
                                        y={node.y + node.height / 2 + 4}
                                        textAnchor="middle"
                                        fill="#D1FAE5"
                                        fontSize="10"
                                        fontWeight="500"
                                        className="pointer-events-none select-none"
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
                                className="graph-item cursor-pointer"
                                onClick={() => setSelectedItem(isSelected ? null : node.id)}
                            >
                                <rect
                                    x={node.x}
                                    y={node.y}
                                    width={node.width}
                                    height={node.height}
                                    rx={4}
                                    fill={isSelected ? colors.border : colors.bg}
                                    stroke={colors.border}
                                    strokeWidth={isSelected ? 2 : 1}
                                />
                                <text
                                    x={node.x + node.width / 2}
                                    y={node.y + node.height / 2 + 4}
                                    textAnchor="middle"
                                    fill="#FFFFFF"
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
                <div className="absolute bottom-3 right-3 bg-gray-900/95 p-3 rounded-lg border border-gray-700 z-20 max-w-64">
                    <div className="text-xs font-bold text-gray-200 mb-1">
                        {selectedItem.startsWith('topic:') ? 'Topic' : 'Node'}
                    </div>
                    <div className="text-xs text-gray-400 break-all">
                        {selectedItem.replace('topic:', '')}
                    </div>
                </div>
            )}
        </div>
    );
};
