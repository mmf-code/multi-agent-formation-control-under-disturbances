import React, { useEffect, useState, useMemo } from 'react';
import Plot from 'react-plotly.js';
import { X, TrendingUp } from 'lucide-react';
import { MetricsData, OdometryData, WindData } from '../api/ws';

interface TopicPlotPanelProps {
    metrics: Record<string, MetricsData>;
    odom: Record<string, OdometryData>;
    wind: WindData | null;
}

interface TopicConfig {
    label: string;
    value: string;
    field: string;
}

interface PlotData {
    x: number[];
    y: number[];
}

export const TopicPlotPanel: React.FC<TopicPlotPanelProps> = ({ metrics, odom, wind }) => {
    const [selectedTopics, setSelectedTopics] = useState<TopicConfig[]>([]);
    const [history, setHistory] = useState<Record<string, PlotData>>({});

    // Available topics
    const availableTopics = useMemo(() => {
        const topics: TopicConfig[] = [];

        // Agent Metrics
        Object.keys(metrics).forEach(agentId => {
            topics.push({ label: `${agentId}/error_norm`, value: `${agentId}`, field: 'error_magnitude' });
            topics.push({ label: `${agentId}/error_x`, value: `${agentId}`, field: 'error_x' });
            topics.push({ label: `${agentId}/error_y`, value: `${agentId}`, field: 'error_y' });
        });

        // Odometry
        Object.keys(odom).forEach(agentId => {
            topics.push({ label: `${agentId}/pos_x`, value: `${agentId}`, field: 'pos_x' });
            topics.push({ label: `${agentId}/vel_x`, value: `${agentId}`, field: 'vel_x' });
        });

        // Wind
        if (wind) {
            topics.push({ label: 'wind/speed', value: 'wind', field: 'speed' });
            topics.push({ label: 'wind/force_x', value: 'wind', field: 'force_x' });
        }

        return topics;
    }, [metrics, odom, wind]);

    // Update history buffer
    useEffect(() => {
        const now = Date.now() / 1000;

        setHistory(prev => {
            const next = { ...prev };

            selectedTopics.forEach(config => {
                const key = `${config.value}/${config.field}`;
                if (!next[key]) next[key] = { x: [], y: [] };

                let value = 0;
                if (config.value === 'wind' && wind) {
                    if (config.field === 'speed') value = Math.sqrt(wind.velocity.x ** 2 + wind.velocity.y ** 2);
                    if (config.field === 'force_x') value = wind.force?.x || 0;
                } else if (metrics[config.value]) {
                    const m = metrics[config.value];
                    if (config.field === 'error_magnitude') value = m.error_magnitude;
                    if (config.field === 'error_x') value = m.error_x;
                    if (config.field === 'error_y') value = m.error_y;
                } else if (odom[config.value]) {
                    const o = odom[config.value];
                    if (config.field === 'pos_x') value = o.position.x;
                    if (config.field === 'vel_x') value = o.velocity.x;
                }

                next[key].x.push(now);
                next[key].y.push(value);

                // Keep last 60 seconds
                const cutoff = now - 60;
                const startIdx = next[key].x.findIndex(t => t >= cutoff);
                if (startIdx > 0) {
                    next[key].x = next[key].x.slice(startIdx);
                    next[key].y = next[key].y.slice(startIdx);
                }
            });

            return next;
        });
    }, [metrics, odom, wind, selectedTopics]);

    const addTopic = (topic: TopicConfig) => {
        if (!selectedTopics.some(t => t.label === topic.label)) {
            setSelectedTopics([...selectedTopics, topic]);
        }
    };

    const removeTopic = (label: string) => {
        setSelectedTopics(selectedTopics.filter(t => t.label !== label));
        setHistory(prev => {
            const next = { ...prev };
            delete next[label];
            return next;
        });
    };

    // Prepare plot data
    const plotData = selectedTopics.map(config => {
        const key = `${config.value}/${config.field}`;
        const data = history[key] || { x: [], y: [] };
        return {
            x: data.x,
            y: data.y,
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: config.label,
            line: { width: 2 }
        };
    });

    return (
        <div className="h-full flex flex-col bg-gray-900 text-white">
            {/* Header */}
            <div className="p-4 border-b border-gray-800 flex items-center justify-between sticky top-0 bg-gray-900 z-10">
                <h2 className="text-lg font-semibold flex items-center">
                    <TrendingUp className="w-5 h-5 mr-2 text-blue-400" />
                    Topic Plotter
                </h2>
                <select
                    className="bg-gray-800 border border-gray-700 rounded px-3 py-1 text-sm"
                    onChange={(e) => {
                        const topic = availableTopics.find(t => t.label === e.target.value);
                        if (topic) addTopic(topic);
                        e.target.value = '';
                    }}
                    value=""
                >
                    <option value="">Add Topic...</option>
                    {availableTopics.map(topic => (
                        <option key={topic.label} value={topic.label}>{topic.label}</option>
                    ))}
                </select>
            </div>

            {/* Plot Area */}
            <div className="flex-1 p-4">
                {selectedTopics.length === 0 ? (
                    <div className="h-full flex items-center justify-center text-gray-500">
                        Select topics to plot
                    </div>
                ) : (
                    <Plot
                        data={plotData}
                        layout={{
                            autosize: true,
                            paper_bgcolor: '#111827',
                            plot_bgcolor: '#1f2937',
                            font: { color: '#9ca3af' },
                            xaxis: { title: 'Time (s)', gridcolor: '#374151' },
                            yaxis: { title: 'Value', gridcolor: '#374151' },
                            margin: { l: 50, r: 20, t: 20, b: 40 },
                            showlegend: true,
                            legend: { x: 1, xanchor: 'right', y: 1 }
                        }}
                        config={{ responsive: true }}
                        style={{ width: '100%', height: '100%' }}
                    />
                )}
            </div>

            {/* Selected Topics */}
            {selectedTopics.length > 0 && (
                <div className="p-4 border-t border-gray-800 flex flex-wrap gap-2">
                    {selectedTopics.map(topic => (
                        <div key={topic.label} className="flex items-center bg-gray-800 rounded px-3 py-1 text-sm">
                            <span>{topic.label}</span>
                            <button
                                onClick={() => removeTopic(topic.label)}
                                className="ml-2 text-gray-400 hover:text-red-400"
                            >
                                <X className="w-3 h-3" />
                            </button>
                        </div>
                    ))}
                </div>
            )}
        </div>
    );
};
