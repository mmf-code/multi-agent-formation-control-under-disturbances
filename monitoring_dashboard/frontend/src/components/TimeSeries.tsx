/**
 * Time Series Component
 * Real-time charts for metrics, errors, and performance indicators
 */
import React, { useMemo } from 'react';
import Plot from 'react-plotly.js';
import { MetricsData, WindData } from '../api/ws';

interface TimeSeriesProps {
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  selectedAgent: string | null;
  chartType: 'error' | 'metrics' | 'wind' | 'overshoot';
}

export const TimeSeries: React.FC<TimeSeriesProps> = ({
  metricsHistory,
  windHistory,
  selectedAgent,
  chartType,
}) => {
  const plotData = useMemo(() => {
    if (chartType === 'wind') {
      return {
        data: [
          {
            x: windHistory.map((w) => new Date(w.timestamp * 1000)),
            y: windHistory.map((w) => w.velocity.x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: 'Wind X',
            line: { color: '#3b82f6', width: 2 },
          },
          {
            x: windHistory.map((w) => new Date(w.timestamp * 1000)),
            y: windHistory.map((w) => w.velocity.y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: 'Wind Y',
            line: { color: '#10b981', width: 2 },
          },
        ],
        layout: {
          title: 'Wind Velocity (m/s)',
          xaxis: { title: 'Time', color: '#9ca3af' },
          yaxis: { title: 'Velocity (m/s)', color: '#9ca3af' },
          paper_bgcolor: '#1f2937',
          plot_bgcolor: '#111827',
          font: { color: '#e5e7eb' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
        },
      };
    }

    const agents = selectedAgent
      ? [selectedAgent]
      : Object.keys(metricsHistory);

    if (chartType === 'error') {
      const data = agents.flatMap((agent) => {
        const history = metricsHistory[agent] || [];
        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Error X`,
            line: { width: 2 },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Error Y`,
            line: { width: 2, dash: 'dot' as const },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Position Error (m)',
          xaxis: { title: 'Time', color: '#9ca3af' },
          yaxis: { title: 'Error (m)', color: '#9ca3af' },
          paper_bgcolor: '#1f2937',
          plot_bgcolor: '#111827',
          font: { color: '#e5e7eb' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          hovermode: 'closest' as const,
        },
      };
    }

    if (chartType === 'metrics') {
      const data = agents.flatMap((agent) => {
        const history = metricsHistory[agent] || [];
        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - IAE X`,
            line: { width: 2 },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - IAE Y`,
            line: { width: 2, dash: 'dot' as const },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Integral Absolute Error (IAE)',
          xaxis: { title: 'Time', color: '#9ca3af' },
          yaxis: { title: 'IAE', color: '#9ca3af' },
          paper_bgcolor: '#1f2937',
          plot_bgcolor: '#111827',
          font: { color: '#e5e7eb' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
        },
      };
    }

    if (chartType === 'overshoot') {
      const data = agents.flatMap((agent) => {
        const history = metricsHistory[agent] || [];
        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.max_overshoot_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Overshoot X`,
            line: { width: 2 },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.rmse_total),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - RMSE Total`,
            line: { width: 2, dash: 'dot' as const },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Overshoot & RMSE',
          xaxis: { title: 'Time', color: '#9ca3af' },
          yaxis: { title: 'Value (m)', color: '#9ca3af' },
          paper_bgcolor: '#1f2937',
          plot_bgcolor: '#111827',
          font: { color: '#e5e7eb' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
        },
      };
    }

    return { data: [], layout: {} };
  }, [metricsHistory, windHistory, selectedAgent, chartType]);

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
      <Plot
        data={plotData.data as any}
        layout={{
          ...plotData.layout,
          autosize: true,
          height: 400,
        }}
        config={{
          responsive: true,
          displayModeBar: true,
          displaylogo: false,
        }}
        style={{ width: '100%' }}
      />
    </div>
  );
};
