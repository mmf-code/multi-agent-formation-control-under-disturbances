/**
 * Time Series Component
 * Real-time charts for metrics, errors, and performance indicators
 */
import React, { useMemo } from 'react';
// @ts-ignore - plotly.js-basic-dist-min has no types
import Plotly from 'plotly.js-basic-dist-min';
// @ts-ignore - react-plotly.js/factory has no types
import createPlotlyComponent from 'react-plotly.js/factory';
import { MetricsData, WindData } from '../api/ws';

const Plot = createPlotlyComponent(Plotly);

interface TimeSeriesProps {
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  selectedAgent: string | null;
  chartType: 'error' | 'metrics' | 'wind' | 'overshoot';
  timeWindow?: number; // seconds, 0 = full run
}

// Sci-Fi Palette Colors
const COLORS = {
  primary: '#00f3ff', // Neon Blue
  secondary: '#bc13fe', // Neon Purple
  success: '#0aff68', // Signal Green
  warning: '#f59e0b', // Amber
  danger: '#ff003c', // Alert Red
  text: '#e2e8f0', // Slate 200
  grid: '#1f2937', // Gray 800
  bg: 'transparent',
};

export const TimeSeries: React.FC<TimeSeriesProps> = ({
  metricsHistory,
  windHistory,
  selectedAgent,
  chartType,
  timeWindow: _timeWindow = 60,
}) => {
  // TODO: Use _timeWindow to filter data by time range
  const plotData = useMemo(() => {
    if (chartType === 'wind') {
      const hasForce = windHistory.some((w) => w.force);
      const forceMagnitude = hasForce
        ? windHistory.map((w) =>
          w.force
            ? Math.sqrt(
              (w.force.x || 0) ** 2 +
              (w.force.y || 0) ** 2 +
              (w.force.z || 0) ** 2
            )
            : 0
        )
        : [];

      return {
        data: [
          {
            x: windHistory.map((w) => new Date(w.timestamp * 1000)),
            y: windHistory.map((w) => w.velocity.x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: 'Wind X',
            line: { color: COLORS.primary, width: 2 },
          },
          {
            x: windHistory.map((w) => new Date(w.timestamp * 1000)),
            y: windHistory.map((w) => w.velocity.y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: 'Wind Y',
            line: { color: COLORS.success, width: 2 },
          },
          ...(hasForce
            ? [
              {
                x: windHistory.map((w) => new Date(w.timestamp * 1000)),
                y: windHistory.map((w) => (w.force ? w.force.x : 0)),
                type: 'scatter' as const,
                mode: 'lines' as const,
                name: 'Force X (N)',
                line: { color: COLORS.warning, width: 1.5, dash: 'dot' as const },
              },
              {
                x: windHistory.map((w) => new Date(w.timestamp * 1000)),
                y: windHistory.map((w) => (w.force ? w.force.y : 0)),
                type: 'scatter' as const,
                mode: 'lines' as const,
                name: 'Force Y (N)',
                line: { color: COLORS.danger, width: 1.5, dash: 'dot' as const },
              },
              {
                x: windHistory.map((w) => new Date(w.timestamp * 1000)),
                y: forceMagnitude,
                type: 'scatter' as const,
                mode: 'lines' as const,
                name: 'Force |F| (N)',
                line: { color: COLORS.secondary, width: 2 },
                yaxis: 'y2' as const,
              },
            ]
            : []),
        ],
        layout: {
          title: hasForce ? 'Wind Velocity (m/s) + Force (N)' : 'Wind Velocity (m/s)',
          xaxis: { title: 'Time', color: COLORS.text, gridcolor: COLORS.grid },
          yaxis: { title: 'Velocity (m/s)', color: COLORS.text, gridcolor: COLORS.grid },
          yaxis2: hasForce
            ? {
              title: 'Force (N)',
              color: COLORS.secondary,
              overlaying: 'y' as any,
              side: 'right' as const,
              gridcolor: 'transparent',
            }
            : undefined,
          paper_bgcolor: COLORS.bg,
          plot_bgcolor: COLORS.bg,
          font: { color: COLORS.text, family: 'Inter, sans-serif' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          uirevision: 'true', // Keep zoom state on update
        },
      };
    }

    const agents = selectedAgent
      ? [selectedAgent]
      : Object.keys(metricsHistory);

    if (chartType === 'error') {
      const data = agents.flatMap((agent, idx) => {
        const history = metricsHistory[agent] || [];
        // Cycle through colors for different agents if needed, or stick to theme
        const baseColor = idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success;

        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Error X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Error Y`,
            line: { width: 2, dash: 'dot' as const, color: baseColor }, // Same color, different style
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Position Error (m)',
          xaxis: { title: 'Time', color: COLORS.text, gridcolor: COLORS.grid },
          yaxis: { title: 'Error (m)', color: COLORS.text, gridcolor: COLORS.grid },
          paper_bgcolor: COLORS.bg,
          plot_bgcolor: COLORS.bg,
          font: { color: COLORS.text, family: 'Inter, sans-serif' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          hovermode: 'closest' as const,
          uirevision: 'true', // Keep zoom state on update
        },
      };
    }

    if (chartType === 'metrics') {
      const data = agents.flatMap((agent, idx) => {
        const history = metricsHistory[agent] || [];
        const baseColor = idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success;
        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - IAE X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - IAE Y`,
            line: { width: 2, dash: 'dot' as const, color: baseColor },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Integral Absolute Error (IAE)',
          xaxis: { title: 'Time', color: COLORS.text, gridcolor: COLORS.grid },
          yaxis: { title: 'IAE', color: COLORS.text, gridcolor: COLORS.grid },
          paper_bgcolor: COLORS.bg,
          plot_bgcolor: COLORS.bg,
          font: { color: COLORS.text, family: 'Inter, sans-serif' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          uirevision: 'true', // Keep zoom state on update
        },
      };
    }

    if (chartType === 'overshoot') {
      const data = agents.flatMap((agent, idx) => {
        const history = metricsHistory[agent] || [];
        const baseColor = idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success;
        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.max_overshoot_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - Overshoot X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.rmse_total),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agent} - RMSE Total`,
            line: { width: 2, dash: 'dot' as const, color: baseColor },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Overshoot & RMSE',
          xaxis: { title: 'Time', color: COLORS.text, gridcolor: COLORS.grid },
          yaxis: { title: 'Value (m)', color: COLORS.text, gridcolor: COLORS.grid },
          paper_bgcolor: COLORS.bg,
          plot_bgcolor: COLORS.bg,
          font: { color: COLORS.text, family: 'Inter, sans-serif' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          uirevision: 'true', // Keep zoom state on update
        },
      };
    }

    return { data: [], layout: {} };
  }, [metricsHistory, windHistory, selectedAgent, chartType]);

  return (
    <div className="glass-panel rounded-lg p-4 transition-all duration-300 hover:shadow-[0_0_20px_rgba(0,243,255,0.1)]">
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
          modeBarButtonsToRemove: ['lasso2d', 'select2d'],
        }}
        style={{ width: '100%' }}
      />
    </div>
  );
};

