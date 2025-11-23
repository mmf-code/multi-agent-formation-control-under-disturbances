/**
 * Time Series Component
 * Real-time charts for metrics, errors, and performance indicators
 */
import React, { useMemo, useState, useCallback } from 'react';
// @ts-ignore - plotly.js-basic-dist-min has no types
import Plotly from 'plotly.js-basic-dist-min';
// @ts-ignore - react-plotly.js/factory has no types
import createPlotlyComponent from 'react-plotly.js/factory';
import { MetricsData, WindData, ControllerParams } from '../api/ws';

const Plot = createPlotlyComponent(Plotly);

interface TimeSeriesProps {
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  selectedAgent: string | null;
  chartType: 'error' | 'metrics' | 'wind' | 'overshoot';
  timeWindow?: number; // seconds, 0 = full run
  viewMode?: 'individual' | 'aggregated';
  controllerParams?: Record<string, ControllerParams>;
}

interface ZoomState {
  xaxis?: { range: [number, number]; autorange?: boolean };
  yaxis?: { range: [number, number]; autorange?: boolean };
  yaxis2?: { range: [number, number]; autorange?: boolean };
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

// Controller type colors
const CONTROLLER_COLORS: Record<string, string> = {
  pid: '#00f3ff', // Neon Blue
  hybrid: '#0aff68', // Signal Green (Emerald)
  fuzzy: '#bc13fe', // Neon Purple
  unknown: '#f59e0b', // Amber
};

export const TimeSeries: React.FC<TimeSeriesProps> = ({
  metricsHistory,
  windHistory,
  selectedAgent,
  chartType,
  timeWindow: _timeWindow = 60,
  viewMode = 'individual',
  controllerParams = {},
}) => {
  // TODO: Use _timeWindow to filter data by time range

  // State to persist zoom across data updates
  const [zoomState, setZoomState] = useState<ZoomState>({});

  // Callback to handle zoom/pan events
  const handleRelayout = useCallback((event: any) => {
    const newZoomState: ZoomState = {};

    // Capture x-axis changes
    if (event['xaxis.range[0]'] !== undefined && event['xaxis.range[1]'] !== undefined) {
      newZoomState.xaxis = {
        range: [event['xaxis.range[0]'], event['xaxis.range[1]']],
        autorange: false,
      };
    } else if (event['xaxis.range'] !== undefined) {
      newZoomState.xaxis = {
        range: event['xaxis.range'] as [number, number],
        autorange: false,
      };
    } else if (event['xaxis.autorange'] === true) {
      newZoomState.xaxis = { range: [0, 0], autorange: true };
    }

    // Capture y-axis changes
    if (event['yaxis.range[0]'] !== undefined && event['yaxis.range[1]'] !== undefined) {
      newZoomState.yaxis = {
        range: [event['yaxis.range[0]'], event['yaxis.range[1]']],
        autorange: false,
      };
    } else if (event['yaxis.range'] !== undefined) {
      newZoomState.yaxis = {
        range: event['yaxis.range'] as [number, number],
        autorange: false,
      };
    } else if (event['yaxis.autorange'] === true) {
      newZoomState.yaxis = { range: [0, 0], autorange: true };
    }

    // Capture y2-axis changes (for wind force chart)
    if (event['yaxis2.range[0]'] !== undefined && event['yaxis2.range[1]'] !== undefined) {
      newZoomState.yaxis2 = {
        range: [event['yaxis2.range[0]'], event['yaxis2.range[1]']],
        autorange: false,
      };
    } else if (event['yaxis2.range'] !== undefined) {
      newZoomState.yaxis2 = {
        range: event['yaxis2.range'] as [number, number],
        autorange: false,
      };
    } else if (event['yaxis2.autorange'] === true) {
      newZoomState.yaxis2 = { range: [0, 0], autorange: true };
    }

    // Only update state if there are actual changes
    if (Object.keys(newZoomState).length > 0) {
      setZoomState((prev) => ({ ...prev, ...newZoomState }));
    }
  }, []);

  // Helper function to aggregate metrics by controller type
  const aggregateByControllerType = useCallback((
    metricsHistory: Record<string, MetricsData[]>,
    controllerParams: Record<string, ControllerParams>
  ): Record<string, MetricsData[]> => {
    // Group agents by controller type
    const groupedAgents: Record<string, string[]> = {};
    Object.keys(metricsHistory).forEach((agentId) => {
      const controllerType = controllerParams[agentId]?.controller_type || 'unknown';
      if (!groupedAgents[controllerType]) {
        groupedAgents[controllerType] = [];
      }
      groupedAgents[controllerType].push(agentId);
    });

    // For each controller type, aggregate metrics at each timestamp
    const aggregated: Record<string, MetricsData[]> = {};

    Object.entries(groupedAgents).forEach(([controllerType, agentIds]) => {
      // Find all unique timestamps across all agents in this group
      const timestampMap = new Map<number, MetricsData[]>();

      agentIds.forEach((agentId) => {
        const history = metricsHistory[agentId] || [];
        history.forEach((metric) => {
          if (!timestampMap.has(metric.timestamp)) {
            timestampMap.set(metric.timestamp, []);
          }
          timestampMap.get(metric.timestamp)!.push(metric);
        });
      });

      // Sort timestamps and aggregate metrics
      const sortedTimestamps = Array.from(timestampMap.keys()).sort();
      aggregated[controllerType] = sortedTimestamps.map((timestamp) => {
        const metricsAtTime = timestampMap.get(timestamp)!;
        const count = metricsAtTime.length;

        // Calculate averages
        const avg = (field: keyof MetricsData) => {
          const sum = metricsAtTime.reduce((acc, m) => acc + (m[field] as number || 0), 0);
          return sum / count;
        };

        // Calculate max for overshoot
        const max = (field: keyof MetricsData) => {
          return Math.max(...metricsAtTime.map(m => (m[field] as number || 0)));
        };

        return {
          timestamp,
          agent_id: controllerType, // Use controller type as "agent_id"
          error_x: avg('error_x'),
          error_y: avg('error_y'),
          error_z: avg('error_z'),
          error_magnitude: avg('error_magnitude'),
          rmse_x: avg('rmse_x'),
          rmse_y: avg('rmse_y'),
          rmse_z: avg('rmse_z'),
          rmse_total: avg('rmse_total'),
          iae_x: avg('iae_x'),
          iae_y: avg('iae_y'),
          itae_x: avg('itae_x'),
          itae_y: avg('itae_y'),
          max_overshoot_x: max('max_overshoot_x'),
          max_overshoot_y: max('max_overshoot_y'),
          settling_time: avg('settling_time'),
          is_settled: metricsAtTime.every(m => m.is_settled),
        } as MetricsData;
      });
    });

    return aggregated;
  }, []);

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
          xaxis: {
            title: 'Time',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.xaxis?.autorange === false && zoomState.xaxis.range
              ? { range: zoomState.xaxis.range, autorange: false }
              : {}),
          },
          yaxis: {
            title: 'Velocity (m/s)',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.yaxis?.autorange === false && zoomState.yaxis.range
              ? { range: zoomState.yaxis.range, autorange: false }
              : {}),
          },
          yaxis2: hasForce
            ? {
              title: 'Force (N)',
              color: COLORS.secondary,
              overlaying: 'y' as any,
              side: 'right' as const,
              gridcolor: 'transparent',
              ...(zoomState.yaxis2?.autorange === false && zoomState.yaxis2.range
                ? { range: zoomState.yaxis2.range, autorange: false }
                : {}),
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

    // Determine data source based on view mode
    const dataSource = viewMode === 'aggregated' && !selectedAgent
      ? aggregateByControllerType(metricsHistory, controllerParams)
      : metricsHistory;

    const agents = selectedAgent
      ? [selectedAgent]
      : Object.keys(dataSource);

    if (chartType === 'error') {
      const data = agents.flatMap((agent, idx) => {
        const history = dataSource[agent] || [];
        // Use controller-specific colors for aggregated view, or cycle through colors for individual view
        const baseColor = viewMode === 'aggregated'
          ? CONTROLLER_COLORS[agent] || COLORS.primary
          : (idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success);

        const agentLabel = viewMode === 'aggregated'
          ? `${agent.toUpperCase()} Controller`
          : agent;

        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - Error X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.error_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - Error Y`,
            line: { width: 2, dash: 'dot' as const, color: baseColor }, // Same color, different style
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Position Error (m)',
          xaxis: {
            title: 'Time',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.xaxis?.autorange === false && zoomState.xaxis.range
              ? { range: zoomState.xaxis.range, autorange: false }
              : {}),
          },
          yaxis: {
            title: 'Error (m)',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.yaxis?.autorange === false && zoomState.yaxis.range
              ? { range: zoomState.yaxis.range, autorange: false }
              : {}),
          },
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
        const history = dataSource[agent] || [];
        const baseColor = viewMode === 'aggregated'
          ? CONTROLLER_COLORS[agent] || COLORS.primary
          : (idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success);
        const agentLabel = viewMode === 'aggregated'
          ? `${agent.toUpperCase()} Controller`
          : agent;

        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - IAE X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.iae_y),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - IAE Y`,
            line: { width: 2, dash: 'dot' as const, color: baseColor },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Integral Absolute Error (IAE)',
          xaxis: {
            title: 'Time',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.xaxis?.autorange === false && zoomState.xaxis.range
              ? { range: zoomState.xaxis.range, autorange: false }
              : {}),
          },
          yaxis: {
            title: 'IAE',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.yaxis?.autorange === false && zoomState.yaxis.range
              ? { range: zoomState.yaxis.range, autorange: false }
              : {}),
          },
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
        const history = dataSource[agent] || [];
        const baseColor = viewMode === 'aggregated'
          ? CONTROLLER_COLORS[agent] || COLORS.primary
          : (idx === 0 ? COLORS.primary : idx === 1 ? COLORS.secondary : COLORS.success);
        const agentLabel = viewMode === 'aggregated'
          ? `${agent.toUpperCase()} Controller`
          : agent;

        return [
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.max_overshoot_x),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - Overshoot X`,
            line: { width: 2, color: baseColor },
          },
          {
            x: history.map((m) => new Date(m.timestamp * 1000)),
            y: history.map((m) => m.rmse_total),
            type: 'scatter' as const,
            mode: 'lines' as const,
            name: `${agentLabel} - RMSE Total`,
            line: { width: 2, dash: 'dot' as const, color: baseColor },
          },
        ];
      });

      return {
        data,
        layout: {
          title: 'Overshoot & RMSE',
          xaxis: {
            title: 'Time',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.xaxis?.autorange === false && zoomState.xaxis.range
              ? { range: zoomState.xaxis.range, autorange: false }
              : {}),
          },
          yaxis: {
            title: 'Value (m)',
            color: COLORS.text,
            gridcolor: COLORS.grid,
            ...(zoomState.yaxis?.autorange === false && zoomState.yaxis.range
              ? { range: zoomState.yaxis.range, autorange: false }
              : {}),
          },
          paper_bgcolor: COLORS.bg,
          plot_bgcolor: COLORS.bg,
          font: { color: COLORS.text, family: 'Inter, sans-serif' },
          margin: { l: 60, r: 40, t: 40, b: 60 },
          uirevision: 'true', // Keep zoom state on update
        },
      };
    }

    return { data: [], layout: {} };
  }, [metricsHistory, windHistory, selectedAgent, chartType, zoomState, viewMode, controllerParams, aggregateByControllerType]);

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
        onRelayout={handleRelayout}
      />
    </div>
  );
};

