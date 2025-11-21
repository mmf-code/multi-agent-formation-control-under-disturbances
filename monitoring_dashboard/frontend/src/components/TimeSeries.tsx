/**
 * Time Series Component
 * Real-time charts for metrics, errors, and performance indicators using Recharts
 */
import React, { useMemo } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from 'recharts';
import { MetricsData, WindData } from '../api/ws';

interface TimeSeriesProps {
  metricsHistory: Record<string, MetricsData[]>;
  windHistory: WindData[];
  selectedAgent: string | null;
  chartType: 'error' | 'metrics' | 'wind' | 'overshoot';
  timeWindow?: number; // seconds, 0 = full run
}

// Color palette for charts
const COLORS = [
  '#3b82f6', // blue
  '#10b981', // green
  '#f59e0b', // amber
  '#ef4444', // red
  '#8b5cf6', // purple
  '#ec4899', // pink
  '#14b8a6', // teal
  '#f97316', // orange
];

export const TimeSeries: React.FC<TimeSeriesProps> = ({
  metricsHistory,
  windHistory,
  selectedAgent,
  chartType,
  timeWindow: _timeWindow = 60,
}) => {
  // Custom tooltip with dark theme
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload.length) {
      return (
        <div className="bg-gray-800 border border-gray-700 rounded p-2 text-xs">
          <p className="text-gray-400 mb-1">{new Date(label).toLocaleTimeString()}</p>
          {payload.map((entry: any, index: number) => (
            <p key={index} style={{ color: entry.color }}>
              {entry.name}: {entry.value.toFixed(3)}
            </p>
          ))}
        </div>
      );
    }
    return null;
  };

  const chartData = useMemo(() => {
    if (chartType === 'wind') {
      return windHistory.map((w) => ({
        timestamp: w.timestamp * 1000,
        windX: w.velocity.x,
        windY: w.velocity.y,
        forceX: w.force?.x || 0,
        forceY: w.force?.y || 0,
        forceMag: w.force
          ? Math.sqrt(
              (w.force.x || 0) ** 2 + (w.force.y || 0) ** 2 + (w.force.z || 0) ** 2
            )
          : 0,
      }));
    }

    const agents = selectedAgent ? [selectedAgent] : Object.keys(metricsHistory);

    if (chartType === 'error') {
      // Combine all agent error data
      const dataMap = new Map<number, any>();

      agents.forEach((agent) => {
        const history = metricsHistory[agent] || [];
        history.forEach((m) => {
          const ts = m.timestamp * 1000;
          if (!dataMap.has(ts)) {
            dataMap.set(ts, { timestamp: ts });
          }
          const entry = dataMap.get(ts)!;
          entry[`${agent}_errorX`] = m.error_x;
          entry[`${agent}_errorY`] = m.error_y;
        });
      });

      return Array.from(dataMap.values()).sort((a, b) => a.timestamp - b.timestamp);
    }

    if (chartType === 'metrics') {
      const dataMap = new Map<number, any>();

      agents.forEach((agent) => {
        const history = metricsHistory[agent] || [];
        history.forEach((m) => {
          const ts = m.timestamp * 1000;
          if (!dataMap.has(ts)) {
            dataMap.set(ts, { timestamp: ts });
          }
          const entry = dataMap.get(ts)!;
          entry[`${agent}_iaeX`] = m.iae_x;
          entry[`${agent}_iaeY`] = m.iae_y;
        });
      });

      return Array.from(dataMap.values()).sort((a, b) => a.timestamp - b.timestamp);
    }

    if (chartType === 'overshoot') {
      const dataMap = new Map<number, any>();

      agents.forEach((agent) => {
        const history = metricsHistory[agent] || [];
        history.forEach((m) => {
          const ts = m.timestamp * 1000;
          if (!dataMap.has(ts)) {
            dataMap.set(ts, { timestamp: ts });
          }
          const entry = dataMap.get(ts)!;
          entry[`${agent}_overshootX`] = m.max_overshoot_x;
          entry[`${agent}_rmse`] = m.rmse_total;
        });
      });

      return Array.from(dataMap.values()).sort((a, b) => a.timestamp - b.timestamp);
    }

    return [];
  }, [metricsHistory, windHistory, selectedAgent, chartType]);

  const renderLines = useMemo(() => {
    if (chartType === 'wind') {
      const hasForce = windHistory.some((w) => w.force);
      return (
        <>
          <Line
            type="monotone"
            dataKey="windX"
            stroke="#3b82f6"
            strokeWidth={2}
            name="Wind X (m/s)"
            dot={false}
            isAnimationActive={false}
          />
          <Line
            type="monotone"
            dataKey="windY"
            stroke="#10b981"
            strokeWidth={2}
            name="Wind Y (m/s)"
            dot={false}
            isAnimationActive={false}
          />
          {hasForce && (
            <>
              <Line
                type="monotone"
                dataKey="forceX"
                stroke="#f59e0b"
                strokeWidth={1.5}
                strokeDasharray="5 5"
                name="Force X (N)"
                dot={false}
                isAnimationActive={false}
              />
              <Line
                type="monotone"
                dataKey="forceY"
                stroke="#ef4444"
                strokeWidth={1.5}
                strokeDasharray="5 5"
                name="Force Y (N)"
                dot={false}
                isAnimationActive={false}
              />
              <Line
                type="monotone"
                dataKey="forceMag"
                stroke="#ec4899"
                strokeWidth={2}
                name="Force |F| (N)"
                dot={false}
                isAnimationActive={false}
              />
            </>
          )}
        </>
      );
    }

    const agents = selectedAgent ? [selectedAgent] : Object.keys(metricsHistory);

    if (chartType === 'error') {
      return agents.flatMap((agent, idx) => [
        <Line
          key={`${agent}_errorX`}
          type="monotone"
          dataKey={`${agent}_errorX`}
          stroke={COLORS[idx % COLORS.length]}
          strokeWidth={2}
          name={`${agent} - Error X`}
          dot={false}
          isAnimationActive={false}
        />,
        <Line
          key={`${agent}_errorY`}
          type="monotone"
          dataKey={`${agent}_errorY`}
          stroke={COLORS[(idx + 1) % COLORS.length]}
          strokeWidth={2}
          strokeDasharray="5 5"
          name={`${agent} - Error Y`}
          dot={false}
          isAnimationActive={false}
        />,
      ]);
    }

    if (chartType === 'metrics') {
      return agents.flatMap((agent, idx) => [
        <Line
          key={`${agent}_iaeX`}
          type="monotone"
          dataKey={`${agent}_iaeX`}
          stroke={COLORS[idx % COLORS.length]}
          strokeWidth={2}
          name={`${agent} - IAE X`}
          dot={false}
          isAnimationActive={false}
        />,
        <Line
          key={`${agent}_iaeY`}
          type="monotone"
          dataKey={`${agent}_iaeY`}
          stroke={COLORS[(idx + 1) % COLORS.length]}
          strokeWidth={2}
          strokeDasharray="5 5"
          name={`${agent} - IAE Y`}
          dot={false}
          isAnimationActive={false}
        />,
      ]);
    }

    if (chartType === 'overshoot') {
      return agents.flatMap((agent, idx) => [
        <Line
          key={`${agent}_overshootX`}
          type="monotone"
          dataKey={`${agent}_overshootX`}
          stroke={COLORS[idx % COLORS.length]}
          strokeWidth={2}
          name={`${agent} - Overshoot X`}
          dot={false}
          isAnimationActive={false}
        />,
        <Line
          key={`${agent}_rmse`}
          type="monotone"
          dataKey={`${agent}_rmse`}
          stroke={COLORS[(idx + 1) % COLORS.length]}
          strokeWidth={2}
          strokeDasharray="5 5"
          name={`${agent} - RMSE`}
          dot={false}
          isAnimationActive={false}
        />,
      ]);
    }

    return null;
  }, [chartType, metricsHistory, windHistory, selectedAgent]);

  const chartTitle = useMemo(() => {
    if (chartType === 'wind') {
      const hasForce = windHistory.some((w) => w.force);
      return hasForce ? 'Wind Velocity & Force' : 'Wind Velocity';
    }
    if (chartType === 'error') return 'Position Error';
    if (chartType === 'metrics') return 'Integral Absolute Error (IAE)';
    if (chartType === 'overshoot') return 'Overshoot & RMSE';
    return '';
  }, [chartType, windHistory]);

  const yAxisLabel = useMemo(() => {
    if (chartType === 'wind') return 'Value';
    if (chartType === 'error') return 'Error (m)';
    if (chartType === 'metrics') return 'IAE';
    if (chartType === 'overshoot') return 'Value (m)';
    return '';
  }, [chartType]);

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
      <div className="text-sm font-medium text-gray-300 mb-2">{chartTitle}</div>
      <ResponsiveContainer width="100%" height={400}>
        <LineChart data={chartData} margin={{ top: 5, right: 20, left: 0, bottom: 5 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="#374151" />
          <XAxis
            dataKey="timestamp"
            stroke="#9ca3af"
            tick={{ fill: '#9ca3af', fontSize: 11 }}
            tickFormatter={(ts) => new Date(ts).toLocaleTimeString()}
          />
          <YAxis
            stroke="#9ca3af"
            tick={{ fill: '#9ca3af', fontSize: 11 }}
            label={{
              value: yAxisLabel,
              angle: -90,
              position: 'insideLeft',
              fill: '#9ca3af',
              fontSize: 12,
            }}
          />
          <Tooltip content={<CustomTooltip />} />
          <Legend
            wrapperStyle={{ fontSize: '11px' }}
            iconType="line"
            iconSize={14}
          />
          {renderLines}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
};
