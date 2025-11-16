/**
 * Formation Map Component
 * 2D visualization of drone positions and formation
 */
import React, { useMemo } from 'react';
import Plot from 'react-plotly.js';
import { OdometryData, FormationState, WindData } from '../api/ws';

interface FormationMapProps {
  odomData: Record<string, OdometryData>;
  targetData: Record<string, { position: { x: number; y: number; z: number } }>;
  formationState: FormationState | null;
  selectedAgents: Set<string>;
  windData?: WindData | null;
}

export const FormationMap: React.FC<FormationMapProps> = ({
  odomData,
  targetData,
  formationState,
  selectedAgents,
  windData,
}) => {
  const plotData = useMemo(() => {
    const agents = selectedAgents.size > 0
      ? Array.from(selectedAgents)
      : Object.keys(odomData);

    // Current positions
    const currentTrace = {
      x: agents.map((id) => odomData[id]?.position.x || 0),
      y: agents.map((id) => odomData[id]?.position.y || 0),
      mode: 'markers+text' as const,
      type: 'scatter' as const,
      name: 'Current Position',
      text: agents,
      textposition: 'top center' as const,
      marker: {
        size: 12,
        color: '#3b82f6',
        symbol: 'circle',
        line: { width: 2, color: '#60a5fa' },
      },
    };

    // Target positions
    const targetTrace = {
      x: agents.map((id) => targetData[id]?.position.x || 0),
      y: agents.map((id) => targetData[id]?.position.y || 0),
      mode: 'markers' as const,
      type: 'scatter' as const,
      name: 'Target Position',
      marker: {
        size: 10,
        color: '#10b981',
        symbol: 'x',
        line: { width: 2, color: '#34d399' },
      },
    };

    // Connection lines (current to target)
    const connectionLines = agents.flatMap((id) => {
      const current = odomData[id]?.position;
      const target = targetData[id]?.position;
      if (!current || !target) return [];

      return [
        {
          x: [current.x, target.x],
          y: [current.y, target.y],
          mode: 'lines' as const,
          type: 'scatter' as const,
          showlegend: false,
          line: { color: '#6b7280', width: 1, dash: 'dot' as const },
        },
      ];
    });

    // Formation center
    const centerTrace = formationState
      ? {
          x: [formationState.center_x],
          y: [formationState.center_y],
          mode: 'markers' as const,
          type: 'scatter' as const,
          name: 'Formation Center',
          marker: {
            size: 8,
            color: '#f59e0b',
            symbol: 'diamond',
            line: { width: 1, color: '#000' },
          },
        }
      : null;

    const traces: any[] = [
      ...connectionLines,
      currentTrace,
      targetTrace,
    ];

    if (centerTrace) {
      traces.push(centerTrace);
    }

    // Wind direction arrow (if wind data available)
    if (windData && windData.velocity) {
      const windMag = Math.sqrt(windData.velocity.x ** 2 + windData.velocity.y ** 2);
      if (windMag > 0.1) {
        // Scale arrow length based on wind magnitude (cap at 3m for visualization)
        const arrowLength = Math.min(windMag * 0.5, 3);
        const windX = windData.velocity.x;
        const windY = windData.velocity.y;
        const normX = (windX / windMag) * arrowLength;
        const normY = (windY / windMag) * arrowLength;

        // Position arrow in top-right corner of the plot
        const bounds = {
          x: [...agents.map((id) => odomData[id]?.position.x || 0),
              ...agents.map((id) => targetData[id]?.position.x || 0)],
          y: [...agents.map((id) => odomData[id]?.position.y || 0),
              ...agents.map((id) => targetData[id]?.position.y || 0)],
        };

        const maxX = Math.max(...bounds.x, 0);
        const maxY = Math.max(...bounds.y, 0);
        const arrowBaseX = maxX + 1;
        const arrowBaseY = maxY + 1;

        const windArrow = {
          x: [arrowBaseX, arrowBaseX + normX],
          y: [arrowBaseY, arrowBaseY + normY],
          mode: 'lines+markers' as const,
          type: 'scatter' as const,
          name: `Wind ${windMag.toFixed(2)} m/s`,
          line: { color: '#f97316', width: 3 },
          marker: {
            size: [0, 12],
            color: '#f97316',
            symbol: ['circle', 'arrow'],
            angleref: 'previous' as const,
          },
        };

        traces.push(windArrow);
      }
    }

    return traces;
  }, [odomData, targetData, formationState, selectedAgents, windData]);

  // Calculate bounds
  const bounds = useMemo(() => {
    const allX: number[] = [];
    const allY: number[] = [];

    Object.values(odomData).forEach((odom) => {
      allX.push(odom.position.x);
      allY.push(odom.position.y);
    });

    Object.values(targetData).forEach((target) => {
      allX.push(target.position.x);
      allY.push(target.position.y);
    });

    if (formationState) {
      allX.push(formationState.center_x);
      allY.push(formationState.center_y);
    }

    const xRange = allX.length > 0
      ? [Math.min(...allX) - 2, Math.max(...allX) + 2]
      : [-5, 5];

    const yRange = allY.length > 0
      ? [Math.min(...allY) - 2, Math.max(...allY) + 2]
      : [-5, 5];

    return { xRange, yRange };
  }, [odomData, targetData, formationState]);

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
      <div className="flex justify-between items-center mb-2">
        <h3 className="text-lg font-semibold text-white">Formation Map (X-Y Plane)</h3>
        {formationState && (
          <div className="text-sm text-gray-400">
            Shape: <span className="text-purple-400 font-semibold">{formationState.shape}</span>
            {' | '}
            Spacing: <span className="text-cyan-400 font-semibold">{formationState.spacing.toFixed(2)}m</span>
          </div>
        )}
      </div>

      <Plot
        data={plotData as any}
        layout={{
          autosize: true,
          height: 500,
          xaxis: {
            title: 'X Position (m)',
            color: '#9ca3af',
            range: bounds.xRange,
            gridcolor: '#374151',
          },
          yaxis: {
            title: 'Y Position (m)',
            color: '#9ca3af',
            range: bounds.yRange,
            gridcolor: '#374151',
            scaleanchor: 'x',
            scaleratio: 1,
          },
          paper_bgcolor: '#1f2937',
          plot_bgcolor: '#111827',
          font: { color: '#e5e7eb' },
          margin: { l: 60, r: 40, t: 20, b: 60 },
          hovermode: 'closest' as const,
          showlegend: true,
          legend: {
            x: 0,
            y: 1,
            bgcolor: 'rgba(31, 41, 55, 0.8)',
            bordercolor: '#4b5563',
            borderwidth: 1,
          },
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
