/**
 * Dashboard Layout - Robotics Control Interface
 * Design: Foxglove/Boston Dynamics inspired modular layout
 * Supports: Light/Dark themes, Thesis export mode
 */
import React, { useState, useCallback } from 'react';
import {
  Play,
  Square,
  Terminal,
  ChevronDown,
  ChevronUp,
  Loader2,
  Wifi,
  WifiOff,
  Radio,
} from 'lucide-react';
import { useSimulation } from '../hooks';
import { ThemeControls } from './ThemeToggle';

// ============================================================================
// Types
// ============================================================================

interface DashboardLayoutProps {
  connected: boolean;
  systemStatus: {
    ros_ok: boolean;
    active_agents: string[];
    available_topics?: string[];
  } | null;
  simulationEnded?: boolean;
  simulationEndReason?: string;
  leftPanel: React.ReactNode;
  rightPanel: React.ReactNode;
  bottomPanel: React.ReactNode;
}

// ============================================================================
// Main Component
// ============================================================================

export const DashboardLayout: React.FC<DashboardLayoutProps> = ({
  connected,
  systemStatus,
  simulationEnded = false,
  simulationEndReason = '',
  leftPanel,
  rightPanel,
  bottomPanel,
}) => {
  const [leftWidth, setLeftWidth] = useState(55);
  const [bottomHeight, setBottomHeight] = useState(160);
  const [bottomCollapsed, setBottomCollapsed] = useState(false);
  const [isResizingH, setIsResizingH] = useState(false);
  const [isResizingV, setIsResizingV] = useState(false);
  const [showScriptDropdown, setShowScriptDropdown] = useState(false);

  const {
    simStatus,
    availableScripts,
    selectedScript,
    simLoading,
    setSelectedScript,
    startSimulation,
    stopSimulation,
  } = useSimulation();

  const handleHorizontalResize = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    setIsResizingH(true);
    const startX = e.clientX;
    const startWidth = leftWidth;

    const onMouseMove = (e: MouseEvent) => {
      const delta = e.clientX - startX;
      const containerWidth = window.innerWidth;
      const newWidth = startWidth + (delta / containerWidth) * 100;
      setLeftWidth(Math.max(30, Math.min(70, newWidth)));
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
      setBottomHeight(Math.max(80, Math.min(350, startHeight + delta)));
    };

    const onMouseUp = () => {
      setIsResizingV(false);
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
    };

    document.addEventListener('mousemove', onMouseMove);
    document.addEventListener('mouseup', onMouseUp);
  }, [bottomHeight]);

  const agentCount = systemStatus?.active_agents?.length || 0;
  const topicCount = systemStatus?.available_topics?.length || 0;

  return (
    <div className={`flex flex-col h-screen overflow-hidden ${isResizingH || isResizingV ? 'select-none cursor-col-resize' : ''}`}
         style={{ background: 'var(--color-bg-primary)' }}>

      {/* Background Grid */}
      <div className="absolute inset-0 bg-grid-pattern opacity-40 pointer-events-none" />

      {/* Header Bar */}
      <header className="h-11 glass-panel border-b flex items-center justify-between px-3 shrink-0 z-20">
        {/* Left: Logo & Controls */}
        <div className="flex items-center gap-3">
          {/* Logo - Foxglove inspired */}
          <div className="flex items-center gap-2">
            <div
              className="w-7 h-7 rounded-md flex items-center justify-center"
              style={{ background: 'var(--color-accent-primary)' }}
            >
              {/* 3D axes symbol inspired by Foxglove */}
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none" className="text-white">
                <path d="M8 2V14M2 8H14M4 4L12 12M12 4L4 12" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
              </svg>
            </div>
            <div className="flex flex-col">
              <span
                className="text-sm font-semibold tracking-tight leading-none"
                style={{ color: 'var(--color-text-primary)' }}
              >
                Formation Control
              </span>
              <span
                className="text-[10px] tracking-wide"
                style={{ color: 'var(--color-text-muted)' }}
              >
                Multi-Agent Dashboard
              </span>
            </div>
          </div>

          <div className="w-px h-5" style={{ background: 'var(--color-border-subtle)' }} />

          {/* Simulation Controls */}
          <div className="flex items-center gap-2">
            {/* Script Selector */}
            <div className="relative">
              <button
                onClick={() => setShowScriptDropdown(!showScriptDropdown)}
                disabled={simStatus.running}
                className="h-7 px-2.5 text-xs font-medium rounded flex items-center gap-1.5 transition-colors disabled:opacity-50"
                style={{
                  background: 'var(--color-bg-tertiary)',
                  border: '1px solid var(--color-border-subtle)',
                  color: 'var(--color-text-secondary)',
                }}
              >
                <span className="max-w-32 truncate">{selectedScript}</span>
                <ChevronDown className="w-3 h-3 text-zinc-500" />
              </button>

              {showScriptDropdown && (
                <div className="absolute top-full left-0 mt-1 min-w-48 max-h-52 overflow-y-auto rounded-lg shadow-xl z-50 animate-slide-in"
                     style={{ background: 'var(--color-bg-elevated)', border: '1px solid var(--color-border-subtle)' }}>
                  {availableScripts.map((script) => (
                    <button
                      key={script.name}
                      onClick={() => {
                        setSelectedScript(script.name);
                        setShowScriptDropdown(false);
                      }}
                      className={`w-full text-left px-3 py-2 text-xs transition-colors ${
                        selectedScript === script.name
                          ? 'text-blue-400 bg-blue-500/10'
                          : 'text-zinc-300 hover:bg-zinc-800'
                      }`}
                    >
                      {script.name}
                    </button>
                  ))}
                  {availableScripts.length === 0 && (
                    <div className="px-3 py-4 text-xs text-zinc-500 text-center">No scripts available</div>
                  )}
                </div>
              )}
            </div>

            {/* Play/Stop Button */}
            {simLoading ? (
              <div className="w-7 h-7 flex items-center justify-center">
                <Loader2 className="w-4 h-4 animate-spin text-blue-400" />
              </div>
            ) : simStatus.running ? (
              <button
                onClick={stopSimulation}
                className="w-7 h-7 rounded flex items-center justify-center transition-colors bg-red-500/15 text-red-400 hover:bg-red-500/25 border border-red-500/30"
              >
                <Square className="w-3 h-3 fill-current" />
              </button>
            ) : (
              <button
                onClick={startSimulation}
                className="w-7 h-7 rounded flex items-center justify-center transition-colors bg-blue-500/15 text-blue-400 hover:bg-blue-500/25 border border-blue-500/30"
              >
                <Play className="w-3 h-3 fill-current" />
              </button>
            )}

            {/* Running Status */}
            {simStatus.running && (
              <div className="flex items-center gap-1.5 px-2">
                <span className="status-dot status-dot-success animate-pulse" />
                <span className="text-xs font-medium text-green-400">Running</span>
              </div>
            )}
          </div>
        </div>

        {/* Right: Status Indicators */}
        <div className="flex items-center gap-4">
          {/* System Stats */}
          <div className="flex items-center gap-3 text-xs">
            <div className="flex items-center gap-1.5">
              <Radio className="w-3 h-3" style={{ color: 'var(--color-text-muted)' }} />
              <span style={{ color: 'var(--color-text-muted)' }}>ROS:</span>
              <span style={{ color: systemStatus?.ros_ok ? 'var(--color-success)' : 'var(--color-error)' }}>
                {systemStatus?.ros_ok ? 'OK' : 'ERR'}
              </span>
            </div>
            <div className="flex items-center gap-1.5">
              <span style={{ color: 'var(--color-text-muted)' }}>Agents:</span>
              <span className="font-mono" style={{ color: 'var(--color-text-primary)' }}>{agentCount}</span>
            </div>
            <div className="flex items-center gap-1.5">
              <span style={{ color: 'var(--color-text-muted)' }}>Topics:</span>
              <span className="font-mono" style={{ color: 'var(--color-text-primary)' }}>{topicCount}</span>
            </div>
          </div>

          <div className="w-px h-4" style={{ background: 'var(--color-border-subtle)' }} />

          {/* Theme Controls */}
          <ThemeControls />

          <div className="w-px h-4" style={{ background: 'var(--color-border-subtle)' }} />

          {/* Connection Status */}
          <div className={`flex items-center gap-1.5 px-2 py-1 rounded text-xs font-medium ${
            connected
              ? 'border'
              : 'border'
          }`}
          style={{
            background: connected ? 'var(--color-success-bg)' : 'var(--color-error-bg)',
            borderColor: connected ? 'var(--color-success-border)' : 'var(--color-error-border)',
            color: connected ? 'var(--color-success)' : 'var(--color-error)',
          }}>
            {connected ? <Wifi className="w-3 h-3" /> : <WifiOff className="w-3 h-3" />}
            <span>{connected ? 'Live' : 'Offline'}</span>
          </div>
        </div>
      </header>

      {/* Simulation Ended Banner */}
      {simulationEnded && (
        <div className="h-8 flex items-center justify-between px-3 text-xs border-b animate-fade-in"
             style={{ background: 'rgba(245, 158, 11, 0.1)', borderColor: 'rgba(245, 158, 11, 0.2)' }}>
          <div className="flex items-center gap-2">
            <Terminal className="w-3.5 h-3.5 text-amber-400" />
            <span className="text-amber-200">Simulation ended:</span>
            <span className="font-mono text-amber-400">{simulationEndReason || 'completed'}</span>
          </div>
          <span className="text-amber-500/70">Data preserved</span>
        </div>
      )}

      {/* Main Content */}
      <div className="flex-1 flex overflow-hidden relative z-10">
        {/* Left Panel */}
        <div className="flex flex-col" style={{ width: `${leftWidth}%`, borderRight: '1px solid var(--color-border-subtle)' }}>
          <div className="flex-1 overflow-y-auto scrollbar-thin p-3" style={{ background: 'var(--color-bg-secondary)' }}>
            {leftPanel}
          </div>
        </div>

        {/* Resize Handle - Horizontal */}
        <div
          className={`w-1 cursor-col-resize flex items-center justify-center transition-colors ${
            isResizingH ? 'bg-blue-500' : 'bg-transparent hover:bg-zinc-700'
          }`}
          onMouseDown={handleHorizontalResize}
        />

        {/* Right Panel */}
        <div className="flex-1 flex flex-col min-w-0" style={{ background: 'var(--color-bg-primary)' }}>
          <div className="flex-1 overflow-hidden">{rightPanel}</div>
        </div>
      </div>

      {/* Resize Handle - Vertical */}
      {!bottomCollapsed && (
        <div
          className={`h-1 cursor-row-resize transition-colors z-20 ${
            isResizingV ? 'bg-blue-500' : 'bg-transparent hover:bg-zinc-700'
          }`}
          onMouseDown={handleVerticalResize}
        />
      )}

      {/* Bottom Panel */}
      <div
        className="shrink-0 z-20 flex flex-col"
        style={{
          height: bottomCollapsed ? '28px' : `${bottomHeight}px`,
          background: 'var(--color-bg-secondary)',
          borderTop: '1px solid var(--color-border-subtle)',
        }}
      >
        {/* Bottom Panel Header */}
        <div
          className="h-7 flex items-center px-3 cursor-pointer shrink-0 transition-colors hover:bg-zinc-800/50"
          style={{ borderBottom: bottomCollapsed ? 'none' : '1px solid var(--color-border-subtle)' }}
          onClick={() => setBottomCollapsed(!bottomCollapsed)}
        >
          <Terminal className="w-3.5 h-3.5 mr-2 text-zinc-500" />
          <span className="text-[11px] font-semibold text-zinc-400 uppercase tracking-wider flex-1">
            Diagnostics
          </span>
          {bottomCollapsed ? (
            <ChevronUp className="w-3.5 h-3.5 text-zinc-500" />
          ) : (
            <ChevronDown className="w-3.5 h-3.5 text-zinc-500" />
          )}
        </div>

        {/* Bottom Panel Content */}
        {!bottomCollapsed && (
          <div className="flex-1 overflow-y-auto" style={{ background: 'var(--color-bg-primary)' }}>
            {bottomPanel}
          </div>
        )}
      </div>
    </div>
  );
};

export default DashboardLayout;
