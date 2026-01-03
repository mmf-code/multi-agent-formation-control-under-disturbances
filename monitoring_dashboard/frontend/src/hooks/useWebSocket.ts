/**
 * WebSocket hook for real-time data streaming
 */
import { useEffect, useRef, useCallback } from 'react';
import { useDashboardStore } from '../store';
import type { SnapshotData, MetricsData, OdometryData, WindData, FormationState, SystemStatus, ControllerParams, RosGraphData, DiagnosticsData } from '../types';

// ============================================================================
// Types
// ============================================================================

interface WebSocketMessage {
  type: 'snapshot' | 'update' | 'pong' | 'server_heartbeat' | 'subscription_updated';
  data_type?: string;
  agent_id?: string;
  data?: unknown;
}

interface UseWebSocketOptions {
  autoConnect?: boolean;
  reconnectInterval?: number;
  heartbeatInterval?: number;
}

// ============================================================================
// URL Detection
// ============================================================================

function getWebSocketUrl(): string {
  const envUrl = import.meta.env?.VITE_BACKEND_WS_URL;

  if (envUrl) return envUrl;

  if (typeof window !== 'undefined') {
    const isUnderUi = window.location.pathname.startsWith('/ui');
    const wsPath = isUnderUi ? '/ui/ws' : '/ws';
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    return `${protocol}://${window.location.host}${wsPath}`;
  }

  return 'ws://localhost:8000/ws';
}

// ============================================================================
// Hook Implementation
// ============================================================================

export function useWebSocket(options: UseWebSocketOptions = {}) {
  const {
    autoConnect = true,
    reconnectInterval = 2000,
    heartbeatInterval = 15000,
  } = options;

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const heartbeatTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const urlRef = useRef(getWebSocketUrl());
  const fallbackTriedRef = useRef(false);

  // Store actions
  const {
    setConnected,
    updateMetrics,
    updateOdom,
    updateTarget,
    updateControllerParams,
    updateWind,
    updateFormation,
    updateSystemStatus,
    updateRosGraph,
    addLog,
    handleSnapshot,
    handleSimulationEvent,
  } = useDashboardStore();

  // Handle incoming messages
  const handleMessage = useCallback((event: MessageEvent) => {
    try {
      const message: WebSocketMessage = JSON.parse(event.data);
      const { type, data_type, agent_id, data } = message;

      if (type === 'snapshot' && data) {
        handleSnapshot(data as SnapshotData);
      } else if (type === 'update' && data_type) {
        switch (data_type) {
          case 'metrics':
            if (agent_id && data) {
              updateMetrics(agent_id, data as MetricsData);
            }
            break;
          case 'odom':
            if (agent_id && data) {
              updateOdom(agent_id, data as OdometryData);
            }
            break;
          case 'target':
            if (agent_id && data) {
              updateTarget(agent_id, data as { position: { x: number; y: number; z: number } });
            }
            break;
          case 'controller_params':
            if (agent_id && data) {
              updateControllerParams(agent_id, data as ControllerParams);
            }
            break;
          case 'wind':
            if (data) {
              updateWind(data as WindData);
            }
            break;
          case 'formation':
            if (data) {
              updateFormation(data as FormationState);
            }
            break;
          case 'status':
            if (data) {
              updateSystemStatus(data as SystemStatus);
            }
            break;
          case 'ros_graph':
            if (data) {
              updateRosGraph(data as RosGraphData);
            }
            break;
          case 'diagnostics':
            if (agent_id && data) {
              const diagData = data as DiagnosticsData;
              addLog({
                timestamp: diagData.timestamp,
                level: 'info',
                message: `Agent ${agent_id} diagnostics updated`,
                agent: agent_id,
              });
            }
            break;
          case 'simulation_event':
            if (data) {
              handleSimulationEvent(data as { event: string; timestamp?: number; reason?: string; script?: string });
            }
            break;
        }
      }
      // Ignore pong, server_heartbeat, subscription_updated
    } catch (error) {
      console.error('Error parsing WebSocket message:', error);
    }
  }, [
    handleSnapshot,
    updateMetrics,
    updateOdom,
    updateTarget,
    updateControllerParams,
    updateWind,
    updateFormation,
    updateSystemStatus,
    updateRosGraph,
    addLog,
    handleSimulationEvent,
  ]);

  // Connect function
  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) return;

    console.log('Connecting to WebSocket:', urlRef.current);

    const ws = new WebSocket(urlRef.current);
    wsRef.current = ws;

    ws.onopen = () => {
      console.log('WebSocket connected');
      setConnected(true);

      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = null;
      }

      // Start heartbeat
      if (heartbeatTimerRef.current) clearInterval(heartbeatTimerRef.current);
      heartbeatTimerRef.current = setInterval(() => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ action: 'ping' }));
        }
      }, heartbeatInterval);

      // Subscribe to topics
      ws.send(JSON.stringify({
        action: 'subscribe',
        topics: ['metrics', 'odom', 'wind', 'formation', 'diagnostics', 'controller_params'],
        agents: [],
      }));
    };

    ws.onclose = () => {
      console.log('WebSocket disconnected');
      setConnected(false);

      if (heartbeatTimerRef.current) {
        clearInterval(heartbeatTimerRef.current);
        heartbeatTimerRef.current = null;
      }

      // Try fallback URL
      try {
        const url = new URL(urlRef.current);
        if (!fallbackTriedRef.current && (url.hostname === 'localhost' || url.hostname === '::1')) {
          fallbackTriedRef.current = true;
          url.hostname = '127.0.0.1';
          urlRef.current = url.toString();
          console.log('Retrying WebSocket with fallback host:', urlRef.current);
        }
      } catch {
        // Ignore parse errors
      }

      // Schedule reconnect
      if (!reconnectTimerRef.current) {
        reconnectTimerRef.current = setTimeout(() => {
          reconnectTimerRef.current = null;
          connect();
        }, reconnectInterval);
      }
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    ws.onmessage = handleMessage;
  }, [setConnected, handleMessage, heartbeatInterval, reconnectInterval]);

  // Disconnect function
  const disconnect = useCallback(() => {
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }

    if (heartbeatTimerRef.current) {
      clearInterval(heartbeatTimerRef.current);
      heartbeatTimerRef.current = null;
    }

    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }

    setConnected(false);
  }, [setConnected]);

  // Auto-connect on mount
  useEffect(() => {
    if (autoConnect) {
      connect();
    }

    return () => {
      disconnect();
    };
  }, [autoConnect, connect, disconnect]);

  // Send function
  const send = useCallback((data: unknown) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(data));
    } else {
      console.warn('Cannot send: WebSocket not connected');
    }
  }, []);

  return {
    connect,
    disconnect,
    send,
  };
}
