/**
 * WebSocket API client for real-time data streaming
 */

export interface MetricsData {
  agent_id: string;
  timestamp: number;
  current_x: number;
  current_y: number;
  current_z: number;
  target_x: number;
  target_y: number;
  target_z: number;
  error_x: number;
  error_y: number;
  error_z: number;
  error_magnitude: number;
  rmse_x: number;
  rmse_y: number;
  rmse_z: number;
  rmse_total: number;
  iae_x: number;
  iae_y: number;
  itae_x: number;
  itae_y: number;
  settling_time: number;
  is_settled: boolean;
  max_overshoot_x: number;
  max_overshoot_y: number;
}

export interface OdometryData {
  agent_id: string;
  timestamp: number;
  position: { x: number; y: number; z: number };
  velocity: { x: number; y: number; z: number };
  orientation_x: number;
  orientation_y: number;
  orientation_z: number;
  orientation_w: number;
}

export interface WindData {
  timestamp: number;
  velocity: { x: number; y: number; z: number };
  force?: { x: number; y: number; z: number };
}

export interface FormationState {
  timestamp: number;
  shape: string;
  spacing: number;
  center_x: number;
  center_y: number;
  center_z: number;
  yaw_deg: number;
  agent_ids: string[];
}

export interface DiagnosticsData {
  agent_id: string;
  timestamp: number;
  x_total: number;
  x_pid: number;
  x_fuzzy: number;
  y_total: number;
  y_pid: number;
  y_fuzzy: number;
}

export interface ControllerParams {
  agent_id: string;
  timestamp: number;
  controller_type: string;
  pid_kp: number;
  pid_ki: number;
  pid_kd: number;
  fuzzy_enable: boolean;
  fuzzy_wind_scalar: number;
  mix_k_pid: number;
  mix_k_fuzzy: number;
  feedforward_enable_drag: boolean;
  feedforward_enable_wind: boolean;
  feedforward_k_drag: number;
  feedforward_k_wind: number;
  output_limit_x_min: number;
  output_limit_x_max: number;
  output_limit_y_min: number;
  output_limit_y_max: number;
  control_frequency_hz: number;
  dt: number;
}

export interface SystemStatus {
  timestamp: number;
  ros_ok: boolean;
  active_agents: string[];
  available_topics: string[];
  wind_active: boolean;
  formation_active: boolean;
}

export interface SnapshotData {
  metrics: Record<string, MetricsData>;
  odom: Record<string, OdometryData>;
  target: Record<string, { position: { x: number; y: number; z: number } }>;
  diagnostics: Record<string, DiagnosticsData>;
  controller_params: Record<string, ControllerParams>;
  formation: FormationState | null;
  wind: WindData | null;
  status: SystemStatus;
}

export type DataUpdateCallback = (
  dataType: string,
  agentId: string | null,
  data: any
) => void;

export type ConnectionStatusCallback = (connected: boolean) => void;

export class WebSocketClient {
  private ws: WebSocket | null = null;
  private reconnectInterval = 2000;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private heartbeatTimer: any = null;
  private callbacks: Set<DataUpdateCallback> = new Set();
  private statusCallbacks: Set<ConnectionStatusCallback> = new Set();
  private url: string;
  private fallbackTried = false;

  public connected = false;

  constructor(url?: string) {
    // Allow explicit override via Vite env
    const envUrl = (typeof import.meta !== 'undefined' && (import.meta as any).env)
      ? (import.meta as any).env.VITE_BACKEND_WS_URL
      : undefined;

    if (url) {
      this.url = url;
    } else if (envUrl) {
      this.url = envUrl;
    } else if (typeof window !== 'undefined') {
      // Detect if we're served from /ui/ path (production build served by backend)
      const isUnderUi = window.location.pathname.startsWith('/ui');
      const wsPath = isUnderUi ? '/ui/ws' : '/ws';
      const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
      this.url = `${protocol}://${window.location.host}${wsPath}`;
    } else {
      // Fallback (shouldn't happen in browser)
      this.url = 'ws://localhost:8000/ws';
    }
  }

  connect(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return; // Already connected
    }

    console.log('Connecting to WebSocket:', this.url);

    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      console.log('WebSocket connected');
      this.connected = true;
      this.notifyStatusChange(true);
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
      if (this.heartbeatTimer) clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = setInterval(() => this.ping(), 15000);
    };

    this.ws.onclose = () => {
      console.log('WebSocket disconnected');
      this.connected = false;
      this.notifyStatusChange(false);
      if (this.heartbeatTimer) {
        clearInterval(this.heartbeatTimer);
        this.heartbeatTimer = null;
      }
      // Fallback: if using localhost and initial connect failed, try 127.0.0.1
      try {
        const u = new URL(this.url);
        if (!this.fallbackTried && (u.hostname === 'localhost' || u.hostname === '::1')) {
          this.fallbackTried = true;
          u.hostname = '127.0.0.1';
          this.url = u.toString();
          console.log('Retrying WebSocket with fallback host:', this.url);
        }
      } catch (_) {
        // ignore parse errors
      }
      this.scheduleReconnect();
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        this.handleMessage(message);
      } catch (error) {
        console.error('Error parsing WebSocket message:', error);
      }
    };
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;

    this.reconnectTimer = setTimeout(() => {
      console.log('Attempting to reconnect...');
      this.connect();
    }, this.reconnectInterval);
  }

  private handleMessage(message: any): void {
    const { type, data_type, agent_id, data } = message;

    if (type === 'snapshot') {
      if (!this.connected) {
        this.connected = true;
        this.notifyStatusChange(true);
      }
      // Initial data snapshot
      this.notifyCallbacks('snapshot', null, message.data);
    } else if (type === 'update') {
      if (!this.connected) {
        this.connected = true;
        this.notifyStatusChange(true);
      }
      // Real-time update
      this.notifyCallbacks(data_type, agent_id || null, data);
    } else if (type === 'pong') {
      // Heartbeat response
    }
  }

  subscribe(topics: string[], agents?: string[]): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('Cannot subscribe: WebSocket not connected');
      return;
    }

    this.ws.send(
      JSON.stringify({
        action: 'subscribe',
        topics,
        agents: agents || [],
      })
    );
  }

  onUpdate(callback: DataUpdateCallback): () => void {
    this.callbacks.add(callback);

    // Return unsubscribe function
    return () => {
      this.callbacks.delete(callback);
    };
  }

  onConnectionStatusChange(callback: ConnectionStatusCallback): () => void {
    this.statusCallbacks.add(callback);

    // Immediately notify of current status
    callback(this.connected);

    // Return unsubscribe function
    return () => {
      this.statusCallbacks.delete(callback);
    };
  }

  private notifyCallbacks(
    dataType: string,
    agentId: string | null,
    data: any
  ): void {
    this.callbacks.forEach((callback) => {
      try {
        callback(dataType, agentId, data);
      } catch (error) {
        console.error('Error in update callback:', error);
      }
    });
  }

  private notifyStatusChange(connected: boolean): void {
    this.statusCallbacks.forEach((callback) => {
      try {
        callback(connected);
      } catch (error) {
        console.error('Error in status callback:', error);
      }
    });
  }

  ping(): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action: 'ping' }));
    }
  }

  disconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this.connected = false;
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }
}

// Singleton instance
export const wsClient = new WebSocketClient();
