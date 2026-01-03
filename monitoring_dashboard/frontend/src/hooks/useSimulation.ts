/**
 * Simulation control hook
 */
import { useEffect, useCallback } from 'react';
import { toast } from 'sonner';
import { useDashboardStore } from '../store';

// ============================================================================
// API Helpers
// ============================================================================

function getApiUrl(path: string): string {
  // In production, we're served from /ui/, but API is at root
  if (typeof window !== 'undefined' && window.location.pathname.startsWith('/ui')) {
    return path;
  }
  return path;
}

// ============================================================================
// Hook Implementation
// ============================================================================

export function useSimulation() {
  const {
    simStatus,
    availableScripts,
    selectedScript,
    simLoading,
    setSimStatus,
    setAvailableScripts,
    setSelectedScript,
    setSimLoading,
  } = useDashboardStore();

  // Fetch simulation status and available scripts
  const fetchSimData = useCallback(async () => {
    try {
      const [statusRes, scriptsRes] = await Promise.all([
        fetch(getApiUrl('/api/simulation/status')),
        fetch(getApiUrl('/api/simulation/scripts')),
      ]);

      if (statusRes.ok) {
        const status = await statusRes.json();
        setSimStatus(status);
      }

      if (scriptsRes.ok) {
        const data = await scriptsRes.json();
        setAvailableScripts(data.scripts || []);
      }
    } catch (e) {
      console.error('Failed to fetch simulation data:', e);
    }
  }, [setSimStatus, setAvailableScripts]);

  // Poll simulation status
  useEffect(() => {
    fetchSimData();
    const interval = setInterval(fetchSimData, 3000);
    return () => clearInterval(interval);
  }, [fetchSimData]);

  // Start simulation
  const startSimulation = useCallback(async () => {
    setSimLoading(true);

    try {
      const url = getApiUrl(`/api/simulation/start?script=${encodeURIComponent(selectedScript)}`);
      console.log('Starting simulation:', url, selectedScript);

      const res = await fetch(url, { method: 'POST' });

      if (res.ok) {
        const data = await res.json();
        setSimStatus({ running: true, pid: data.pid, script: data.script });
        toast.success(`Simulation started: ${selectedScript}`, {
          description: `PID: ${data.pid}`,
        });
      } else {
        const err = await res.json();
        toast.error('Failed to start simulation', {
          description: err.error || 'Unknown error',
        });
      }
    } catch (e) {
      console.error('Start simulation error:', e);
      toast.error('Failed to start simulation', {
        description: e instanceof Error ? e.message : 'Network error',
      });
    } finally {
      setSimLoading(false);
    }
  }, [selectedScript, setSimLoading, setSimStatus]);

  // Stop simulation
  const stopSimulation = useCallback(async () => {
    setSimLoading(true);

    try {
      const res = await fetch(getApiUrl('/api/simulation/stop'), { method: 'POST' });

      if (res.ok) {
        setSimStatus({ running: false, pid: null, script: null });
        toast.info('Simulation stopped');
      } else {
        const err = await res.json();
        toast.error('Failed to stop simulation', {
          description: err.error || 'Unknown error',
        });
      }
    } catch (e) {
      console.error('Stop simulation error:', e);
      toast.error('Failed to stop simulation', {
        description: e instanceof Error ? e.message : 'Network error',
      });
    } finally {
      setSimLoading(false);
    }
  }, [setSimLoading, setSimStatus]);

  return {
    simStatus,
    availableScripts,
    selectedScript,
    simLoading,
    setSelectedScript,
    startSimulation,
    stopSimulation,
    refreshStatus: fetchSimData,
  };
}
