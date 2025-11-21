import React from 'react';
import { EventLog, LogEntry } from './EventLog';

interface DiagnosticsPanelProps {
  logs: LogEntry[];
}

export const DiagnosticsPanel: React.FC<DiagnosticsPanelProps> = ({ logs }) => {
  return (
    <div className="h-full">
      <EventLog logs={logs} maxEntries={50} />
    </div>
  );
};
