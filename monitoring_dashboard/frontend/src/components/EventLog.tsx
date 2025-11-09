/**
 * Event Log Component
 * Displays recent events and system messages
 */
import React from 'react';
import { Clock, AlertCircle, Info, CheckCircle } from 'lucide-react';

export interface LogEntry {
  timestamp: number;
  level: 'info' | 'warning' | 'success' | 'error';
  message: string;
  agent?: string;
}

interface EventLogProps {
  logs: LogEntry[];
  maxEntries?: number;
}

export const EventLog: React.FC<EventLogProps> = ({ logs, maxEntries = 100 }) => {
  const displayLogs = logs.slice(-maxEntries).reverse();

  const getIcon = (level: string) => {
    switch (level) {
      case 'success':
        return <CheckCircle className="w-4 h-4 text-green-400" />;
      case 'warning':
        return <AlertCircle className="w-4 h-4 text-yellow-400" />;
      case 'error':
        return <AlertCircle className="w-4 h-4 text-red-400" />;
      default:
        return <Info className="w-4 h-4 text-blue-400" />;
    }
  };

  const getTextColor = (level: string) => {
    switch (level) {
      case 'success':
        return 'text-green-300';
      case 'warning':
        return 'text-yellow-300';
      case 'error':
        return 'text-red-300';
      default:
        return 'text-gray-300';
    }
  };

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
      <h3 className="text-lg font-semibold text-white mb-3 flex items-center">
        <Clock className="w-5 h-5 mr-2" />
        Event Log
      </h3>

      <div className="space-y-2 max-h-96 overflow-y-auto">
        {displayLogs.length === 0 && (
          <div className="text-gray-500 text-sm text-center py-8">
            No events yet
          </div>
        )}

        {displayLogs.map((log, index) => (
          <div
            key={`${log.timestamp}-${index}`}
            className="flex items-start space-x-3 p-2 rounded bg-gray-700 hover:bg-gray-650 transition-colors"
          >
            <div className="mt-0.5">{getIcon(log.level)}</div>
            <div className="flex-1 min-w-0">
              <div className="flex items-center space-x-2">
                <span className="text-xs text-gray-500 font-mono">
                  {new Date(log.timestamp * 1000).toLocaleTimeString()}
                </span>
                {log.agent && (
                  <span className="text-xs bg-blue-900 text-blue-300 px-2 py-0.5 rounded">
                    {log.agent}
                  </span>
                )}
              </div>
              <p className={`text-sm mt-1 ${getTextColor(log.level)}`}>
                {log.message}
              </p>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};
