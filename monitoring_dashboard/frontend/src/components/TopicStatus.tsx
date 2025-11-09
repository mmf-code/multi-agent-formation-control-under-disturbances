/**
 * Topic Status Component
 * Interactive topic selection and status display
 */
import React from 'react';
import { Activity, Wind, Users, Target } from 'lucide-react';

interface TopicStatusProps {
  status: {
    ros_ok: boolean;
    active_agents: string[];
    available_topics: string[];
    wind_active: boolean;
    formation_active: boolean;
  } | null;
  selectedTopics: Set<string>;
  selectedAgents: Set<string>;
  onTopicToggle: (topic: string) => void;
  onAgentToggle: (agent: string) => void;
}

const TOPIC_CATEGORIES = [
  { key: 'metrics', label: 'Metrics', icon: Activity, color: 'text-blue-400' },
  { key: 'odom', label: 'Odometry', icon: Target, color: 'text-green-400' },
  { key: 'wind', label: 'Wind', icon: Wind, color: 'text-cyan-400' },
  { key: 'formation', label: 'Formation', icon: Users, color: 'text-purple-400' },
  { key: 'diagnostics', label: 'Diagnostics', icon: Activity, color: 'text-orange-400' },
];

export const TopicStatus: React.FC<TopicStatusProps> = ({
  status,
  selectedTopics,
  selectedAgents,
  onTopicToggle,
  onAgentToggle,
}) => {
  if (!status) {
    return (
      <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
        <div className="flex items-center space-x-2">
          <div className="w-3 h-3 bg-yellow-500 rounded-full animate-pulse"></div>
          <span className="text-gray-400">Connecting to backend...</span>
        </div>
      </div>
    );
  }

  return (
    <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
      {/* System Status Header */}
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-white">System Status</h3>
        <div className="flex items-center space-x-2">
          <div className={`w-3 h-3 rounded-full ${
            status.ros_ok ? 'bg-green-500' : 'bg-red-500'
          } ${status.ros_ok ? 'animate-pulse' : ''}`}></div>
          <span className={`text-sm font-medium ${
            status.ros_ok ? 'text-green-400' : 'text-red-400'
          }`}>
            {status.ros_ok ? 'ROS2 Active' : 'ROS2 Disconnected'}
          </span>
        </div>
      </div>

      {/* Topic Selection */}
      <div className="mb-4">
        <h4 className="text-sm font-semibold text-gray-400 mb-2 uppercase tracking-wide">
          Active Topics
        </h4>
        <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-5 gap-2">
          {TOPIC_CATEGORIES.map(({ key, label, icon: Icon, color }) => {
            const isSelected = selectedTopics.has(key);
            const isActive = key === 'wind' ? status.wind_active :
                           key === 'formation' ? status.formation_active : true;

            return (
              <button
                key={key}
                onClick={() => onTopicToggle(key)}
                disabled={!isActive}
                className={`flex items-center space-x-2 px-3 py-2 rounded-lg border transition-all ${
                  isSelected
                    ? 'bg-blue-900 border-blue-500 text-blue-200'
                    : 'bg-gray-700 border-gray-600 text-gray-400 hover:bg-gray-600'
                } ${!isActive ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}`}
              >
                <Icon className={`w-4 h-4 ${isSelected ? color : 'text-gray-500'}`} />
                <span className="text-sm font-medium">{label}</span>
                {isSelected && (
                  <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse ml-auto"></div>
                )}
              </button>
            );
          })}
        </div>
      </div>

      {/* Agent Selection */}
      <div>
        <h4 className="text-sm font-semibold text-gray-400 mb-2 uppercase tracking-wide">
          Active Agents ({status.active_agents.length})
        </h4>
        <div className="flex flex-wrap gap-2">
          {status.active_agents.length === 0 && (
            <div className="text-gray-500 text-sm">No agents detected</div>
          )}
          {status.active_agents.map((agent) => {
            const isSelected = selectedAgents.has(agent);

            return (
              <button
                key={agent}
                onClick={() => onAgentToggle(agent)}
                className={`px-3 py-1 rounded-md border text-sm font-medium transition-all ${
                  isSelected
                    ? 'bg-green-900 border-green-500 text-green-200'
                    : 'bg-gray-700 border-gray-600 text-gray-400 hover:bg-gray-600'
                }`}
              >
                {agent}
                {isSelected && (
                  <span className="ml-2 text-green-400">✓</span>
                )}
              </button>
            );
          })}
        </div>
      </div>

      {/* Quick Stats */}
      <div className="mt-4 pt-4 border-t border-gray-700 grid grid-cols-3 gap-4 text-center">
        <div>
          <div className="text-2xl font-bold text-blue-400">
            {status.available_topics.length}
          </div>
          <div className="text-xs text-gray-500">Topics</div>
        </div>
        <div>
          <div className="text-2xl font-bold text-green-400">
            {status.active_agents.length}
          </div>
          <div className="text-xs text-gray-500">Agents</div>
        </div>
        <div>
          <div className="text-2xl font-bold text-purple-400">
            {selectedTopics.size}
          </div>
          <div className="text-xs text-gray-500">Subscribed</div>
        </div>
      </div>
    </div>
  );
};
