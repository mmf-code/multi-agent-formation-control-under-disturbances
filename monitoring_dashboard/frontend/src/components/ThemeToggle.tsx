/**
 * Theme Toggle Component
 * Single button to toggle Thesis mode (light + export optimizations)
 */
import React from 'react';
import { FileText, Monitor } from 'lucide-react';
import { useThemeStore, useThesisMode } from '../store/theme';

// ============================================================================
// Thesis Mode Toggle - Single Button
// ============================================================================

export const ThemeControls: React.FC = () => {
  const thesisMode = useThesisMode();
  const toggleThesisMode = useThemeStore((s) => s.toggleThesisMode);

  return (
    <button
      onClick={toggleThesisMode}
      className="h-7 px-2.5 rounded flex items-center gap-1.5 transition-all text-xs font-medium border"
      style={{
        background: thesisMode ? 'var(--color-accent-muted)' : 'var(--color-bg-tertiary)',
        borderColor: thesisMode ? 'var(--color-accent-primary)' : 'var(--color-border-subtle)',
        color: thesisMode ? 'var(--color-accent-primary)' : 'var(--color-text-secondary)',
      }}
      title={thesisMode ? 'Exit Thesis Mode (return to dark theme)' : 'Enter Thesis Mode (light theme for screenshots)'}
    >
      {thesisMode ? (
        <>
          <FileText className="w-3.5 h-3.5" />
          <span>Thesis</span>
        </>
      ) : (
        <>
          <Monitor className="w-3.5 h-3.5" />
          <span>Normal</span>
        </>
      )}
    </button>
  );
};

export default ThemeControls;
