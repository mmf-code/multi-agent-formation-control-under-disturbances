/**
 * Theme Store - Thesis mode for academic publications
 * Thesis mode = Light theme + clean export styling
 */
import { create } from 'zustand';
import { persist } from 'zustand/middleware';

// ============================================================================
// Types
// ============================================================================

interface ThemeState {
  // Thesis mode: combines light theme + export optimizations
  thesisMode: boolean;

  // Actions
  setThesisMode: (enabled: boolean) => void;
  toggleThesisMode: () => void;
}

// ============================================================================
// Store
// ============================================================================

export const useThemeStore = create<ThemeState>()(
  persist(
    (set, get) => ({
      thesisMode: false,

      setThesisMode: (enabled) => {
        set({ thesisMode: enabled });
        applyTheme(enabled);
      },

      toggleThesisMode: () => {
        const newMode = !get().thesisMode;
        set({ thesisMode: newMode });
        applyTheme(newMode);
      },
    }),
    {
      name: 'dashboard-theme',
      onRehydrateStorage: () => (state) => {
        if (state) {
          applyTheme(state.thesisMode);
        }
      },
    }
  )
);

// ============================================================================
// Theme Application
// ============================================================================

function applyTheme(thesisMode: boolean) {
  const root = document.documentElement;

  // Remove existing theme classes
  root.classList.remove('light', 'dark', 'thesis-mode');

  if (thesisMode) {
    // Thesis mode: light theme + export optimizations
    root.classList.add('light', 'thesis-mode');
    root.style.colorScheme = 'light';
  } else {
    // Normal mode: dark theme
    root.classList.add('dark');
    root.style.colorScheme = 'dark';
  }
}

// ============================================================================
// Selectors
// ============================================================================

export const useThesisMode = () => useThemeStore((state) => state.thesisMode);
export const useTheme = () => useThemeStore((state) => state.thesisMode ? 'light' : 'dark');
export const useIsLightMode = () => useThemeStore((state) => state.thesisMode);

// ============================================================================
// Initialize theme on load
// ============================================================================

export function initializeTheme() {
  const stored = localStorage.getItem('dashboard-theme');
  if (stored) {
    try {
      const parsed = JSON.parse(stored);
      applyTheme(parsed.state?.thesisMode || false);
    } catch {
      applyTheme(false);
    }
  } else {
    applyTheme(false);
  }
}
