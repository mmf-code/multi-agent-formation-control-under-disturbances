/**
 * Shared UI Components - Theme-aware Professional Design
 * Supports: Light/Dark mode, Thesis export mode
 */
import React from 'react';
import { clsx } from 'clsx';
import {
  CheckCircle,
  XCircle,
  AlertTriangle,
  Info,
  Loader2,
  type LucideIcon,
} from 'lucide-react';
import type { ControllerType, LogLevel } from '../../types';
import { CONTROLLER_LABELS } from '../../types';

// ============================================================================
// Color Types - Theme Aware
// ============================================================================

export type ColorVariant = 'green' | 'red' | 'blue' | 'gray' | 'purple' | 'cyan' | 'yellow' | 'amber' | 'emerald' | 'violet';

// Theme-aware color classes using CSS variables
const COLOR_CLASSES: Record<ColorVariant, { bg: string; text: string; border: string }> = {
  green: {
    bg: 'bg-[var(--color-success-bg)]',
    text: 'text-[var(--color-success)]',
    border: 'border-[var(--color-success-border)]',
  },
  red: {
    bg: 'bg-[var(--color-error-bg)]',
    text: 'text-[var(--color-error)]',
    border: 'border-[var(--color-error-border)]',
  },
  blue: {
    bg: 'bg-[var(--color-accent-muted)]',
    text: 'text-[var(--color-accent-primary)]',
    border: 'border-[var(--color-accent-primary)]/30',
  },
  gray: {
    bg: 'bg-[var(--color-bg-tertiary)]',
    text: 'text-[var(--color-text-secondary)]',
    border: 'border-[var(--color-border-subtle)]',
  },
  purple: {
    bg: 'bg-purple-500/10',
    text: 'text-purple-500 dark:text-purple-400',
    border: 'border-purple-500/30',
  },
  cyan: {
    bg: 'bg-[var(--color-info-bg)]',
    text: 'text-[var(--color-info)]',
    border: 'border-[var(--color-info-border)]',
  },
  yellow: {
    bg: 'bg-[var(--color-warning-bg)]',
    text: 'text-[var(--color-warning)]',
    border: 'border-[var(--color-warning-border)]',
  },
  amber: {
    bg: 'bg-[var(--color-warning-bg)]',
    text: 'text-[var(--color-warning)]',
    border: 'border-[var(--color-warning-border)]',
  },
  emerald: {
    bg: 'bg-[var(--color-success-bg)]',
    text: 'text-[var(--color-success)]',
    border: 'border-[var(--color-success-border)]',
  },
  violet: {
    bg: 'bg-[var(--color-accent-muted)]',
    text: 'text-[var(--color-accent-primary)]',
    border: 'border-[var(--color-accent-primary)]/30',
  },
};

// Controller colors using CSS variables
const CONTROLLER_COLOR_CLASSES: Record<string, { bg: string; text: string; border: string }> = {
  pid_fuzzy: {
    bg: 'bg-[var(--color-pid-fuzzy-bg)]',
    text: 'text-[var(--color-pid-fuzzy)]',
    border: 'border-[var(--color-pid-fuzzy-border)]',
  },
  hybrid: {
    bg: 'bg-[var(--color-pid-fuzzy-bg)]',
    text: 'text-[var(--color-pid-fuzzy)]',
    border: 'border-[var(--color-pid-fuzzy-border)]',
  },
  pd: {
    bg: 'bg-[var(--color-pd-bg)]',
    text: 'text-[var(--color-pd)]',
    border: 'border-[var(--color-pd-border)]',
  },
  pid: {
    bg: 'bg-[var(--color-pid-bg)]',
    text: 'text-[var(--color-pid)]',
    border: 'border-[var(--color-pid-border)]',
  },
  unknown: {
    bg: 'bg-[var(--color-bg-tertiary)]',
    text: 'text-[var(--color-text-muted)]',
    border: 'border-[var(--color-border-subtle)]',
  },
};

// ============================================================================
// Panel Component - Theme Aware
// ============================================================================

interface PanelProps {
  children: React.ReactNode;
  className?: string;
  title?: string;
  icon?: React.ReactNode;
  action?: React.ReactNode;
  noPadding?: boolean;
}

export const Panel: React.FC<PanelProps> = ({
  children,
  className,
  title,
  icon,
  action,
  noPadding = false,
}) => {
  return (
    <div
      className={clsx('rounded-lg border shadow-panel', className)}
      style={{
        background: 'var(--color-bg-secondary)',
        borderColor: 'var(--color-border-subtle)',
      }}
    >
      {title && (
        <div
          className="flex items-center justify-between px-4 py-2.5 border-b"
          style={{
            borderColor: 'var(--color-border-subtle)',
            background: 'var(--color-bg-tertiary)',
          }}
        >
          <div className="flex items-center gap-2">
            {icon && <span style={{ color: 'var(--color-text-muted)' }}>{icon}</span>}
            <h3
              className="text-sm font-medium"
              style={{ color: 'var(--color-text-secondary)' }}
            >
              {title}
            </h3>
          </div>
          {action}
        </div>
      )}
      <div className={clsx({ 'p-4': !noPadding })}>{children}</div>
    </div>
  );
};

// ============================================================================
// Status Card Component - Theme Aware
// ============================================================================

interface StatusCardProps {
  label: string;
  value: string | number;
  subtext?: string;
  icon?: React.ReactNode;
  color?: ColorVariant;
  size?: 'sm' | 'md' | 'lg';
  pulse?: boolean;
}

export const StatusCard: React.FC<StatusCardProps> = ({
  label,
  value,
  subtext,
  icon,
  color = 'gray',
  size = 'md',
  pulse = false,
}) => {
  const colorClasses = COLOR_CLASSES[color];
  const sizeClasses = { sm: 'p-2', md: 'p-3', lg: 'p-4' };
  const valueSizeClasses = { sm: 'text-base', md: 'text-lg', lg: 'text-xl' };

  return (
    <div
      className={clsx(
        'rounded-lg border',
        colorClasses.bg,
        colorClasses.border,
        sizeClasses[size]
      )}
    >
      <div className="flex items-center justify-between mb-1">
        <span
          className="text-[11px] uppercase tracking-wide font-medium"
          style={{ color: 'var(--color-text-muted)' }}
        >
          {label}
        </span>
        {icon && (
          <span className={clsx(colorClasses.text, 'opacity-70', { 'animate-pulse': pulse })}>
            {icon}
          </span>
        )}
      </div>
      <div className={clsx('font-semibold font-mono', valueSizeClasses[size], colorClasses.text)}>
        {value}
      </div>
      {subtext && (
        <div className="text-[11px] mt-0.5" style={{ color: 'var(--color-text-muted)' }}>
          {subtext}
        </div>
      )}
    </div>
  );
};

// ============================================================================
// Badge Component - Theme Aware
// ============================================================================

interface BadgeProps {
  children: React.ReactNode;
  color?: ColorVariant;
  size?: 'xs' | 'sm' | 'md';
  pulse?: boolean;
  className?: string;
}

export const Badge: React.FC<BadgeProps> = ({
  children,
  color = 'gray',
  size = 'sm',
  pulse = false,
  className,
}) => {
  const colorClasses = COLOR_CLASSES[color];
  const sizeClasses = {
    xs: 'px-1.5 py-0.5 text-[10px]',
    sm: 'px-2 py-0.5 text-xs',
    md: 'px-2.5 py-1 text-sm',
  };

  return (
    <span
      className={clsx(
        'inline-flex items-center gap-1 rounded font-medium border',
        colorClasses.bg,
        colorClasses.text,
        colorClasses.border,
        sizeClasses[size],
        { 'animate-pulse': pulse },
        className
      )}
    >
      {children}
    </span>
  );
};

// ============================================================================
// Controller Badge Component - Theme Aware
// ============================================================================

interface ControllerBadgeProps {
  type: ControllerType;
  size?: 'xs' | 'sm' | 'md';
  showLabel?: boolean;
}

export const ControllerBadge: React.FC<ControllerBadgeProps> = ({
  type,
  size = 'sm',
  showLabel = true,
}) => {
  const colors = CONTROLLER_COLOR_CLASSES[type] || CONTROLLER_COLOR_CLASSES.unknown;
  const label = CONTROLLER_LABELS[type] || type.toUpperCase();
  const sizeClasses = {
    xs: 'px-1.5 py-0.5 text-[10px]',
    sm: 'px-2 py-0.5 text-xs',
    md: 'px-2.5 py-1 text-sm',
  };

  return (
    <span
      className={clsx(
        'inline-flex items-center gap-1 rounded font-semibold border',
        colors.bg,
        colors.text,
        colors.border,
        sizeClasses[size]
      )}
    >
      {showLabel ? label : type}
    </span>
  );
};

// ============================================================================
// Status Indicator Component - Theme Aware
// ============================================================================

interface StatusIndicatorProps {
  status: 'online' | 'offline' | 'warning' | 'error' | 'loading';
  label?: string;
  showLabel?: boolean;
  size?: 'sm' | 'md' | 'lg';
}

export const StatusIndicator: React.FC<StatusIndicatorProps> = ({
  status,
  label,
  showLabel = true,
  size = 'md',
}) => {
  const statusConfig = {
    online: { color: 'emerald' as ColorVariant, text: 'Online' },
    offline: { color: 'red' as ColorVariant, text: 'Offline' },
    warning: { color: 'yellow' as ColorVariant, text: 'Warning' },
    error: { color: 'red' as ColorVariant, text: 'Error' },
    loading: { color: 'blue' as ColorVariant, text: 'Loading' },
  };

  const config = statusConfig[status];
  const colorClasses = COLOR_CLASSES[config.color];
  const sizeClasses = {
    sm: { dot: 'w-1.5 h-1.5', text: 'text-xs' },
    md: { dot: 'w-2 h-2', text: 'text-xs' },
    lg: { dot: 'w-2.5 h-2.5', text: 'text-sm' },
  };

  return (
    <div
      className={clsx(
        'flex items-center gap-2 px-2.5 py-1.5 rounded-md border',
        colorClasses.bg,
        colorClasses.border
      )}
    >
      <div
        className={clsx(
          'rounded-full',
          sizeClasses[size].dot,
          (status === 'online' || status === 'loading') && 'animate-pulse'
        )}
        style={{
          background: `var(--color-${config.color === 'emerald' ? 'success' : config.color === 'red' ? 'error' : config.color === 'yellow' ? 'warning' : 'accent-primary'})`,
        }}
      />
      {showLabel && (
        <span className={clsx('font-medium', sizeClasses[size].text, colorClasses.text)}>
          {label || config.text}
        </span>
      )}
    </div>
  );
};

// ============================================================================
// Metric Display Component - Theme Aware
// ============================================================================

interface MetricDisplayProps {
  label: string;
  value: number | string;
  unit?: string;
  precision?: number;
  color?: ColorVariant;
  size?: 'sm' | 'md' | 'lg';
  highlight?: boolean;
}

export const MetricDisplay: React.FC<MetricDisplayProps> = ({
  label,
  value,
  unit,
  precision = 3,
  color = 'gray',
  size = 'md',
  highlight = false,
}) => {
  const colorClasses = COLOR_CLASSES[color];
  const displayValue = typeof value === 'number' ? value.toFixed(precision) : value;
  const sizeClasses = {
    sm: { value: 'text-sm', label: 'text-[10px]' },
    md: { value: 'text-base', label: 'text-xs' },
    lg: { value: 'text-xl', label: 'text-sm' },
  };

  return (
    <div className="flex flex-col">
      <span
        className={sizeClasses[size].label}
        style={{ color: 'var(--color-text-muted)' }}
      >
        {label}
      </span>
      <span
        className={clsx('font-mono font-semibold', sizeClasses[size].value)}
        style={{ color: highlight ? undefined : 'var(--color-text-primary)' }}
      >
        <span className={highlight ? colorClasses.text : undefined}>{displayValue}</span>
        {unit && (
          <span className="text-xs ml-1" style={{ color: 'var(--color-text-muted)' }}>
            {unit}
          </span>
        )}
      </span>
    </div>
  );
};

// ============================================================================
// Progress Bar Component - Theme Aware
// ============================================================================

interface ProgressBarProps {
  value: number;
  max?: number;
  color?: ColorVariant;
  size?: 'sm' | 'md' | 'lg';
  showLabel?: boolean;
  label?: string;
}

export const ProgressBar: React.FC<ProgressBarProps> = ({
  value,
  max = 100,
  color = 'blue',
  size = 'md',
  showLabel = false,
  label,
}) => {
  const percentage = Math.min(100, Math.max(0, (value / max) * 100));
  const heightClasses = { sm: 'h-1', md: 'h-1.5', lg: 'h-2' };

  // Map color to CSS variable
  const getProgressColor = () => {
    switch (color) {
      case 'green':
      case 'emerald':
        return 'var(--color-success)';
      case 'red':
        return 'var(--color-error)';
      case 'yellow':
      case 'amber':
        return 'var(--color-warning)';
      case 'cyan':
        return 'var(--color-info)';
      default:
        return 'var(--color-accent-primary)';
    }
  };

  return (
    <div className="w-full">
      {showLabel && (
        <div
          className="flex justify-between text-xs mb-1"
          style={{ color: 'var(--color-text-muted)' }}
        >
          <span>{label}</span>
          <span>{percentage.toFixed(0)}%</span>
        </div>
      )}
      <div
        className={clsx('w-full rounded-full overflow-hidden', heightClasses[size])}
        style={{ background: 'var(--color-bg-tertiary)' }}
      >
        <div
          className="h-full rounded-full transition-all duration-300"
          style={{
            width: `${percentage}%`,
            background: getProgressColor(),
          }}
        />
      </div>
    </div>
  );
};

// ============================================================================
// Button Component - Theme Aware
// ============================================================================

interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'primary' | 'secondary' | 'ghost' | 'danger';
  size?: 'sm' | 'md' | 'lg';
  icon?: React.ReactNode;
  loading?: boolean;
}

export const Button: React.FC<ButtonProps> = ({
  children,
  variant = 'primary',
  size = 'md',
  icon,
  loading = false,
  className,
  disabled,
  ...props
}) => {
  const variantStyles = {
    primary: {
      background: 'var(--color-accent-primary)',
      color: '#ffffff',
      border: 'transparent',
    },
    secondary: {
      background: 'var(--color-bg-tertiary)',
      color: 'var(--color-text-secondary)',
      border: 'var(--color-border-subtle)',
    },
    ghost: {
      background: 'transparent',
      color: 'var(--color-text-muted)',
      border: 'transparent',
    },
    danger: {
      background: 'var(--color-error-bg)',
      color: 'var(--color-error)',
      border: 'var(--color-error-border)',
    },
  };

  const sizeClasses = {
    sm: 'px-2 py-1 text-xs gap-1',
    md: 'px-3 py-1.5 text-sm gap-1.5',
    lg: 'px-4 py-2 text-sm gap-2',
  };

  const styles = variantStyles[variant];

  return (
    <button
      className={clsx(
        'inline-flex items-center justify-center rounded-md font-medium transition-all',
        'hover:opacity-90 active:opacity-80',
        sizeClasses[size],
        (disabled || loading) && 'opacity-50 cursor-not-allowed',
        className
      )}
      style={{
        background: styles.background,
        color: styles.color,
        border: `1px solid ${styles.border}`,
      }}
      disabled={disabled || loading}
      {...props}
    >
      {loading ? <Loader2 className="w-4 h-4 animate-spin" /> : icon}
      {children}
    </button>
  );
};

// ============================================================================
// Tab Component - Theme Aware
// ============================================================================

interface TabProps {
  tabs: Array<{ id: string; label: string; icon?: React.ReactNode }>;
  activeTab: string;
  onTabChange: (tabId: string) => void;
  size?: 'sm' | 'md';
}

export const Tabs: React.FC<TabProps> = ({
  tabs,
  activeTab,
  onTabChange,
  size = 'md',
}) => {
  const sizeClasses = {
    sm: 'px-2.5 py-1.5 text-xs',
    md: 'px-3 py-2 text-sm',
  };

  return (
    <div
      className="flex gap-1 border-b overflow-x-auto"
      style={{ borderColor: 'var(--color-border-subtle)' }}
    >
      {tabs.map((tab) => (
        <button
          key={tab.id}
          onClick={() => onTabChange(tab.id)}
          className={clsx(
            'flex items-center gap-1.5 font-medium whitespace-nowrap border-b-2 -mb-px transition-colors',
            sizeClasses[size]
          )}
          style={{
            color: activeTab === tab.id ? 'var(--color-accent-primary)' : 'var(--color-text-muted)',
            borderColor: activeTab === tab.id ? 'var(--color-accent-primary)' : 'transparent',
          }}
        >
          {tab.icon}
          {tab.label}
        </button>
      ))}
    </div>
  );
};

// ============================================================================
// Log Entry Component - Theme Aware
// ============================================================================

interface LogEntryDisplayProps {
  entry: { timestamp: number; level: LogLevel; message: string; agent?: string };
}

export const LogEntryDisplay: React.FC<LogEntryDisplayProps> = ({ entry }) => {
  const levelConfig: Record<LogLevel, { color: ColorVariant; icon: LucideIcon }> = {
    info: { color: 'blue', icon: Info },
    warning: { color: 'yellow', icon: AlertTriangle },
    error: { color: 'red', icon: XCircle },
    success: { color: 'green', icon: CheckCircle },
    debug: { color: 'gray', icon: Info },
  };

  const config = levelConfig[entry.level];
  const Icon = config.icon;
  const colorClasses = COLOR_CLASSES[config.color];
  const time = new Date(entry.timestamp * 1000).toLocaleTimeString();

  return (
    <div
      className="flex items-start gap-2 px-3 py-1.5 transition-colors"
      style={{ background: 'transparent' }}
      onMouseEnter={(e) => (e.currentTarget.style.background = 'var(--color-bg-tertiary)')}
      onMouseLeave={(e) => (e.currentTarget.style.background = 'transparent')}
    >
      <Icon className={clsx('w-3.5 h-3.5 mt-0.5 flex-shrink-0', colorClasses.text)} />
      <span
        className="text-xs font-mono w-16 flex-shrink-0"
        style={{ color: 'var(--color-text-disabled)' }}
      >
        {time}
      </span>
      {entry.agent && (
        <Badge color="purple" size="xs">
          {entry.agent}
        </Badge>
      )}
      <span className="text-xs flex-1" style={{ color: 'var(--color-text-secondary)' }}>
        {entry.message}
      </span>
    </div>
  );
};

// ============================================================================
// Empty State Component - Theme Aware
// ============================================================================

interface EmptyStateProps {
  icon?: React.ReactNode;
  title: string;
  description?: string;
  action?: React.ReactNode;
}

export const EmptyState: React.FC<EmptyStateProps> = ({
  icon,
  title,
  description,
  action,
}) => {
  return (
    <div className="flex flex-col items-center justify-center py-12 px-4 text-center">
      {icon && (
        <div className="mb-3" style={{ color: 'var(--color-border-default)' }}>
          {icon}
        </div>
      )}
      <h3
        className="text-sm font-medium mb-1"
        style={{ color: 'var(--color-text-secondary)' }}
      >
        {title}
      </h3>
      {description && (
        <p
          className="text-xs mb-4 max-w-xs"
          style={{ color: 'var(--color-text-muted)' }}
        >
          {description}
        </p>
      )}
      {action}
    </div>
  );
};

// ============================================================================
// Skeleton Loader Component - Theme Aware
// ============================================================================

interface SkeletonProps {
  className?: string;
  variant?: 'text' | 'card' | 'circle';
}

export const Skeleton: React.FC<SkeletonProps> = ({
  className,
  variant = 'text',
}) => {
  const variantClasses = {
    text: 'h-4 rounded',
    card: 'h-24 rounded-lg',
    circle: 'h-10 w-10 rounded-full',
  };

  return (
    <div
      className={clsx('animate-pulse', variantClasses[variant], className)}
      style={{ background: 'var(--color-bg-tertiary)' }}
    />
  );
};
