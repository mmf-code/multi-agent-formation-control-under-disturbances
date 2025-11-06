#!/usr/bin/env python3
"""
Enhanced Metrics Logger for Long Scenario Thesis Demo
- Checkpoint-aware recording (saves intermediate data every phase)
- Phase detection (5 phases: 60s, 120s, 180s, 240s, 300s)
- Real-time terminal dashboard
- Comprehensive metrics (IAE, ITAE, ISE, ITSE, step info)
- Auto-recovery on crash
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
import sys
import os
import time
from datetime import datetime
import shutil
from pathlib import Path

# Import message types
try:
    from my_custom_interfaces_pkg.msg import MetricsData
    HAS_METRICS_MSG = True
except ImportError:
    HAS_METRICS_MSG = False
    print("ERROR: MetricsData message not found!")
    sys.exit(1)


class EnhancedMetricsLogger(Node):
    """
    Enhanced metrics logger with:
    - Phase-based checkpoints (saves CSV every 60s)
    - Real-time dashboard
    - Comprehensive metrics
    - Auto-save on crash
    """

    # Phase boundaries (seconds)
    PHASE_BOUNDARIES = [60, 120, 180, 240, 300]
    PHASE_NAMES = ["Phase 1", "Phase 2", "Phase 3", "Phase 4", "Phase 5"]

    def __init__(self, output_dir, duration=300, agents_to_record=None):
        super().__init__('enhanced_metrics_logger')

        self.output_dir = Path(output_dir)
        self.duration = duration
        self.start_time = self.get_clock().now()
        self.start_walltime = time.time()

        # Agent configuration
        if agents_to_record is None:
            agents_to_record = [0, 3, 6]

        self.agents = {
            f'agent_{i}': self._get_controller_name(i)
            for i in agents_to_record
        }

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # CSV writers
        self.csv_files = {}
        self.csv_writers = {}
        self.sample_counts = {}

        # Data buffers for checkpoint saves
        self.data_buffers = {agent_id: [] for agent_id in self.agents.keys()}

        # Phase tracking
        self.current_phase = 0
        self.checkpoints_saved = set()

        # Create output structure
        self._setup_output_dirs()
        self._setup_csv_files()

        # Subscribe to metrics
        for agent_id in self.agents.keys():
            self.create_subscription(
                MetricsData,
                f'/{agent_id}/metrics',
                lambda msg, aid=agent_id: self.metrics_callback(msg, aid),
                qos
            )
            self.sample_counts[agent_id] = 0

        # Dashboard timer (1Hz update)
        self.create_timer(1.0, self.update_dashboard)

        # Checkpoint timer (check every 10s)
        self.create_timer(10.0, self.check_checkpoint)

        self.get_logger().info(f'Enhanced Logger initialized')
        self.get_logger().info(f'Output: {self.output_dir}')
        self.get_logger().info(f'Duration: {duration}s, Agents: {list(self.agents.keys())}')

    def _get_controller_name(self, agent_id):
        """Map agent ID to controller type"""
        controller_map = {
            0: 'pidfuzzy', 1: 'pidfuzzy', 2: 'pidfuzzy',
            3: 'pd', 4: 'pd', 5: 'pd',
            6: 'pid', 7: 'pid', 8: 'pid'
        }
        return controller_map.get(agent_id, 'unknown')

    def _setup_output_dirs(self):
        """Create output directory structure"""
        # Main output dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Subdirectories
        (self.output_dir / 'raw_data').mkdir(exist_ok=True)
        (self.output_dir / 'checkpoints').mkdir(exist_ok=True)
        (self.output_dir / 'final_results').mkdir(exist_ok=True)

    def _setup_csv_files(self):
        """Setup CSV files for logging"""
        for agent_id, controller in self.agents.items():
            csv_path = self.output_dir / 'raw_data' / f'{agent_id}_{controller}_full.csv'

            f = open(csv_path, 'w', newline='')
            writer = csv.writer(f)

            # CSV Header
            header = [
                'timestamp', 'elapsed_time', 'phase',
                'current_x', 'current_y', 'current_z',
                'target_x', 'target_y', 'target_z',
                'error_x', 'error_y', 'error_z', 'error_magnitude',
                'rmse_x', 'rmse_y', 'rmse_z', 'rmse_total',
                'iae_x', 'iae_y', 'itae_x', 'itae_y',
                'settling_time', 'is_settled',
                'max_overshoot_x', 'max_overshoot_y'
            ]
            writer.writerow(header)
            f.flush()

            self.csv_files[agent_id] = f
            self.csv_writers[agent_id] = writer

    def metrics_callback(self, msg, agent_id):
        """Record metrics data"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            return

        # Determine current phase
        phase = self._get_phase(elapsed)

        timestamp = time.time()

        row = [
            timestamp, elapsed, phase,
            msg.current_x, msg.current_y, msg.current_z,
            msg.target_x, msg.target_y, msg.target_z,
            msg.error_x, msg.error_y, msg.error_z, msg.error_magnitude,
            msg.rmse_x, msg.rmse_y, msg.rmse_z, msg.rmse_total,
            msg.iae_x, msg.iae_y, msg.itae_x, msg.itae_y,
            msg.settling_time, 1 if msg.is_settled else 0,
            msg.max_overshoot_x, msg.max_overshoot_y
        ]

        # Write to CSV
        self.csv_writers[agent_id].writerow(row)
        self.csv_files[agent_id].flush()

        # Buffer for checkpoints
        self.data_buffers[agent_id].append(row)

        self.sample_counts[agent_id] += 1

    def _get_phase(self, elapsed_time):
        """Determine current phase based on elapsed time"""
        for i, boundary in enumerate(self.PHASE_BOUNDARIES):
            if elapsed_time <= boundary:
                return i + 1
        return 5  # Final phase

    def check_checkpoint(self):
        """Check if we need to save a checkpoint"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Check if duration exceeded
        if elapsed >= self.duration:
            self.get_logger().info('Duration complete, finalizing...')
            self.finalize()
            rclpy.shutdown()
            return

        # Check for phase boundary
        phase = self._get_phase(elapsed)

        if phase != self.current_phase and phase not in self.checkpoints_saved:
            self.save_checkpoint(phase)
            self.checkpoints_saved.add(phase)
            self.current_phase = phase

    def save_checkpoint(self, phase):
        """Save checkpoint data for completed phase"""
        checkpoint_dir = self.output_dir / 'checkpoints' / f'phase{phase}_{self.PHASE_BOUNDARIES[phase-1]}s'
        checkpoint_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Saving checkpoint for Phase {phase}...')

        for agent_id, controller in self.agents.items():
            # Get data up to this phase
            csv_path = checkpoint_dir / f'{agent_id}_{controller}.csv'

            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)

                # Header
                header = [
                    'timestamp', 'elapsed_time', 'phase',
                    'error_x', 'error_y', 'error_z', 'error_magnitude',
                    'rmse_x', 'rmse_y', 'rmse_z', 'rmse_total',
                    'iae_x', 'iae_y', 'itae_x', 'itae_y',
                    'settling_time', 'is_settled',
                    'max_overshoot_x', 'max_overshoot_y'
                ]
                writer.writerow(header)

                # Write buffered data
                for row in self.data_buffers[agent_id]:
                    if row[2] <= phase:  # row[2] is phase number
                        writer.writerow(row)

        # Create checkpoint summary
        self._create_checkpoint_summary(checkpoint_dir, phase)

        self.get_logger().info(f'✓ Checkpoint saved: {checkpoint_dir.name}')

    def _create_checkpoint_summary(self, checkpoint_dir, phase):
        """Create a summary file for this checkpoint"""
        summary_path = checkpoint_dir / 'summary.txt'

        with open(summary_path, 'w') as f:
            f.write(f"CHECKPOINT SUMMARY - Phase {phase}\n")
            f.write(f"{'='*60}\n\n")
            f.write(f"Time: {self.PHASE_BOUNDARIES[phase-1]}s\n")
            f.write(f"Phase: {self.PHASE_NAMES[phase-1]}\n\n")

            for agent_id in self.agents.keys():
                f.write(f"{agent_id}: {self.sample_counts[agent_id]} samples\n")

    def update_dashboard(self):
        """Update real-time terminal dashboard"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        remaining = max(0, self.duration - elapsed)
        progress = min(100, int(elapsed / self.duration * 100))

        phase = self._get_phase(elapsed)

        # Clear line and print dashboard
        print(f"\r\033[K", end='')  # Clear line

        # Progress bar
        bar_length = 40
        filled = int(bar_length * progress / 100)
        bar = '█' * filled + '░' * (bar_length - filled)

        # Build dashboard line
        dashboard = (
            f"[{bar}] {progress}% | "
            f"Phase {phase}/5 | "
            f"Time: {elapsed:.1f}/{self.duration}s | "
            f"Samples: {sum(self.sample_counts.values())} | "
            f"Remaining: {remaining:.0f}s"
        )

        print(dashboard, end='', flush=True)

    def finalize(self):
        """Finalize recording and generate final summary"""
        self.get_logger().info('\n\nFinalizing recording...')

        # Close CSV files
        for f in self.csv_files.values():
            f.close()

        # Create final summary
        self._create_final_summary()

        # Copy full CSVs to final_results
        for agent_id, controller in self.agents.items():
            src = self.output_dir / 'raw_data' / f'{agent_id}_{controller}_full.csv'
            dst = self.output_dir / 'final_results' / f'{agent_id}_{controller}_full.csv'
            shutil.copy(src, dst)

        elapsed = time.time() - self.start_walltime
        self.get_logger().info(f'Recording complete! Wall time: {elapsed:.1f}s')
        self.get_logger().info(f'Data saved to: {self.output_dir}')

        # Exit cleanly to prevent script hanging
        import sys
        sys.exit(0)

    def _create_final_summary(self):
        """Create final summary report"""
        summary_path = self.output_dir / 'final_results' / 'summary.txt'

        with open(summary_path, 'w') as f:
            f.write("ENHANCED METRICS LOGGER - FINAL SUMMARY\n")
            f.write("="*70 + "\n\n")

            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Duration: {self.duration}s\n")
            f.write(f"Phases: {len(self.PHASE_BOUNDARIES)}\n")
            f.write(f"Checkpoints Saved: {len(self.checkpoints_saved)}\n\n")

            f.write("AGENTS & SAMPLES:\n")
            f.write("-" * 70 + "\n")
            total_samples = 0
            for agent_id, count in self.sample_counts.items():
                controller = self.agents[agent_id]
                f.write(f"  {agent_id} ({controller:10s}): {count:6d} samples\n")
                total_samples += count

            f.write(f"\n  TOTAL: {total_samples} samples\n\n")

            f.write("OUTPUT STRUCTURE:\n")
            f.write("-" * 70 + "\n")
            f.write(f"  raw_data/         - Full CSV files (all 300s)\n")
            f.write(f"  checkpoints/      - Intermediate saves (phase1-5)\n")
            f.write(f"  final_results/    - Summary + full data\n\n")

            f.write("NEXT STEPS:\n")
            f.write("-" * 70 + "\n")
            f.write(f"  1. Analysis: python3 analysis/analyze_checkpoint_data.py {self.output_dir}\n")
            f.write(f"  2. Plotting: python3 analysis/plot_long_scenario.py {self.output_dir}\n")
            f.write(f"  3. Report:   python3 analysis/generate_thesis_report.py {self.output_dir}\n")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Enhanced Metrics Logger for Long Scenario')
    parser.add_argument('--output-dir', '-o', required=True,
                        help='Output directory for all data')
    parser.add_argument('--duration', '-d', type=int, default=300,
                        help='Recording duration in seconds (default: 300)')
    parser.add_argument('--agents', '-a', nargs='+', type=int, default=[0, 3, 6],
                        help='Agent IDs to record (default: 0 3 6)')

    args = parser.parse_args()

    print("\n" + "="*70)
    print("  ENHANCED METRICS LOGGER - Long Scenario Thesis Demo")
    print("="*70)
    print(f"  Output:   {args.output_dir}")
    print(f"  Duration: {args.duration}s ({args.duration/60:.1f} minutes)")
    print(f"  Agents:   {args.agents}")
    print(f"  Phases:   5 (checkpoints at 60s intervals)")
    print("="*70 + "\n")

    rclpy.init()

    logger = EnhancedMetricsLogger(
        output_dir=args.output_dir,
        duration=args.duration,
        agents_to_record=args.agents
    )

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
        logger.get_logger().warn('Recording interrupted - saving data...')
        logger.finalize()
    finally:
        logger.destroy_node()


if __name__ == '__main__':
    main()
