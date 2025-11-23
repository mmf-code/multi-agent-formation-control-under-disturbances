#!/usr/bin/env python3
"""
Formation Comparison Demo - Metrics Recorder
Records metrics from 3 representative agents (PID+Fuzzy, PD, PID) to CSV
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import sys
import os
from datetime import datetime

class MetricsRecorder(Node):
    def __init__(self, output_dir, duration=60):
        super().__init__('metrics_recorder')
        
        self.output_dir = output_dir
        self.duration = duration
        self.start_time = self.get_clock().now()
        
        # CSV files
        self.csv_files = {
            'agent_0': os.path.join(output_dir, 'agent_0_pidfuzzy.csv'),
            'agent_3': os.path.join(output_dir, 'agent_3_pd.csv'),
            'agent_6': os.path.join(output_dir, 'agent_6_pid.csv'),
        }
        
        # Initialize CSV files
        for agent, filepath in self.csv_files.items():
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'elapsed_time',
                    'error_x', 'error_y', 'error_z', 'error_magnitude',
                    'rmse_x', 'rmse_y', 'rmse_z', 'rmse_total',
                    'iae_x', 'iae_y', 'settling_time', 'is_settled'
                ])
        
        # Subscribe to metrics topics
        # Note: We'll use a custom message listener since we don't know the exact message type
        # Let's try with topic echo approach
        
        self.get_logger().info(f'Metrics Recorder initialized')
        self.get_logger().info(f'Output directory: {output_dir}')
        self.get_logger().info(f'Recording for {duration} seconds...')
        
        # Create timer for periodic recording
        self.timer = self.create_timer(2.0, self.record_callback)
        self.sample_count = 0
        
    def record_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed >= self.duration:
            self.get_logger().info(f'Recording complete! {self.sample_count} samples collected')
            self.cleanup()
            rclpy.shutdown()
            return
        
        # Record metrics from each agent
        # This is a placeholder - we'll use subprocess to call ros2 topic echo
        import subprocess
        
        for agent_id, csv_path in self.csv_files.items():
            try:
                # Get metrics using ros2 topic echo
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', f'/{agent_id}/metrics', '--once'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                
                if result.returncode == 0:
                    # Parse the output
                    data = self.parse_metrics(result.stdout, elapsed)
                    if data:
                        with open(csv_path, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(data)
                
            except Exception as e:
                self.get_logger().warn(f'Failed to record {agent_id}: {e}')
        
        self.sample_count += 1
        progress = int(elapsed / self.duration * 100)
        self.get_logger().info(f'Progress: {progress}% | Sample {self.sample_count} | {int(self.duration - elapsed)}s remaining')
    
    def parse_metrics(self, output, elapsed):
        """Parse ros2 topic echo output"""
        lines = output.split('\n')
        data = {'elapsed': elapsed}
        
        for line in lines:
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                # Parse numeric values
                if key in ['error_x', 'error_y', 'error_z', 'error_magnitude',
                          'rmse_x', 'rmse_y', 'rmse_z', 'rmse_total',
                          'iae_x', 'iae_y', 'settling_time']:
                    try:
                        data[key] = float(value)
                    except:
                        data[key] = 0.0
                elif key == 'is_settled':
                    data[key] = 1 if 'true' in value.lower() else 0
        
        # Return row if we have data
        if len(data) > 2:
            timestamp = datetime.now().timestamp()
            return [
                timestamp, data.get('elapsed', 0),
                data.get('error_x', 0), data.get('error_y', 0), data.get('error_z', 0),
                data.get('error_magnitude', 0),
                data.get('rmse_x', 0), data.get('rmse_y', 0), data.get('rmse_z', 0),
                data.get('rmse_total', 0),
                data.get('iae_x', 0), data.get('iae_y', 0),
                data.get('settling_time', 0), data.get('is_settled', 0)
            ]
        return None
    
    def cleanup(self):
        self.get_logger().info('Cleaning up...')
        for agent, filepath in self.csv_files.items():
            if os.path.exists(filepath):
                with open(filepath, 'r') as f:
                    lines = len(f.readlines()) - 1  # Subtract header
                self.get_logger().info(f'{agent}: {lines} samples saved')

def main():
    if len(sys.argv) < 2:
        print("Usage: record_formation_metrics.py <output_directory> [duration_seconds]")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    
    os.makedirs(output_dir, exist_ok=True)
    
    rclpy.init()
    recorder = MetricsRecorder(output_dir, duration)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.cleanup()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
