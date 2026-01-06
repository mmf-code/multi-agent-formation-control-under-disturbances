#!/usr/bin/env python3
"""
Trajectory Generator for Formation Coordinator

Generates waypoints for different trajectory types:
- linear: Straight line movement (default)
- circular: Circular path around center
- figure8: Figure-8 (lemniscate) pattern
- expanding: Expanding spiral from center

Usage:
    python3 trajectory_generator.py --type circular --duration 60 --output config.yaml
    python3 trajectory_generator.py --type figure8 --radius 8 --duration 60

Output: YAML waypoint arrays for formation_coordinator config
"""

import argparse
import math
import sys
from typing import List, Tuple, Dict, Any
import yaml


def generate_linear_waypoints(
    duration: float,
    start_x: float = -5.0,
    end_x: float = 5.0,
    y: float = 0.0,
    z_base: float = 0.5,
    z_peak: float = 0.8,
    num_waypoints: int = 4
) -> Dict[str, List[float]]:
    """Generate linear (back-and-forth) trajectory waypoints."""
    times = []
    x_vals = []
    y_vals = []
    z_vals = []

    segment_time = duration / num_waypoints

    for i in range(num_waypoints):
        t = (i + 1) * segment_time
        progress = (i + 1) / num_waypoints

        # Linear interpolation for x
        x = start_x + (end_x - start_x) * progress

        # Z goes up in middle, down at ends
        if progress < 0.5:
            z = z_base + (z_peak - z_base) * (progress * 2)
        else:
            z = z_peak - (z_peak - z_base) * ((progress - 0.5) * 2)

        times.append(round(t, 1))
        x_vals.append(round(x, 2))
        y_vals.append(round(y, 2))
        z_vals.append(round(z, 2))

    return {
        'times': times,
        'x': x_vals,
        'y': y_vals,
        'z': z_vals
    }


def generate_circular_waypoints(
    duration: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    radius: float = 8.0,
    z_base: float = 0.5,
    z_amplitude: float = 0.3,
    num_waypoints: int = 12,
    start_angle: float = 0.0
) -> Dict[str, List[float]]:
    """Generate circular trajectory waypoints."""
    times = []
    x_vals = []
    y_vals = []
    z_vals = []

    for i in range(num_waypoints):
        t = (i + 1) * duration / num_waypoints
        angle = start_angle + 2 * math.pi * (i + 1) / num_waypoints

        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        # Z oscillates slightly for visual interest
        z = z_base + z_amplitude * math.sin(2 * angle)

        times.append(round(t, 1))
        x_vals.append(round(x, 2))
        y_vals.append(round(y, 2))
        z_vals.append(round(z, 2))

    return {
        'times': times,
        'x': x_vals,
        'y': y_vals,
        'z': z_vals
    }


def generate_figure8_waypoints(
    duration: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    radius: float = 8.0,
    z_base: float = 0.5,
    z_amplitude: float = 0.3,
    num_waypoints: int = 16
) -> Dict[str, List[float]]:
    """Generate figure-8 (lemniscate) trajectory waypoints."""
    times = []
    x_vals = []
    y_vals = []
    z_vals = []

    for i in range(num_waypoints):
        t = (i + 1) * duration / num_waypoints
        # Parametric lemniscate of Bernoulli
        theta = 2 * math.pi * (i + 1) / num_waypoints

        # Lemniscate: x = a*cos(t)/(1+sin^2(t)), y = a*sin(t)*cos(t)/(1+sin^2(t))
        # Simplified figure-8:
        x = center_x + radius * math.sin(theta)
        y = center_y + radius * math.sin(theta) * math.cos(theta)

        # Z varies with position
        z = z_base + z_amplitude * math.cos(2 * theta)

        times.append(round(t, 1))
        x_vals.append(round(x, 2))
        y_vals.append(round(y, 2))
        z_vals.append(round(z, 2))

    return {
        'times': times,
        'x': x_vals,
        'y': y_vals,
        'z': z_vals
    }


def generate_expanding_waypoints(
    duration: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    start_radius: float = 2.0,
    end_radius: float = 10.0,
    z_base: float = 0.5,
    z_amplitude: float = 0.4,
    num_waypoints: int = 12,
    num_rotations: float = 2.0
) -> Dict[str, List[float]]:
    """Generate expanding spiral trajectory waypoints."""
    times = []
    x_vals = []
    y_vals = []
    z_vals = []

    for i in range(num_waypoints):
        progress = (i + 1) / num_waypoints
        t = progress * duration

        # Radius increases linearly
        r = start_radius + (end_radius - start_radius) * progress

        # Angle increases for spiral effect
        angle = 2 * math.pi * num_rotations * progress

        x = center_x + r * math.cos(angle)
        y = center_y + r * math.sin(angle)
        # Z increases with expansion
        z = z_base + z_amplitude * progress

        times.append(round(t, 1))
        x_vals.append(round(x, 2))
        y_vals.append(round(y, 2))
        z_vals.append(round(z, 2))

    return {
        'times': times,
        'x': x_vals,
        'y': y_vals,
        'z': z_vals
    }


def generate_shape_sequence(
    num_waypoints: int,
    shapes: List[str] = None
) -> List[str]:
    """Generate formation shape sequence for visual variety."""
    if shapes is None:
        # Default: cycle through shapes
        available = ['triangle', 'line', 'v_shape', 'triangle']
        shapes = []
        for i in range(num_waypoints):
            shapes.append(available[i % len(available)])
    return shapes


def create_formation_config(
    group_name: str,
    agent_ids: List[str],
    waypoints: Dict[str, List[float]],
    base_y: float,
    shapes: List[str] = None,
    spacing: float = 3.0
) -> Dict[str, Any]:
    """Create full formation coordinator config for a group."""

    config = {
        f'/{group_name}/formation_coordinator': {
            'ros__parameters': {
                'agent_ids': agent_ids,
                'formation.shape': 'triangle',
                'formation.spacing': spacing,
                'formation.center.x': 0.0,
                'formation.center.y': base_y,
                'formation.center.z': 0.5,
                'formation.yaw_deg': 0.0,
                'frame_id': 'odom',
                'publish_frequency_hz': 10.0,
                'pso.enable': False,
                'motion.enable': False,
                'waypoints.enable': True,
                'waypoints.times': waypoints['times'],
                'waypoints.x': waypoints['x'],
                'waypoints.y': [base_y] * len(waypoints['y']),  # Keep Y constant per group
                'waypoints.z': waypoints['z'],
                'etc.enable': False,
            }
        }
    }

    if shapes:
        config[f'/{group_name}/formation_coordinator']['ros__parameters']['waypoints.shapes'] = shapes

    return config


def print_waypoints_table(waypoints: Dict[str, List[float]], trajectory_type: str):
    """Print waypoints in a nice table format."""
    print(f"\n{'='*60}")
    print(f"Trajectory: {trajectory_type.upper()}")
    print(f"{'='*60}")
    print(f"{'WP':<4} {'Time':<8} {'X':<10} {'Y':<10} {'Z':<10}")
    print(f"{'-'*60}")

    for i in range(len(waypoints['times'])):
        print(f"{i:<4} {waypoints['times'][i]:<8.1f} {waypoints['x'][i]:<10.2f} {waypoints['y'][i]:<10.2f} {waypoints['z'][i]:<10.2f}")

    print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description='Generate trajectory waypoints for formation coordinator')
    parser.add_argument('--type', choices=['linear', 'circular', 'figure8', 'expanding'],
                        default='circular', help='Trajectory type')
    parser.add_argument('--duration', type=float, default=60.0, help='Total duration in seconds')
    parser.add_argument('--radius', type=float, default=8.0, help='Radius for circular/figure8')
    parser.add_argument('--num-waypoints', type=int, default=12, help='Number of waypoints')
    parser.add_argument('--center-x', type=float, default=0.0, help='Center X coordinate')
    parser.add_argument('--center-y', type=float, default=0.0, help='Center Y coordinate')
    parser.add_argument('--z-base', type=float, default=0.5, help='Base altitude')
    parser.add_argument('--output', type=str, help='Output YAML file path')
    parser.add_argument('--print-only', action='store_true', help='Only print, do not write file')
    parser.add_argument('--with-shapes', action='store_true', help='Include shape transitions')

    args = parser.parse_args()

    # Generate waypoints based on type
    if args.type == 'linear':
        waypoints = generate_linear_waypoints(
            duration=args.duration,
            start_x=-args.radius,
            end_x=args.radius,
            y=args.center_y,
            z_base=args.z_base,
            num_waypoints=args.num_waypoints
        )
    elif args.type == 'circular':
        waypoints = generate_circular_waypoints(
            duration=args.duration,
            center_x=args.center_x,
            center_y=args.center_y,
            radius=args.radius,
            z_base=args.z_base,
            num_waypoints=args.num_waypoints
        )
    elif args.type == 'figure8':
        waypoints = generate_figure8_waypoints(
            duration=args.duration,
            center_x=args.center_x,
            center_y=args.center_y,
            radius=args.radius,
            z_base=args.z_base,
            num_waypoints=args.num_waypoints
        )
    elif args.type == 'expanding':
        waypoints = generate_expanding_waypoints(
            duration=args.duration,
            center_x=args.center_x,
            center_y=args.center_y,
            start_radius=args.radius / 4,
            end_radius=args.radius,
            z_base=args.z_base,
            num_waypoints=args.num_waypoints
        )

    # Print table
    print_waypoints_table(waypoints, args.type)

    # Generate shapes if requested
    shapes = None
    if args.with_shapes:
        shapes = generate_shape_sequence(len(waypoints['times']))
        print(f"Shapes: {shapes}")

    # Print YAML snippet for copy-paste
    print("\nYAML snippet for config:")
    print("-" * 40)
    yaml_snippet = {
        'waypoints.enable': True,
        'waypoints.times': waypoints['times'],
        'waypoints.x': waypoints['x'],
        'waypoints.y': waypoints['y'],
        'waypoints.z': waypoints['z'],
    }
    if shapes:
        yaml_snippet['waypoints.shapes'] = shapes

    print(yaml.dump(yaml_snippet, default_flow_style=False))

    # Write to file if requested
    if args.output and not args.print_only:
        with open(args.output, 'w') as f:
            yaml.dump(yaml_snippet, f, default_flow_style=False)
        print(f"\nWritten to: {args.output}")

    return waypoints


if __name__ == '__main__':
    main()
