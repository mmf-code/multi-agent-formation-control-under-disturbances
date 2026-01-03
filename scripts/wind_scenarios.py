#!/usr/bin/env python3
"""
Wind Scenario Library for Systematic Testing

Generates standard meteorological wind scenarios based on IEC 61400-1 and
MIL-F-8785C standards for systematic controller testing.

Scenarios range from calm indoor conditions to challenging outdoor gusts,
allowing reproducible comparison of PID, PD, and PID+Fuzzy controllers.

Usage:
    # Generate YAML scenario files
    python3 wind_scenarios.py --output-dir config/wind_scenarios

    # Generate scenario summary table
    python3 wind_scenarios.py --print-table

    # Export to launch file parameters
    python3 wind_scenarios.py --format launch

Author: Multi-Agent Formation Control Research
Date: 2026-01-02
"""

import os
import sys
import argparse
import yaml
from dataclasses import dataclass, asdict
from typing import Dict, List
from pathlib import Path


@dataclass
class WindScenario:
    """Wind scenario parameters for systematic testing."""
    name: str
    description: str
    U_ref: float              # Reference wind speed at 1m altitude [m/s]
    TI: float                 # Turbulence intensity [-]
    L_u: float                # Integral length scale [m]
    profile: str              # Turbulence profile: 'vonkarman' or 'dryden'
    coherence_model: str = "iec"  # Spatial coherence model
    L_c: float = 340.2        # Coherence decay constant (IEC 61400-1)

    @property
    def sigma_w(self) -> float:
        """Turbulence standard deviation [m/s]."""
        return self.TI * self.U_ref

    @property
    def category(self) -> str:
        """Scenario difficulty category."""
        if self.U_ref < 2.0 and self.TI < 0.12:
            return "CALM"
        elif self.U_ref < 4.0 and self.TI < 0.18:
            return "MODERATE"
        else:
            return "CHALLENGING"


# Standard wind scenarios for multi-agent drone formation control
WIND_SCENARIOS: Dict[str, WindScenario] = {
    # ===== Indoor / Calm Scenarios =====
    "indoor_baseline": WindScenario(
        name="indoor_baseline",
        description="Indoor conditions - baseline test with minimal disturbances",
        U_ref=0.5,      # Very light air movement (HVAC)
        TI=0.05,        # 5% turbulence intensity (smooth)
        L_u=20.0,       # Small indoor space
        profile="vonkarman"
    ),

    "calm_outdoor": WindScenario(
        name="calm_outdoor",
        description="Calm outdoor conditions - light air (Beaufort 1)",
        U_ref=1.5,      # Light breeze
        TI=0.10,        # 10% turbulence intensity
        L_u=40.0,       # Open space
        profile="vonkarman"
    ),

    # ===== Moderate Scenarios (Typical Operating Conditions) =====
    "low_wind_low_turbulence": WindScenario(
        name="low_wind_low_turbulence",
        description="Low wind, low turbulence - IEC Class A (high turbulence site)",
        U_ref=3.0,      # Light breeze (Beaufort 2)
        TI=0.10,        # 10% TI
        L_u=50.0,       # Standard length scale
        profile="vonkarman"
    ),

    "moderate_wind_moderate_turbulence": WindScenario(
        name="moderate_wind_moderate_turbulence",
        description="Normal outdoor conditions - typical small drone operations",
        U_ref=5.0,      # Gentle breeze (Beaufort 3)
        TI=0.15,        # 15% TI (moderate turbulence)
        L_u=50.0,
        profile="vonkarman"
    ),

    "moderate_wind_dryden": WindScenario(
        name="moderate_wind_dryden",
        description="Moderate wind with Dryden model - comparison test",
        U_ref=5.0,
        TI=0.15,
        L_u=50.0,
        profile="dryden"  # Alternative spectral model
    ),

    # ===== Challenging Scenarios (Stress Tests) =====
    "high_wind_moderate_turbulence": WindScenario(
        name="high_wind_moderate_turbulence",
        description="High wind, moderate turbulence - near operational limits",
        U_ref=7.0,      # Moderate breeze (Beaufort 4)
        TI=0.15,        # Still moderate TI
        L_u=45.0,
        profile="vonkarman"
    ),

    "high_wind_high_turbulence": WindScenario(
        name="high_wind_high_turbulence",
        description="Challenging conditions - stress test for controller robustness",
        U_ref=8.0,      # Fresh breeze (Beaufort 5) - max for nano drones
        TI=0.20,        # 20% TI (high turbulence)
        L_u=40.0,       # Reduced length scale (more high-freq content)
        profile="dryden"
    ),

    "extreme_gusts": WindScenario(
        name="extreme_gusts",
        description="Extreme gust scenario - maximum stress test",
        U_ref=10.0,     # Strong breeze (Beaufort 6) - beyond typical limits
        TI=0.25,        # 25% TI (very high turbulence)
        L_u=35.0,       # Small length scale (rapid fluctuations)
        profile="dryden"
    ),

    # ===== Special Test Scenarios =====
    "urban_canyon": WindScenario(
        name="urban_canyon",
        description="Urban environment - building wake effects",
        U_ref=4.0,
        TI=0.30,        # Very high turbulence (building wakes)
        L_u=25.0,       # Small length scale (confined space)
        profile="vonkarman"
    ),

    "forest_canopy": WindScenario(
        name="forest_canopy",
        description="Forest canopy - vegetation-induced turbulence",
        U_ref=3.5,
        TI=0.35,        # Extremely high turbulence (foliage)
        L_u=15.0,       # Very small length scale
        profile="vonkarman"
    ),

    # ===== Thesis Comparison Scenarios =====
    "thesis_baseline": WindScenario(
        name="thesis_baseline",
        description="Thesis baseline - standard comparison scenario",
        U_ref=3.5,      # Representative outdoor wind
        TI=0.15,        # Moderate turbulence
        L_u=50.0,
        profile="vonkarman"
    ),

    "thesis_stress": WindScenario(
        name="thesis_stress",
        description="Thesis stress test - challenging conditions",
        U_ref=6.0,
        TI=0.20,
        L_u=40.0,
        profile="dryden"
    ),
}


def generate_scenario_yaml(scenario: WindScenario, output_file: str):
    """
    Generate YAML configuration file for a wind scenario.

    Args:
        scenario: WindScenario object
        output_file: Output YAML file path
    """
    config = {
        'wind_turbulence_node': {
            'ros__parameters': {
                # Scenario metadata
                'scenario_name': scenario.name,
                'scenario_description': scenario.description,

                # Wind parameters
                'mean_wind_speed': scenario.U_ref,
                'turbulence_intensity': scenario.TI,
                'integral_length_scale_u': scenario.L_u,
                'integral_length_scale_v': scenario.L_u / 2.0,  # Typical ratio
                'integral_length_scale_w': scenario.L_u / 5.0,  # Vertical scale

                # Turbulence model selection
                'turbulence_profile': scenario.profile,  # 'vonkarman' or 'dryden'

                # Spatial coherence
                'coherence_model': scenario.coherence_model,
                'coherence_decay_constant': scenario.L_c,

                # ROS parameters
                'publish_rate_hz': 10.0,
                'frame_id': 'odom',

                # Agent positions (9-drone formation, will be updated from odometry)
                'num_agents': 9,
            }
        }
    }

    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)

    # Write YAML file
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"  Generated: {output_file}")


def generate_launch_parameters(scenario: WindScenario) -> str:
    """
    Generate launch file parameter string for a scenario.

    Args:
        scenario: WindScenario object

    Returns:
        Launch file parameter string
    """
    params = f"""
    # Wind Scenario: {scenario.name}
    # {scenario.description}
    wind_params = {{
        'mean_wind_speed': {scenario.U_ref},
        'turbulence_intensity': {scenario.TI},
        'integral_length_scale_u': {scenario.L_u},
        'integral_length_scale_v': {scenario.L_u / 2.0:.1f},
        'integral_length_scale_w': {scenario.L_u / 5.0:.1f},
        'turbulence_profile': '{scenario.profile}',
        'coherence_model': '{scenario.coherence_model}',
        'coherence_decay_constant': {scenario.L_c},
    }}
    """
    return params


def print_scenario_table():
    """Print formatted table of all wind scenarios."""
    print("\n" + "="*100)
    print("WIND SCENARIO LIBRARY - SUMMARY TABLE")
    print("="*100)
    print(f"{'Scenario Name':<30} {'Category':<12} {'U_ref':<8} {'TI':<8} {'σ_w':<8} {'L_u':<8} {'Model':<12}")
    print(f"{'':=<30} {'':=<12} {'':=<8} {'':=<8} {'':=<8} {'':=<8} {'':=<12}")

    for name, scenario in WIND_SCENARIOS.items():
        print(f"{scenario.name:<30} {scenario.category:<12} "
              f"{scenario.U_ref:<8.1f} {scenario.TI:<8.2f} "
              f"{scenario.sigma_w:<8.2f} {scenario.L_u:<8.1f} "
              f"{scenario.profile:<12}")

    print("="*100)
    print(f"\nTotal scenarios: {len(WIND_SCENARIOS)}")
    print("\nUnits:")
    print("  U_ref: Reference wind speed [m/s]")
    print("  TI:    Turbulence intensity [-]")
    print("  σ_w:   Turbulence standard deviation [m/s]")
    print("  L_u:   Integral length scale [m]")
    print("  Model: Turbulence spectral model (vonkarman/dryden)")


def print_scenario_details():
    """Print detailed information for each scenario."""
    print("\n" + "="*100)
    print("DETAILED SCENARIO DESCRIPTIONS")
    print("="*100)

    categories = {
        "CALM": [],
        "MODERATE": [],
        "CHALLENGING": []
    }

    for scenario in WIND_SCENARIOS.values():
        categories[scenario.category].append(scenario)

    for category, scenarios in categories.items():
        if scenarios:
            print(f"\n{'='*40}")
            print(f"{category} SCENARIOS ({len(scenarios)} scenarios)")
            print(f"{'='*40}")
            for scenario in scenarios:
                print(f"\n{scenario.name}:")
                print(f"  Description: {scenario.description}")
                print(f"  Wind Speed:  {scenario.U_ref} m/s")
                print(f"  Turbulence:  TI={scenario.TI:.1%}, σ_w={scenario.sigma_w:.2f} m/s")
                print(f"  Length Scale: L_u={scenario.L_u} m")
                print(f"  Model:       {scenario.profile}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Wind Scenario Library - Generate standard test scenarios"
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory for YAML scenario files'
    )
    parser.add_argument(
        '--print-table',
        action='store_true',
        help='Print scenario summary table'
    )
    parser.add_argument(
        '--print-details',
        action='store_true',
        help='Print detailed scenario descriptions'
    )
    parser.add_argument(
        '--format',
        choices=['yaml', 'launch'],
        default='yaml',
        help='Output format (default: yaml)'
    )
    parser.add_argument(
        '--scenarios',
        nargs='*',
        help='Generate specific scenarios (default: all)'
    )

    args = parser.parse_args()

    # Print table if requested
    if args.print_table:
        print_scenario_table()
        return

    # Print details if requested
    if args.print_details:
        print_scenario_details()
        return

    # Generate scenario files
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        print(f"\nGenerating wind scenario files...")
        print(f"Output directory: {output_dir}")
        print(f"Format: {args.format}")

        # Determine which scenarios to generate
        scenarios_to_generate = WIND_SCENARIOS
        if args.scenarios:
            scenarios_to_generate = {
                k: v for k, v in WIND_SCENARIOS.items()
                if k in args.scenarios
            }

        # Generate files
        for name, scenario in scenarios_to_generate.items():
            if args.format == 'yaml':
                output_file = output_dir / f"{name}.yaml"
                generate_scenario_yaml(scenario, str(output_file))
            elif args.format == 'launch':
                print(generate_launch_parameters(scenario))

        print(f"\n✓ Generated {len(scenarios_to_generate)} scenario files")
        print(f"  Location: {output_dir}")

    else:
        # No arguments - print help
        print("\nWind Scenario Library")
        print("=====================\n")
        print("Available scenarios:")
        for name in WIND_SCENARIOS.keys():
            print(f"  - {name}")
        print(f"\nTotal: {len(WIND_SCENARIOS)} scenarios")
        print("\nUsage examples:")
        print("  python3 wind_scenarios.py --print-table")
        print("  python3 wind_scenarios.py --print-details")
        print("  python3 wind_scenarios.py --output-dir config/wind_scenarios")
        print("  python3 wind_scenarios.py --output-dir /tmp/wind --scenarios thesis_baseline thesis_stress")


if __name__ == '__main__':
    main()
