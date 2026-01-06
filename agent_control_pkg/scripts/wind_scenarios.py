#!/usr/bin/env python3
"""
Wind Scenarios Module for 6-Phase Thesis Test Framework

Provides standardized wind configurations for each test phase:
  Phase 1: BASELINE      - No wind (reference performance)
  Phase 2: STEADY_WIND   - Constant wind (DC rejection)
  Phase 3: TURBULENCE    - Von Karman with TI sweep [0.15, 0.25, 0.35]
  Phase 4: GUST          - Periodic gusts (transient response)
  Phase 5: COMBINED      - Turbulence + gusts + direction wander
  Phase 6: ENDURANCE     - Long-run combined (300s)

Usage:
    from wind_scenarios import get_phase_config, PHASE_DEFINITIONS
    config = get_phase_config(phase_id=3, sweep_index=1)  # TI=0.25
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List


@dataclass
class PhaseDefinition:
    """Definition of a test phase with wind configuration."""
    phase_id: int
    name: str
    purpose: str
    duration_sec: int
    wind_config: Dict[str, Any]
    sweep_values: Optional[List[float]] = None  # For Phase 3 TI sweep
    expectations: List[str] = field(default_factory=list)


# Phase definitions following thesis test framework
PHASE_DEFINITIONS: Dict[int, PhaseDefinition] = {
    1: PhaseDefinition(
        phase_id=1,
        name="BASELINE",
        purpose="Reference performance without wind; sanity check metrics",
        duration_sec=60,
        wind_config={
            "profile": "constant",
            "magnitude": 0.0,
            "direction": 0.0,
            "publish_rate": 10.0,
        },
        expectations=[
            "Controllers should be close in performance",
            "PD may be slightly worse but not catastrophically",
            "Establish baseline RMSE/ITAE/control-effort",
        ],
    ),
    2: PhaseDefinition(
        phase_id=2,
        name="STEADY_WIND",
        purpose="DC offset rejection; integral action sensitivity",
        duration_sec=60,
        wind_config={
            "profile": "constant",
            "magnitude": 3.0,
            "direction": 45.0,
            "publish_rate": 10.0,
        },
        expectations=[
            "PID > PD due to integral action",
            "Fuzzy hybrids should not regress",
            "Reduced steady-state bias with fuzzy",
        ],
    ),
    3: PhaseDefinition(
        phase_id=3,
        name="TURBULENCE",
        purpose="High chaos stochastic uncertainty; fuzzy advantage demonstration",
        duration_sec=60,
        wind_config={
            "profile": "vonkarman",
            "magnitude": 4.0,  # Increased from 2.5 for dramatic effect
            "direction": 45.0,
            "mean_wind_speed": 4.0,  # Match magnitude
            "turbulence_intensity": 0.55,  # Default, overridden by sweep
            "integral_length_u": 30.0,
            "integral_length_v": 15.0,
            "integral_length_w": 5.0,
            "publish_rate": 20.0,
        },
        sweep_values=[0.45, 0.55, 0.65],  # High TI sweep for chaos
        expectations=[
            "Strong performance separation between controllers",
            "PID/PD struggle with high-frequency variations",
            "GT2 > IT2 > PID > PD clearly visible",
        ],
    ),
    4: PhaseDefinition(
        phase_id=4,
        name="GUST",
        purpose="Transient response and recovery; robustness to intermittent disturbances",
        duration_sec=60,
        wind_config={
            "profile": "gust",
            "magnitude": 5.0,
            "direction": 45.0,
            "gust_duration": 1.5,
            "gust_interval": 10.0,
            "publish_rate": 20.0,
        },
        expectations=[
            "Fuzzy hybrids should recover faster",
            "Reduced peak deviation vs PID/PD",
            "Overshoot and settling metrics primary",
        ],
    ),
    5: PhaseDefinition(
        phase_id=5,
        name="COMBINED",
        purpose="Realistic atmospheric: mean wind + turbulence + gusts + direction wander",
        duration_sec=60,
        wind_config={
            "profile": "stochastic",  # Uses stochastic for combined behavior
            "magnitude": 2.5,
            "direction": 45.0,
            "stochastic_mag_std": 1.5,
            "stochastic_dir_rate": 15.0,  # deg/s direction wander
            "stochastic_gust_prob": 0.08,  # ~1 gust every 12s
            "stochastic_gust_mag": 2.0,  # 2x magnitude during gust
            "stochastic_turbulence": 0.5,  # High-freq overlay
            "publish_rate": 20.0,
        },
        expectations=[
            "GT2 should be most robust if secondary MF is effective",
            "Control effort vs error tradeoff must be reported",
        ],
    ),
    6: PhaseDefinition(
        phase_id=6,
        name="ENDURANCE",
        purpose="Long-run stability and cumulative metrics",
        duration_sec=300,  # 5 minutes
        wind_config={
            "profile": "stochastic",
            "magnitude": 2.5,
            "direction": 45.0,
            "stochastic_mag_std": 1.5,
            "stochastic_dir_rate": 15.0,
            "stochastic_gust_prob": 0.08,
            "stochastic_gust_mag": 2.0,
            "stochastic_turbulence": 0.5,
            "publish_rate": 20.0,
        },
        expectations=[
            "Stable tracking without divergence",
            "Meaningful cumulative comparisons (ITAE, effort)",
        ],
    ),
}


def get_phase_config(
    phase_id: int,
    sweep_index: Optional[int] = None,
    random_seed: int = -1,
) -> Dict[str, Any]:
    """
    Get wind publisher parameters for a specific phase.

    Args:
        phase_id: Phase number (1-6)
        sweep_index: For Phase 3, index into TI sweep [0, 1, 2] for [0.15, 0.25, 0.35]
        random_seed: Random seed for reproducibility (-1 for random)

    Returns:
        Dictionary of ROS2 parameters for wind_publisher node

    Raises:
        ValueError: If phase_id is invalid or sweep_index out of range
    """
    if phase_id not in PHASE_DEFINITIONS:
        raise ValueError(f"Invalid phase_id: {phase_id}. Must be 1-6.")

    phase = PHASE_DEFINITIONS[phase_id]
    config = dict(phase.wind_config)  # Copy to avoid mutation

    # Handle Phase 3 TI sweep
    if phase_id == 3 and phase.sweep_values:
        if sweep_index is None:
            sweep_index = 0  # Default to first value
        if sweep_index < 0 or sweep_index >= len(phase.sweep_values):
            raise ValueError(
                f"sweep_index {sweep_index} out of range for Phase 3. "
                f"Valid: 0-{len(phase.sweep_values) - 1}"
            )
        config["turbulence_intensity"] = phase.sweep_values[sweep_index]

    # Add random seed for reproducibility
    config["random_seed"] = random_seed

    return config


def get_phase_duration(phase_id: int) -> int:
    """Get the duration in seconds for a specific phase."""
    if phase_id not in PHASE_DEFINITIONS:
        raise ValueError(f"Invalid phase_id: {phase_id}. Must be 1-6.")
    return PHASE_DEFINITIONS[phase_id].duration_sec


def get_phase_name(phase_id: int) -> str:
    """Get the name of a specific phase."""
    if phase_id not in PHASE_DEFINITIONS:
        raise ValueError(f"Invalid phase_id: {phase_id}. Must be 1-6.")
    return PHASE_DEFINITIONS[phase_id].name


def get_sweep_count(phase_id: int) -> int:
    """Get number of sweep iterations for a phase (1 if no sweep)."""
    if phase_id not in PHASE_DEFINITIONS:
        raise ValueError(f"Invalid phase_id: {phase_id}. Must be 1-6.")
    phase = PHASE_DEFINITIONS[phase_id]
    return len(phase.sweep_values) if phase.sweep_values else 1


def get_all_phase_configs(random_seed: int = -1) -> Dict[str, Dict[str, Any]]:
    """
    Get all phase configurations for a full test run.

    Returns:
        Dictionary mapping phase keys to configurations:
        {
            "phase_1": {...},
            "phase_2": {...},
            "phase_3_ti015": {...},
            "phase_3_ti025": {...},
            "phase_3_ti035": {...},
            "phase_4": {...},
            "phase_5": {...},
            "phase_6": {...},
        }
    """
    configs = {}

    for phase_id, phase in PHASE_DEFINITIONS.items():
        if phase.sweep_values:
            # Expand sweep phases
            for i, val in enumerate(phase.sweep_values):
                key = f"phase_{phase_id}_ti{int(val * 100):03d}"
                configs[key] = {
                    "phase_id": phase_id,
                    "sweep_index": i,
                    "sweep_value": val,
                    "name": f"{phase.name}_TI{int(val * 100)}",
                    "duration_sec": phase.duration_sec,
                    "wind_config": get_phase_config(phase_id, i, random_seed),
                }
        else:
            key = f"phase_{phase_id}"
            configs[key] = {
                "phase_id": phase_id,
                "sweep_index": None,
                "sweep_value": None,
                "name": phase.name,
                "duration_sec": phase.duration_sec,
                "wind_config": get_phase_config(phase_id, None, random_seed),
            }

    return configs


def print_phase_summary():
    """Print a summary of all phases for documentation."""
    print("=" * 70)
    print("THESIS PHASED TEST FRAMEWORK - Wind Scenarios")
    print("=" * 70)

    for phase_id, phase in PHASE_DEFINITIONS.items():
        print(f"\nPhase {phase_id}: {phase.name}")
        print(f"  Purpose: {phase.purpose}")
        print(f"  Duration: {phase.duration_sec}s")
        print(f"  Profile: {phase.wind_config.get('profile', 'N/A')}")

        if phase.sweep_values:
            print(f"  TI Sweep: {phase.sweep_values}")

        print("  Expectations:")
        for exp in phase.expectations:
            print(f"    - {exp}")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    # Demo: print all phase configurations
    print_phase_summary()

    print("\n\nExample: Phase 3 configurations (TI sweep)")
    for i in range(3):
        cfg = get_phase_config(3, sweep_index=i)
        print(f"  TI={cfg['turbulence_intensity']}: {cfg}")
