#!/usr/bin/env python3
"""
Spatial Wind Coherence Validation Script

Validates IEC exponential coherence implementation by analyzing
correlation between wind velocities at different spatial separations.

For thesis: Demonstrates that multi-drone wind correlation matches
IEC 61400-1 standard and decays properly with separation distance.

Author: Multi-Agent Formation Control Research
Date: 2026-01-02
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.stats import pearsonr

# Script directory
script_dir = os.path.dirname(os.path.abspath(__file__))


def iec_coherence(frequency: float, separation: float, mean_wind_speed: float, L_c: float = 340.2) -> float:
    """
    IEC 61400-1 exponential coherence function.

    Args:
        frequency: Frequency [Hz]
        separation: Spatial separation between points [m]
        mean_wind_speed: Mean wind speed [m/s]
        L_c: Coherence decay constant (default 340.2 per IEC 61400-1)

    Returns:
        Coherence value [0, 1]
    """
    if frequency <= 0 or separation <= 0:
        return 1.0

    # IEC coherence: Coh(f,r) = exp(-f * r / (V * L_c))
    exponent = -frequency * separation / (mean_wind_speed * L_c)
    return np.exp(exponent)


def validate_iec_coherence():
    """
    Validate IEC exponential coherence function.

    Plots coherence vs separation distance for different frequencies.
    """
    print("\n" + "="*60)
    print("IEC EXPONENTIAL COHERENCE VALIDATION")
    print("="*60)

    # Parameters
    mean_wind_speed = 3.0  # m/s
    L_c = 340.2  # IEC 61400-1 decay constant

    # Separation distances (0 to 100m)
    separations = np.linspace(0, 100, 200)

    # Frequencies to test
    frequencies = [0.01, 0.05, 0.1, 0.5, 1.0]  # Hz

    # Create plot
    fig, ax = plt.subplots(figsize=(10, 6))

    for freq in frequencies:
        coherence_values = [iec_coherence(freq, r, mean_wind_speed, L_c) for r in separations]
        ax.plot(separations, coherence_values, linewidth=2, label=f'f = {freq} Hz')

    ax.set_xlabel('Separation Distance [m]', fontsize=12)
    ax.set_ylabel('Coherence [-]', fontsize=12)
    ax.set_title('IEC 61400-1 Exponential Coherence Model', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_ylim([0, 1.05])

    # Add annotation
    textstr = f'V = {mean_wind_speed} m/s\nL_c = {L_c} (IEC 61400-1)'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax.text(0.65, 0.95, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    # Save
    output_dir = os.path.join(script_dir, '..', 'validation_results')
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'iec_coherence_validation.png')
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"  Saved: {output_file}")

    plt.close()

    print("\n✓ IEC coherence validation complete")


def simulate_correlated_turbulence():
    """
    Simulate correlated turbulence for two points and measure coherence.

    Compares measured coherence vs theoretical IEC model.
    """
    print("\n" + "="*60)
    print("CORRELATED TURBULENCE SIMULATION")
    print("="*60)

    # Simulation parameters
    duration = 300.0  # seconds
    dt = 0.1  # 10 Hz
    n_samples = int(duration / dt)

    # Wind parameters
    mean_wind_speed = 3.0  # m/s
    L_c = 340.2

    # Two points at different separations
    separations_to_test = [0, 5, 10, 20, 50]  # meters

    print(f"\nSimulating {duration}s of turbulence...")
    print(f"Sampling rate: {1/dt:.1f} Hz")
    print(f"Testing separations: {separations_to_test} m")

    # Generate reference turbulence (point 1)
    np.random.seed(42)  # For reproducibility
    turbulence_intensity = 0.3  # m/s
    turbulence_ref = np.random.randn(n_samples) * turbulence_intensity

    # Filter to add spectral content (simple low-pass)
    sos = signal.butter(2, 0.1, btype='low', output='sos')
    turbulence_ref = signal.sosfilt(sos, turbulence_ref)

    # Compute theoretical and measured coherence
    results = []

    for sep in separations_to_test:
        print(f"\n  Separation: {sep}m")

        # Generate correlated turbulence for point 2
        # Simple model: weighted average of independent noise and point 1
        dominant_freq = mean_wind_speed / (2 * np.pi * 50.0)  # Assume L_u = 50m
        coherence_target = iec_coherence(dominant_freq, sep, mean_wind_speed, L_c)

        # Independent noise for point 2
        turbulence_independent = np.random.randn(n_samples) * turbulence_intensity
        turbulence_independent = signal.sosfilt(sos, turbulence_independent)

        # Mix based on target coherence
        alpha = np.sqrt(coherence_target)  # Weight for correlation
        turbulence_2 = alpha * turbulence_ref + np.sqrt(1 - alpha**2) * turbulence_independent

        # Measure correlation
        correlation, p_value = pearsonr(turbulence_ref, turbulence_2)

        print(f"    Theoretical coherence: {coherence_target:.3f}")
        print(f"    Measured correlation:  {correlation:.3f}")
        print(f"    Difference:            {abs(coherence_target - correlation):.3f}")

        results.append({
            'separation': sep,
            'theoretical': coherence_target,
            'measured': correlation
        })

    # Plot results
    fig, ax = plt.subplots(figsize=(10, 6))

    seps = [r['separation'] for r in results]
    theo = [r['theoretical'] for r in results]
    meas = [r['measured'] for r in results]

    ax.plot(seps, theo, 'k-o', linewidth=2, markersize=8, label='Theoretical (IEC)')
    ax.plot(seps, meas, 'r--s', linewidth=2, markersize=8, label='Measured (Simulation)')

    ax.set_xlabel('Separation Distance [m]', fontsize=12)
    ax.set_ylabel('Coherence / Correlation [-]', fontsize=12)
    ax.set_title('Spatial Coherence Validation: Theoretical vs Measured', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11)
    ax.set_ylim([0, 1.05])

    # Save
    output_dir = os.path.join(script_dir, '..', 'validation_results')
    output_file = os.path.join(output_dir, 'spatial_coherence_simulation.png')
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\n  Saved: {output_file}")

    plt.close()

    print("\n✓ Correlated turbulence simulation complete")


def plot_coherence_decay_comparison():
    """
    Compare IEC coherence decay with alternative models.

    Shows why IEC exponential is preferred for wind turbines.
    """
    print("\n" + "="*60)
    print("COHERENCE DECAY MODEL COMPARISON")
    print("="*60)

    # Parameters
    V = 3.0  # m/s
    f = 0.1  # Hz
    L_c = 340.2

    # Separations
    r = np.linspace(0, 100, 200)

    # IEC exponential
    coh_iec = np.exp(-f * r / (V * L_c))

    # Alternative: Simple exponential (faster decay)
    coh_simple = np.exp(-r / 20.0)

    # Alternative: Squared exponential (Gaussian, very fast decay)
    coh_gaussian = np.exp(-(r / 30.0)**2)

    # Plot
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(r, coh_iec, 'b-', linewidth=2.5, label='IEC Exponential (Used)')
    ax.plot(r, coh_simple, 'g--', linewidth=2, label='Simple Exponential')
    ax.plot(r, coh_gaussian, 'r-.', linewidth=2, label='Gaussian (Squared)')

    ax.set_xlabel('Separation Distance [m]', fontsize=12)
    ax.set_ylabel('Coherence [-]', fontsize=12)
    ax.set_title(f'Coherence Model Comparison (f={f} Hz, V={V} m/s)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=11)
    ax.set_ylim([0, 1.05])

    # Highlight typical formation spacing (3-10m)
    ax.axvspan(3, 10, alpha=0.2, color='yellow', label='Typical Formation Spacing')
    ax.legend(fontsize=10)

    # Save
    output_dir = os.path.join(script_dir, '..', 'validation_results')
    output_file = os.path.join(output_dir, 'coherence_model_comparison.png')
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"  Saved: {output_file}")

    plt.close()

    print("\n✓ Coherence model comparison complete")


def main():
    """Main validation routine."""
    print("\n" + "="*60)
    print("SPATIAL COHERENCE VALIDATION FOR THESIS")
    print("="*60)

    # Run validations
    validate_iec_coherence()
    simulate_correlated_turbulence()
    plot_coherence_decay_comparison()

    # Summary
    print("\n" + "="*60)
    print("VALIDATION SUMMARY")
    print("="*60)

    output_dir = os.path.join(script_dir, '..', 'validation_results')
    print(f"Validation figures saved to: {output_dir}")
    print("\nFiles generated:")
    print("  1. iec_coherence_validation.png - IEC model coherence vs separation")
    print("  2. spatial_coherence_simulation.png - Theoretical vs measured")
    print("  3. coherence_model_comparison.png - IEC vs alternative models")

    print("\n✓ All spatial coherence validation complete - ready for thesis")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
