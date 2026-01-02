#!/usr/bin/env python3
"""
Wind Turbulence Model Validation Script

Validates von Kármán and Dryden turbulence models against theoretical PSD.
Generates thesis-quality figures comparing:
1. Generated turbulence PSD vs theoretical PSD
2. Time series comparison
3. Statistical properties validation

References:
- MIL-F-8785C: Military Specification - Flying Qualities of Piloted Airplanes
- NASA TP-1313: A Comparison of the von Kármán and Dryden Turbulence Models

Author: Multi-Agent Formation Control Research
Date: 2026-01-02
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.stats import normaltest

# Add scripts directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..', 'agent_control_pkg', 'scripts'))

from wind_turbulence_models import (
    VonKarmanTurbulence,
    DrydenTurbulence,
    TurbulenceParameters,
    create_standard_turbulence
)


def validate_turbulence_model(
    model_name: str,
    turbulence_generator,
    duration: float = 300.0,
    output_dir: str = 'validation_results'
):
    """
    Validate turbulence model by comparing generated vs theoretical PSD.

    Args:
        model_name: 'von Kármán' or 'Dryden'
        turbulence_generator: VonKarmanTurbulence or DrydenTurbulence instance
        duration: Duration of time series for validation [s]
        output_dir: Directory to save validation figures
    """
    print(f"\n{'='*60}")
    print(f"Validating {model_name} Turbulence Model")
    print(f"{'='*60}")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Generate turbulence time series
    print(f"Generating {duration}s turbulence time series...")
    time, u, v, w = turbulence_generator.generate_turbulence(duration)

    # Compute statistics
    print("\nStatistical Properties:")
    print(f"  u: mean={np.mean(u):7.4f} m/s, std={np.std(u):7.4f} m/s")
    print(f"  v: mean={np.mean(v):7.4f} m/s, std={np.std(v):7.4f} m/s")
    print(f"  w: mean={np.mean(w):7.4f} m/s, std={np.std(w):7.4f} m/s")

    # Expected standard deviations from parameters
    params = turbulence_generator.params
    print(f"\nExpected (from parameters):")
    print(f"  σ_u = {params.sigma_u:.4f} m/s")
    print(f"  σ_v = {params.sigma_v:.4f} m/s")
    print(f"  σ_w = {params.sigma_w:.4f} m/s")

    # Normality test (turbulence should be Gaussian)
    stat_u, p_u = normaltest(u)
    stat_v, p_v = normaltest(v)
    stat_w, p_w = normaltest(w)
    print(f"\nNormality Test (p-value > 0.05 for Gaussian):")
    print(f"  u: p={p_u:.4f} {'✓ Gaussian' if p_u > 0.05 else '✗ Non-Gaussian'}")
    print(f"  v: p={p_v:.4f} {'✓ Gaussian' if p_v > 0.05 else '✗ Non-Gaussian'}")
    print(f"  w: p={p_w:.4f} {'✓ Gaussian' if p_w > 0.05 else '✗ Non-Gaussian'}")

    # Compute PSD using Welch's method
    print("\nComputing PSD using Welch's method...")
    dt = params.dt
    fs = 1.0 / dt
    nperseg = min(2048, len(u) // 4)  # Use 1/4 of data for each segment

    freq_u, psd_u = signal.welch(u, fs=fs, nperseg=nperseg)
    freq_v, psd_v = signal.welch(v, fs=fs, nperseg=nperseg)
    freq_w, psd_w = signal.welch(w, fs=fs, nperseg=nperseg)

    # Get theoretical PSD
    freq_theoretical = np.logspace(-2, np.log10(fs/2), 200)
    psd_u_theory, psd_v_theory, psd_w_theory = turbulence_generator.get_theoretical_psd(freq_theoretical)

    # Create validation plots
    print("Generating validation figures...")

    # Figure 1: PSD Comparison (3 subplots)
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle(f'{model_name} Turbulence Model - PSD Validation', fontsize=14, fontweight='bold')

    components = ['u (longitudinal)', 'v (lateral)', 'w (vertical)']
    psd_generated = [psd_u, psd_v, psd_w]
    psd_theory = [psd_u_theory, psd_v_theory, psd_w_theory]
    freq_generated = [freq_u, freq_v, freq_w]

    for i, (ax, comp, psd_gen, psd_th) in enumerate(zip(axes, components, psd_generated, psd_theory)):
        # Plot theoretical PSD
        ax.loglog(freq_theoretical, psd_th, 'k-', linewidth=2, label='Theoretical')
        # Plot generated PSD
        ax.loglog(freq_generated[i], psd_gen, 'r--', linewidth=1.5, alpha=0.7, label='Generated')

        ax.set_xlabel('Frequency [Hz]')
        ax.set_ylabel(f'PSD [{chr(956)}² s²/Hz]')  # μ = m/s
        ax.set_title(f'{comp} component')
        ax.grid(True, which='both', alpha=0.3)
        ax.legend()

    plt.tight_layout()
    psd_filename = os.path.join(output_dir, f'{model_name.lower().replace(" ", "_")}_psd_validation.png')
    plt.savefig(psd_filename, dpi=300, bbox_inches='tight')
    print(f"  Saved: {psd_filename}")

    # Figure 2: Time Series
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle(f'{model_name} Turbulence Model - Time Series', fontsize=14, fontweight='bold')

    time_plot = time[:int(60/dt)]  # Plot first 60 seconds
    components_data = [u[:len(time_plot)], v[:len(time_plot)], w[:len(time_plot)]]
    sigmas = [params.sigma_u, params.sigma_v, params.sigma_w]

    for ax, comp, data, sigma in zip(axes, components, components_data, sigmas):
        ax.plot(time_plot, data, 'b-', linewidth=0.5, alpha=0.7)
        ax.axhline(y=3*sigma, color='r', linestyle='--', linewidth=1, alpha=0.5, label=f'±3σ ({3*sigma:.2f} m/s)')
        ax.axhline(y=-3*sigma, color='r', linestyle='--', linewidth=1, alpha=0.5)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)

        ax.set_ylabel(f'{comp} [m/s]')
        ax.grid(True, alpha=0.3)
        ax.legend()

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    time_filename = os.path.join(output_dir, f'{model_name.lower().replace(" ", "_")}_timeseries.png')
    plt.savefig(time_filename, dpi=300, bbox_inches='tight')
    print(f"  Saved: {time_filename}")

    # Figure 3: Statistical validation (histogram)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle(f'{model_name} Turbulence Model - Gaussian Distribution Check', fontsize=14, fontweight='bold')

    for ax, comp, data, sigma in zip(axes, components, [u, v, w], sigmas):
        # Histogram
        n, bins, patches = ax.hist(data, bins=50, density=True, alpha=0.6, color='blue', edgecolor='black')

        # Theoretical Gaussian
        mu = np.mean(data)
        x = np.linspace(data.min(), data.max(), 100)
        gaussian = (1/(sigma * np.sqrt(2*np.pi))) * np.exp(-0.5*((x-mu)/sigma)**2)
        ax.plot(x, gaussian, 'r-', linewidth=2, label=f'Gaussian(μ={mu:.3f}, σ={sigma:.3f})')

        ax.set_xlabel(f'{comp} [m/s]')
        ax.set_ylabel('Probability Density')
        ax.set_title(f'{comp} component')
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    hist_filename = os.path.join(output_dir, f'{model_name.lower().replace(" ", "_")}_histogram.png')
    plt.savefig(hist_filename, dpi=300, bbox_inches='tight')
    print(f"  Saved: {hist_filename}")

    print(f"\n✓ Validation complete for {model_name}")

    return {
        'time': time,
        'u': u,
        'v': v,
        'w': w,
        'freq_u': freq_u,
        'psd_u': psd_u,
        'freq_v': freq_v,
        'psd_v': psd_v,
        'freq_w': freq_w,
        'psd_w': psd_w
    }


def compare_models(vk_results, dr_results, output_dir: str = 'validation_results'):
    """
    Compare von Kármán and Dryden models side-by-side.

    Args:
        vk_results: Validation results from von Kármán model
        dr_results: Validation results from Dryden model
        output_dir: Directory to save comparison figures
    """
    print(f"\n{'='*60}")
    print("Comparing von Kármán vs Dryden Models")
    print(f"{'='*60}")

    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle('Turbulence Model Comparison: von Kármán vs Dryden', fontsize=14, fontweight='bold')

    components = ['u (longitudinal)', 'v (lateral)', 'w (vertical)']
    vk_psds = [vk_results['psd_u'], vk_results['psd_v'], vk_results['psd_w']]
    vk_freqs = [vk_results['freq_u'], vk_results['freq_v'], vk_results['freq_w']]
    dr_psds = [dr_results['psd_u'], dr_results['psd_v'], dr_results['psd_w']]
    dr_freqs = [dr_results['freq_u'], dr_results['freq_v'], dr_results['freq_w']]

    for i, (ax, comp) in enumerate(zip(axes, components)):
        ax.loglog(vk_freqs[i], vk_psds[i], 'b-', linewidth=2, alpha=0.7, label='von Kármán')
        ax.loglog(dr_freqs[i], dr_psds[i], 'r--', linewidth=2, alpha=0.7, label='Dryden')

        ax.set_xlabel('Frequency [Hz]')
        ax.set_ylabel(f'PSD [m²/s²/Hz]')
        ax.set_title(f'{comp} component')
        ax.grid(True, which='both', alpha=0.3)
        ax.legend()

    plt.tight_layout()
    comparison_filename = os.path.join(output_dir, 'vonkarman_vs_dryden_comparison.png')
    plt.savefig(comparison_filename, dpi=300, bbox_inches='tight')
    print(f"  Saved: {comparison_filename}")

    print("\n✓ Model comparison complete")


def main():
    """Main validation routine."""
    print("\n" + "="*60)
    print("WIND TURBULENCE MODEL VALIDATION")
    print("MIL-F-8785C Standard Compliance Check")
    print("="*60)

    # Standard parameters for small drones at low altitude
    params = TurbulenceParameters(
        L_u=50.0,      # Integral length scale [m]
        L_v=25.0,      # Half of L_u (typical)
        L_w=10.0,      # Vertical scale (low altitude)
        sigma_u=0.3,   # 10% TI at 3 m/s mean wind
        sigma_v=0.3,   # Isotropic turbulence
        sigma_w=0.3,
        V=3.0,         # Mean wind speed [m/s]
        dt=0.1         # 10 Hz sampling
    )

    print("\nTurbulence Parameters:")
    print(f"  Integral length scales: L_u={params.L_u}m, L_v={params.L_v}m, L_w={params.L_w}m")
    print(f"  Turbulence intensities: σ_u={params.sigma_u}m/s, σ_v={params.sigma_v}m/s, σ_w={params.sigma_w}m/s")
    print(f"  Mean wind speed: V={params.V}m/s")
    print(f"  Turbulence Intensity (TI): {params.sigma_u/params.V:.1%}")
    print(f"  Sampling rate: {1/params.dt:.0f} Hz")

    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'validation_results')
    os.makedirs(output_dir, exist_ok=True)

    # Validate von Kármán model
    vk = VonKarmanTurbulence(params)
    vk_results = validate_turbulence_model('von Kármán', vk, duration=300.0, output_dir=output_dir)

    # Validate Dryden model
    dr = DrydenTurbulence(params)
    dr_results = validate_turbulence_model('Dryden', dr, duration=300.0, output_dir=output_dir)

    # Compare models
    compare_models(vk_results, dr_results, output_dir)

    print("\n" + "="*60)
    print("VALIDATION SUMMARY")
    print("="*60)
    print(f"Validation figures saved to: {output_dir}")
    print("\nFiles generated:")
    print("  1. von_karman_psd_validation.png - PSD comparison (3 components)")
    print("  2. von_karman_timeseries.png - Time series (first 60s)")
    print("  3. von_karman_histogram.png - Gaussian distribution check")
    print("  4. dryden_psd_validation.png - PSD comparison (3 components)")
    print("  5. dryden_timeseries.png - Time series (first 60s)")
    print("  6. dryden_histogram.png - Gaussian distribution check")
    print("  7. vonkarman_vs_dryden_comparison.png - Side-by-side PSD comparison")
    print("\n✓ All validation complete - ready for thesis inclusion")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
